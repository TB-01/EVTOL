/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm.h"
#include "protocol.h"
#include "hx711.h"
#include "adc.h"   // hadc, MX_ADC_Init()



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMCLK 48000000
#define PRESCALER 480
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t IC_Val1Mot1 = 0;
uint32_t IC_Val2Mot1 = 0;
//uint32_t IC_Val1Mot2 = 0;
//uint32_t IC_Val2Mot2 = 0;
uint32_t DiffMot1 = 0;
//uint32_t DiffMot2 = 0;
int first_cap_flagMot1 = 0;
//int first_cap_flagMot2 = 0;
float freqMot1 = 0;
//float freqMot2 = 0;

// --- Hall current sensors ---
#define I1_SENS_mV_PER_A   24u   // 24 mV/A
#define I2_SENS_mV_PER_A   24u

static volatile int32_t  g_i1_mA = 0;
static volatile int32_t  g_i2_mA = 0;

static volatile uint32_t g_i1_zero_mV = 1650, g_i2_zero_mV = 1650;  // used if I_USE_VDD_HALF_AS_ZERO==0

static volatile uint32_t g_stream_period_ms = 0;
static uint32_t          g_next_stream_ms   = 0;
static uint8_t           g_stream_dest_id   = 0x0A; // default = PC

// --- ADC DMA buffer order with Scan FORWARD: [PA0, PA1, PA3, PA4, VREFINT] ---
enum { ADC_IDX_I1=0, ADC_IDX_I2=1, ADC_IDX_V1=2, ADC_IDX_V2=3, ADC_IDX_VREF=4, ADC_COUNT=5 };
__attribute__((aligned(4))) static volatile uint16_t g_adc[ADC_COUNT];
static uint8_t g_adc_ok = 0;

// Results (millivolts)
static volatile uint32_t g_vin1_mV = 0;
static volatile uint32_t g_vin2_mV = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t millis(void){ return HAL_GetTick(); }

extern TIM_HandleTypeDef htim1;

static uint16_t g_esc1_us = 1050;
static uint16_t g_esc2_us = 1050;

// HX711 last valid sample
static int32_t  g_last_raw = 0;
static uint8_t  g_last_raw_valid = 0;

static volatile uint16_t ic_last = 0;
static volatile uint32_t ic_period_us = 0;   // last measured period in microseconds
static volatile uint32_t ic_irq_count = 0;
static volatile uint8_t  ic_has_lock = 0;    // 1 once we got a valid period

static inline void esc_apply_hw(void){
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_esc1_us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, g_esc2_us);
}

static void esc_init(void){
  // start PWM outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  esc_apply_hw();
}

static void esc_set_us(uint32_t ch, uint32_t us){
  if (us < 1050) us = 1050;
  if (us > 1940) us = 1940;
  if (ch == 1) g_esc1_us = (uint16_t)us;
  else         g_esc2_us = (uint16_t)us;
  esc_apply_hw();
}

// Factory VREFINT calibration (STM32F0): 3.3V at 30°C
#define R_HIGH1 180000u  // PA3 divider high resistor (ohms)
#define R_LOW1   10000u  // PA3 divider low resistor (ohms)
#define R_HIGH2 180000u  // PA4
#define R_LOW2   10000u

#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR  ((uint16_t*)0x1FFFF7BA)  // STM32F0 ref manual
#endif
#define ADC_FULL_SCALE 4095u


static inline void adc_try_start(void){
    HAL_ADC_Stop_DMA(&hadc);  // in case something was half-started
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) { g_adc_ok = 0; return; }
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)g_adc, ADC_COUNT) != HAL_OK)   { g_adc_ok = 0; return; }
    __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
    g_adc_ok = 1;
}

static inline void adc_compute_mv(void)
{
    if (!g_adc_ok) return;

    // Fixed order: [0]=PA0, [1]=PA1, [2]=PA3, [3]=PA4, [4]=VREFINT
    uint16_t raw_i1  = g_adc[ADC_IDX_I1];
    uint16_t raw_i2  = g_adc[ADC_IDX_I2];
    uint16_t raw_v1  = g_adc[ADC_IDX_V1];
    uint16_t raw_v2  = g_adc[ADC_IDX_V2];
    uint16_t raw_ref = g_adc[ADC_IDX_VREF];
    if (raw_ref == 0) return;  // avoid div/0 while VREFINT wakes up

    // --- VDD from factory calibration (same approach you already use) ---
    uint32_t vref_cal = *VREFINT_CAL_ADDR;           // ~1500 @ 3.3V
    if (vref_cal < 1000 || vref_cal > 3000) vref_cal = 1500;
    uint32_t vdd_mV = (3300u * vref_cal) / raw_ref;

    // --- Node voltages (mV) at each ADC pin (64-bit to avoid overflow later) ---
    uint32_t node_i1_mV = (uint32_t)(((uint64_t)raw_i1 * vdd_mV + 2047) / 4095u);
    uint32_t node_i2_mV = (uint32_t)(((uint64_t)raw_i2 * vdd_mV + 2047) / 4095u);
    uint32_t node_v1_mV = (uint32_t)(((uint64_t)raw_v1 * vdd_mV + 2047) / 4095u);
    uint32_t node_v2_mV = (uint32_t)(((uint64_t)raw_v2 * vdd_mV + 2047) / 4095u);

    // --- External voltages through dividers (19:1) ---
    const uint32_t k1 = (R_HIGH1 + R_LOW1), k2 = (R_HIGH2 + R_LOW2);
    g_vin1_mV = (uint32_t)(((uint64_t)node_v1_mV * k1 + R_LOW1/2) / R_LOW1);
    g_vin2_mV = (uint32_t)(((uint64_t)node_v2_mV * k2 + R_LOW2/2) / R_LOW2);


    //Current Calculation
    int32_t dv1_mV = (int32_t)node_i1_mV - (int32_t)g_i1_zero_mV;
    int32_t dv2_mV = (int32_t)node_i2_mV - (int32_t)g_i1_zero_mV;

    // I(mA) = (dv_mV * 1000) / (mV per A)
    g_i1_mA = (int32_t)(((int64_t)dv1_mV * 1000) / I1_SENS_mV_PER_A);
    g_i2_mA = (int32_t)(((int64_t)dv2_mV * 1000) / I2_SENS_mV_PER_A);
}


void comm_on_frame(const comm_frame_t *f)
{
	// Handle Ping
    if (f->msg_type == MT_PING) {
        uint64_t host_ms64 = 0;
        (void)tlv_find_u64(f->payload, f->payload_len, TLV_TS_HOST_MS, &host_ms64);

        uint8_t buf[24]; uint8_t *w = buf;
        w = tlv_put_u64(w, TLV_TS_HOST_MS, host_ms64);   // echo host 64-bit time
        w = tlv_put_u32(w, TLV_TS_MS, HAL_GetTick());    // device uptime (u32)

        comm_send(f->tx_id, f->rx_id, MT_PING, 0x01, 0, buf, (uint16_t)(w - buf));
        return;
    }
    // Handle Command
    if (f->msg_type == MT_COMMAND) {
            // Parse the command code (u32)
            uint32_t cmd = 0xFFFFFFFFu;
            (void)tlv_find_u32(f->payload, f->payload_len, TLV_CMD_CODE, &cmd);

            uint32_t result = 0; // OK by default

			if (cmd == CMD_ECHO) {
				// nothing else to do
			}
			else if (cmd == CMD_SNAPSHOT) {
				// Build one telemetry frame (MT_TEL_A) with a timestamp TLV for now
				uint8_t t[16]; uint8_t *tw = t;

				// Read HX711 (timeout e.g. 50ms; pick 10SPS or 80SPS rate on your module pin)
				int32_t raw = 0;
				int ok = hx711_read_raw(&raw, 50);

				tw = tlv_put_u32(tw, TLV_TS_MS, HAL_GetTick());

				if (ok) {
					// reuse u32 helper but cast the bytes of raw (two's complement)
					tw = tlv_put_i32(tw, TLV_LOAD_RAW, raw);  // if you don’t have tlv_put_i32 yet, I can add it; else use tlv_put_u32 on (uint32_t)raw
				}
				// Send telemetry to the command sender
				comm_send(f->tx_id, f->rx_id, MT_TEL_A, 0x01, /*reqid=*/0, t, (uint16_t)(tw - t));
			}
			else if (cmd == CMD_STREAM) {
			    uint32_t per = 0;
			    (void)tlv_find_u32(f->payload, f->payload_len, TLV_STREAM_PERIOD_MS, &per);

			    g_stream_period_ms = per;
			    g_next_stream_ms   = HAL_GetTick() + 1;
			    g_stream_dest_id   = f->tx_id;          // send to whoever asked

			    // result = OK (0)
			}
			else if (cmd == CMD_SET_ESC) {
			    uint32_t ch = 0, us = 0;
			    (void)tlv_find_u32(f->payload, f->payload_len, TLV_ESC_CH, &ch);
			    (void)tlv_find_u32(f->payload, f->payload_len, TLV_ESC_US, &us);

			    uint32_t result = 0; // OK
			    if ((ch != 1u) && (ch != 2u)) {
			        result = 2; // invalid channel
			    } else if (us < 800 || us > 2200) { // loose input guard; hard clamp to 1050..1940
			        result = 1; // out of plausible range
			    } else {
			        esc_set_us(ch, us); // clamps internally to 1050..1940
			    }

			    uint8_t ack[24]; uint8_t *aw = ack;
			    aw = tlv_put_u32(aw, TLV_CMD_CODE,   cmd);
			    aw = tlv_put_u32(aw, TLV_CMD_RESULT, result);
			    aw = tlv_put_u32(aw, TLV_ESC_CH,     ch);
			    aw = tlv_put_u32(aw, TLV_ESC_US,     us);
			    comm_send(f->tx_id, f->rx_id, MT_COMMAND_ACK, 0x01, f->reqid, ack, (uint16_t)(aw - ack));
			    return;
			}

			else {
				result = 3; // BAD/unknown command
			}

			// Always send an ACK for MT_COMMAND, echoing the CMD_CODE
			uint8_t ack[16]; uint8_t *aw = ack;
			aw = tlv_put_u32(aw, TLV_CMD_CODE,   cmd);
			aw = tlv_put_u32(aw, TLV_CMD_RESULT, result);
			comm_send(f->tx_id, f->rx_id, MT_COMMAND_ACK, 0x01, f->reqid, ack, (uint16_t)(aw - ack));
			return;

        }
    // ...
}

static void telemetry_service(void)
{
    if (g_stream_period_ms == 0) return;

    uint32_t now = HAL_GetTick();
    // not time yet?
    if ((int32_t)(now - g_next_stream_ms) < 0) return;
    g_next_stream_ms = now + g_stream_period_ms;

    extern int hx711_dout_level(void);

    // read HX711 if ready (non-blocking)
    int32_t raw = 0; int ok = 0;
    if (hx711_dout_level() == 0) {        // data ready low
        ok = hx711_read_raw(&raw, 0);     // no wait
        if (ok) { g_last_raw = raw; g_last_raw_valid = 1; }
    }

    uint8_t t[96]; uint8_t *tw = t;
    tw = tlv_put_u32(tw, TLV_TS_MS, now);
    tw = tlv_put_u32(tw, TLV_ESC1_US, g_esc1_us);
    tw = tlv_put_u32(tw, TLV_ESC2_US, g_esc2_us);
    tw = tlv_put_u32(tw, TLV_IC_IRQS,      ic_irq_count);
    tw = tlv_put_u32(tw, TLV_IC_PERIOD_US, ic_period_us);
    if (g_last_raw_valid) tw = tlv_put_i32(tw, TLV_LOAD_RAW, g_last_raw);

    // RPM from capture
    if (ic_has_lock && ic_period_us) {
        const uint32_t P = 14; // rotor magnets (try 7 if you discover they mean pole pairs)
        uint32_t hz  = 1000000u / ic_period_us;
        uint32_t rpm = (hz * 60u) / (P);
        tw = tlv_put_u32(tw, TLV_ESC_RPM1, rpm);
    }


    // new voltage TLVs (always include)
    adc_compute_mv();
    tw = tlv_put_u32(tw, TLV_VIN1_MV, g_vin1_mV);
    tw = tlv_put_u32(tw, TLV_VIN2_MV, g_vin2_mV);

    tw = tlv_put_u32(tw, 0x90, g_adc[0]); // RAW_PA3
    tw = tlv_put_u32(tw, 0x91, g_adc[1]); // RAW_PA4
    tw = tlv_put_u32(tw, 0x92, g_adc[2]); // RAW_VREFINT

    tw = tlv_put_i32(tw, TLV_I1_MA, g_i1_mA);
    tw = tlv_put_i32(tw, TLV_I2_MA, g_i2_mA);

    // send telemetry to whoever requested streaming
    extern int comm_send(uint8_t rx_id, uint8_t tx_id, uint8_t msg_type,
                         uint8_t flags, uint8_t reqid, const uint8_t *payload, uint16_t payload_len);

    comm_send(g_stream_dest_id, 0x20/*MCU id*/, MT_TEL_A, 0x01, 0, t, (uint16_t)(tw - t));
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(first_cap_flagMot1 == 0)
		{
			IC_Val1Mot1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			first_cap_flagMot1 = 1;
		}
		else
		{
			IC_Val2Mot1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			if(IC_Val2Mot1 > IC_Val1Mot1)
			{
				DiffMot1 = IC_Val2Mot1 - IC_Val1Mot1;
			}
			else if(IC_Val1Mot1 > IC_Val2Mot1)
			{
				DiffMot1 = (0xffffffff - IC_Val1Mot1) + IC_Val2Mot1;
			}

			float refClock = TIMCLK/(PRESCALER);
			freqMot1 = refClock/DiffMot1;

			__HAL_TIM_SET_COUNTER(htim,0);
			first_cap_flagMot1 = 0;
		}
	}

	/*
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(first_cap_flagMot2 == 0)
		{
			IC_Val1Mot2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			first_cap_flagMot2 = 1;
		}
		else
		{
			IC_Val2Mot2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			if(IC_Val2Mot2 > IC_Val1Mot2)
			{
				DiffMot2 = IC_Val2Mot2 - IC_Val1Mot2;
			}
			else if(IC_Val1Mot2 > IC_Val2Mot2)
			{
				DiffMot2 = (0xffffffff - IC_Val1Mot2) + IC_Val2Mot2;
			}

			float refClock = TIMCLK/(PRESCALAR);
			frequencyMot2 = refClock/DiffMot2;

			__HAL_TIM_SET_COUNTER(htim,0);
			first_cap_flagMot2 = 0;
		}
	}
	*/
}







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  esc_init();

  // apply initial µs (will be overwritten by commands)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1050);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1050);

  adc_try_start();          // non-fatal start (sets g_adc_ok)

  comm_init(&huart1);
  uint32_t last_hello = HAL_GetTick();

  hx711_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  comm_poll();
	  telemetry_service();
	  /*
	  if (HAL_GetTick() - last_hello >= 1000) {
	    last_hello += 1000;
	    static const char msg[] = "hello, pc\r\n";
	    uart_tx_dma((const uint8_t*)msg, sizeof(msg)-1);
	  }*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
