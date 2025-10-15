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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm.h"
#include "protocol.h"
#include "hx711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMCLK 48000000
#define PRESCALER 4800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

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
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
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

static inline void esc_apply_hw(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_esc1_us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, g_esc2_us);
}

static void esc_init(void)
{
  // start PWM outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  esc_apply_hw();
}

static void esc_set_us(uint32_t ch, uint32_t us)
{
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


static inline void adc_try_start(void)
{
    HAL_ADC_Stop_DMA(&hadc1);  // in case something was half-started
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { g_adc_ok = 0; return; }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc, ADC_COUNT) != HAL_OK)   { g_adc_ok = 0; return; }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
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
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(first_cap_flagMot1 == 0)
		{
			IC_Val1Mot1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			first_cap_flagMot1 = 1;
		}
		else
		{
			IC_Val2Mot1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  esc_init();

  // apply initial µs (will be overwritten by commands)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1050);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1050);

  adc_try_start();          // non-fatal start (sets g_adc_ok)

  comm_init(&huart2);
  uint32_t last_hello = HAL_GetTick();

  hx711_init();
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN BSP */
  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    comm_poll();
	  telemetry_service();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1050;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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

#ifdef  USE_FULL_ASSERT
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
