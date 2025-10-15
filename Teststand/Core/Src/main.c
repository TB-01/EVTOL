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



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// --- Hall current sensors ---
#define I1_SENS_mV_PER_A   24u   // 24 mV/A
#define I2_SENS_mV_PER_A   24u

static volatile int32_t  g_i1_mA = 0;
static volatile int32_t  g_i2_mA = 0;

static volatile uint32_t g_i1_zero_mV = 1650, g_i2_zero_mV = 1650;  // used if I_USE_VDD_HALF_AS_ZERO==0

static volatile uint32_t g_stream_period_ms = 0;
static uint32_t          g_next_stream_ms   = 0;
static uint8_t           g_stream_dest_id   = 0x0A; // default = PC

static uint32_t hb_next_ms = 0;

// Tacho characteristics (adjust PULSES_PER_REV after testing)
#define PERIOD_US_MIN       20u       // reject >12.5 kHz (too fast / glitches)
#define PERIOD_US_MAX       200000u   // reject <5 Hz (too slow / timeout-ish)
#define EMA_SHIFT           2u        // 1/4 smoothing;  old = (3*old + new)/4

#define PULSES_PER_REV 14
#define MAGNETS                 28
#define POLE_PAIRS              (MAGNETS/2)   // = 14
#define PULSES_PER_ELEC_REV     1             // try 1 first; if RPM is off by ×3 or ×6, set to 3 or 6
#define TACH_PULSES_PER_MECHREV (POLE_PAIRS * PULSES_PER_ELEC_REV)  // 14, 42, or 84 typically

static inline uint32_t ema_u32(uint32_t old, uint32_t x) {
    return (old == 0u) ? x : ((old * ((1u<<EMA_SHIFT)-1u) + x) >> EMA_SHIFT);
}


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

typedef struct {
    uint32_t last;
    uint32_t period_ticks;
    uint8_t  have;
} ic_ch_t;

static volatile uint32_t ic1_last = 0, ic2_last = 0;
static volatile uint32_t ic1_period_us = 0, ic2_period_us = 0;
static volatile uint8_t  ic1_lock = 0, ic2_lock = 0;

static volatile uint32_t g_ic_last[2] = {0,0};
static volatile ic_ch_t g_ic[2];  // [0]=CH1 (PA5), [1]=CH2 (PB3)
static volatile uint32_t ic_irq_count = 0;  // keep this (telemetry)
static volatile uint32_t ic1_irq = 0, ic2_irq = 0;
static const uint32_t TIMER_HZ = 1000000u; // 1 MHz after PSC=47

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

static inline void current_calibrate_zero()
{
	uint64_t acc1=0, acc2=0; uint32_t ok=0;
	uint32_t vref_cal = *VREFINT_CAL_ADDR;
	if (vref_cal < 1000 || vref_cal > 3000) vref_cal = 1500;

	for (uint8_t i=0; i<20; ++i){
		uint16_t raw_ref = g_adc[ADC_IDX_VREF];
		if (!raw_ref) continue;
		uint32_t vdd_mV = (3300u * vref_cal) / raw_ref;
		acc1 += (((uint64_t)g_adc[ADC_IDX_I1] * vdd_mV + 2047) / 4095u);
		acc2 += (((uint64_t)g_adc[ADC_IDX_I2] * vdd_mV + 2047) / 4095u);
		ok++;
	}
	if (ok){
		g_i1_zero_mV = (uint32_t)(acc1/ok);
		g_i2_zero_mV = (uint32_t)(acc2/ok);
	}

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

    // --- VDD from factory calibration ---
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
    int32_t dv2_mV = (int32_t)node_i2_mV - (int32_t)g_i2_zero_mV;

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
			else if (cmd == CMD_CALIB_I_ZERO) {

				current_calibrate_zero();

			    uint8_t ack[32]; uint8_t *aw = ack;
			    aw = tlv_put_u32(aw, TLV_CMD_CODE,   cmd);
			    aw = tlv_put_u32(aw, TLV_CMD_RESULT, 0);
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

static uint32_t tach_hz_from_period_us(uint32_t us) {
        return (us ? (1000000u + us/2) / us : 0u);  // rounded
}

static void telemetry_service(void)
{
    if (g_stream_period_ms == 0) return;

    /*
    static uint32_t last_ic_irqs = 0, last_ic_check_ms = 0;
    if ((int32_t)(now - last_ic_check_ms) >= 300) {
        if (ic_irq_count == last_ic_irqs) ic_has_lock = 0;
        last_ic_irqs = ic_irq_count;
        last_ic_check_ms = now;
    }*/

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

    if (g_last_raw_valid) tw = tlv_put_i32(tw, TLV_LOAD_RAW, g_last_raw);

    // --- RPM from captures (both channels)





    uint32_t hz1 = tach_hz_from_period_us(ic1_period_us);
    uint32_t hz2 = tach_hz_from_period_us(ic2_period_us);

    uint32_t rpm_ch2 = (hz1 ? (hz1 * 60u) / TACH_PULSES_PER_MECHREV : 0u);
    uint32_t rpm_ch1 = (hz2 ? (hz2 * 60u) / TACH_PULSES_PER_MECHREV : 0u);

    // Map to ESC1/ESC2 as you intend (swap temporarily if your wiring is crossed)
    tw = tlv_put_u32(tw, TLV_ESC_RPM1, rpm_ch1);
    tw = tlv_put_u32(tw, TLV_ESC_RPM2, rpm_ch2);
    tw = tlv_put_u32(tw, 0xA0, ic1_period_us);
    tw = tlv_put_u32(tw, 0xA1, ic2_period_us);
    tw = tlv_put_u32(tw, 0xA2, ic1_irq);
    tw = tlv_put_u32(tw, 0xA3, ic2_irq);



    // new voltage TLVs (always include)
    adc_compute_mv();
    tw = tlv_put_u32(tw, TLV_VIN1_MV, g_vin1_mV);
    tw = tlv_put_u32(tw, TLV_VIN2_MV, g_vin2_mV);

    /*
    tw = tlv_put_u32(tw, 0x90, g_adc[ADC_IDX_I1]);   // RAW_PA0 (I1)
    tw = tlv_put_u32(tw, 0x91, g_adc[ADC_IDX_I2]);   // RAW_PA1 (I2)
    tw = tlv_put_u32(tw, 0x92, g_adc[ADC_IDX_V1]);   // RAW_PA3 (VIN1 node)
    tw = tlv_put_u32(tw, 0x93, g_adc[ADC_IDX_V2]);   // RAW_PA4 (VIN2 node)
    tw = tlv_put_u32(tw, 0x94, g_adc[ADC_IDX_VREF]); // RAW_VREFINT
    */

    tw = tlv_put_i32(tw, TLV_I1_MA, g_i1_mA);
    tw = tlv_put_i32(tw, TLV_I2_MA, g_i2_mA);

    // send telemetry to whoever requested streaming
    extern int comm_send(uint8_t rx_id, uint8_t tx_id, uint8_t msg_type,
                         uint8_t flags, uint8_t reqid, const uint8_t *payload, uint16_t payload_len);

    comm_send(g_stream_dest_id, 0x20/*MCU id*/, MT_TEL_A, 0x01, 0, t, (uint16_t)(tw - t));
}

static void heartbeat_service(void)
{
    const uint32_t period_ms = 1000; // adjust as you like (>= 50ms recommended)
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - hb_next_ms) < 0) return;
    hb_next_ms = now + period_ms;

    // Minimal payload: device uptime
    uint8_t p[8];
    uint8_t *w = p;
    w = tlv_put_u32(w, TLV_TS_MS, now);

    // Send to PC (rx=0x0A), from MCU (tx=0x20)
    comm_send(0x0A, 0x20, MT_PING, 0x01, 0, p, (uint16_t)(w - p));
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    uint32_t cap, prev, diff;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        cap  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        prev = ic1_last; ic1_last = cap;
        diff = cap - prev;                          // unsigned, wrap-safe
        if (diff < PERIOD_US_MIN || diff > PERIOD_US_MAX) return;
        ic1_period_us = ema_u32(ic1_period_us, diff);
        ic1_lock = 1; ic_irq_count++; ic1_irq++;

    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        cap  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        prev = ic2_last; ic2_last = cap;
        diff = cap - prev;                          // unsigned, wrap-safe
        if (diff < PERIOD_US_MIN || diff > PERIOD_US_MAX) return;
        ic2_period_us = ema_u32(ic2_period_us, diff);
        ic2_lock = 1; ic_irq_count++; ic2_irq++;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // if you want, you can detect “no pulse” here (timeout)
        // e.g., zero ic_has_lock after some ms without captures
    }
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

  esc_init();

  // apply initial µs (will be overwritten by commands)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1050);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1050);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  adc_try_start();          // non-fatal start (sets g_adc_ok)

  comm_init(&huart1);
  uint32_t last_hello = HAL_GetTick();

  hx711_init();

  //Calibrate Current ADC after all Inits are complete
  current_calibrate_zero();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  comm_poll();
	  telemetry_service();
	  heartbeat_service();
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
