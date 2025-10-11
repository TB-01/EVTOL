/*
 * hx711.c
 *
 *  Created on: Oct 10, 2025
 *      Author: E_T_R
 */

#include "hx711.h"

// small ~sub-microsecond delay; exact value not critical.
// We only need to ensure SCK high time is short (<60us).
static inline void hx711_udelay_ticks(int n){
  for (volatile int i = 0; i < n; i++) __NOP();
}
static inline void hx711_sck_high(void){ HAL_GPIO_WritePin(HX711_SCK_GPIO, HX711_SCK_PIN, GPIO_PIN_SET); }
static inline void hx711_sck_low (void){ HAL_GPIO_WritePin(HX711_SCK_GPIO, HX711_SCK_PIN, GPIO_PIN_RESET); }
static inline int  hx711_dout(void){ return HAL_GPIO_ReadPin(HX711_DOUT_GPIO, HX711_DOUT_PIN) ? 1 : 0; }

// public: simple pin read (used by telemetry_service)
int hx711_dout_level(void){ return hx711_dout(); }

static inline void short_pulse(void){
  for (volatile int i=0;i<30;i++) __NOP();
}

void hx711_init(void){
  // Ensure SCK is low at idle; high >60us would power down the HX711.
  hx711_sck_low();
}

int hx711_read_raw(int32_t *out, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  // 1) wait for data ready (DOUT low) or timeout
  while (hx711_dout()){
    if ((HAL_GetTick() - t0) >= timeout_ms) return 0;
  }

  // 2) clock out 24 bits, MSB first
  uint32_t v = 0;
  for (int i = 0; i < 24; i++){
    hx711_sck_high();
    hx711_udelay_ticks(30);            // short high pulse
    v = (v << 1) | (hx711_dout() ? 1u : 0u);
    hx711_sck_low();
    hx711_udelay_ticks(30);
  }

  // 3) set GAIN for NEXT conversion (128 = +1 extra clock)
  hx711_sck_high();
  hx711_udelay_ticks(30);
  hx711_sck_low();
  hx711_udelay_ticks(30);

  // 4) sign-extend 24-bit two's complement to 32-bit
  if (v & 0x800000u) v |= 0xFF000000u;
  *out = (int32_t)v;
  return 1;
}
