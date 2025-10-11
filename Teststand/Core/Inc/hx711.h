/*
 * hx711.h
 *
 *  Created on: Oct 10, 2025
 *      Author: E_T_R
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#pragma once
#include <stdint.h>
#include "main.h"   // for GPIO port/pin defs

// === Pin mapping (adjust to your wiring) ===
#define HX711_SCK_GPIO      GPIOB
#define HX711_SCK_PIN       GPIO_PIN_0
#define HX711_DOUT_GPIO     GPIOB
#define HX711_DOUT_PIN      GPIO_PIN_1

// Gain selection for NEXT conversion: 128 (A) uses 1 extra clock
enum { HX711_GAIN128 = 1 /* pulses after 24 bits */ };

void hx711_init(void);

// Blocking read: waits until DOUT low (ready) or timeout,
// then clocks out 24 bits and sets gain for next conversion.
// Returns 1 on success, 0 on timeout.
int  hx711_read_raw(int32_t *out, uint32_t timeout_ms);
int  hx711_dout_level(void);

#endif /* INC_HX711_H_ */
