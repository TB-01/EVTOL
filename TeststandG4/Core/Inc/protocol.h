/*
 * protocol.h
 *
 *  Created on: Oct 9, 2025
 *      Author: E_T_R
 */

#ifndef SRC_PROTOCOL_H_
#define SRC_PROTOCOL_H_

// Message types
enum {
  MT_PING        = 0x05,
  MT_COMMAND     = 0x0A,
  MT_COMMAND_ACK = 0x0B,
  MT_TEL_A       = 0x65,  // telemetry frame "A"
};

// TLVs
enum {
  TLV_TS_MS       = 0x01, // u32 device uptime
  TLV_TS_HOST_MS  = 0x02, // u64 host epoch ms
  TLV_CMD_CODE    = 0x4F, // u32
  TLV_CMD_RESULT  = 0x50, // u32
  TLV_LOAD_RAW    = 0x30, // i32 raw ADC counts
  TLV_STREAM_PERIOD_MS = 0x40, // u32
  TLV_ESC_CH           = 0x60, // u32: 1 or 2 (keep u32 to reuse tlv_find_u32)
  TLV_ESC_US           = 0x61, // u32: pulse width in microseconds
  TLV_ESC1_US          = 0x62, // u32: current applied CH1 (telemetry)
  TLV_ESC2_US          = 0x63, // u32: current applied CH2 (telemetry)
  TLV_ESC_RPM1         = 0x70,  // u32, motor 1 RPM from speed pin
  TLV_ESC_RPM2         = 0x71,  // u32, motor 2 RPM from speed pin
  TLV_IC_IRQS          = 0x72,  // u32: number of IC interrupts
  TLV_IC_PERIOD_US     = 0x73,  // u32: last captured period in us
  TLV_VIN1_MV          = 0x80,   // u32: PA3 scaled to mV
  TLV_VIN2_MV          = 0x81,   // u32: PA4 scaled to mV
  TLV_I1_MA            = 0x82, // i32: current 1 (mA)
  TLV_I2_MA            = 0x83, // i32: current 2 (mA)
  // (more later)
};

// Commands
enum {
  CMD_ECHO        = 0x00000000,  // request and Echo
  CMD_SNAPSHOT    = 0x00000001,  // request one telemetry frame
  CMD_STREAM   = 0x00000002, //command a telemetry stream
  CMD_SET_ESC  = 0x00000010, // set one ESC channel
};

//Known IDs
enum { DEV_CONTROL_PC=0x0A, DEV_TESTSTAND_MCU=0x20 };

#endif /* SRC_PROTOCOL_H_ */
