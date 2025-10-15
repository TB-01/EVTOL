#pragma once
#include <stdint.h>
#include <stddef.h>
#include "main.h"  // UART_HandleTypeDef



// We’ll fill these later; for now, just declare the functions.
void comm_init(UART_HandleTypeDef *huart);
void comm_on_uart_idle_irq(void);
void comm_poll(void);
int uart_tx_dma(const uint8_t *data, uint16_t len);
// Send one framed message (Start + Len + 8B header + payload + CRC-32C)
int  comm_send(uint8_t rx_id, uint8_t tx_id, uint8_t msg_type,
               uint8_t flags, uint8_t reqid,
               const uint8_t *payload, uint16_t payload_len);

// Tiny TLV writer (used for ping)
uint8_t* tlv_put_u32(uint8_t *w, uint8_t type, uint32_t v);
int tlv_find_u32(const uint8_t *p, uint16_t len, uint8_t want_type, uint32_t *out);

uint8_t* tlv_put_u64(uint8_t *w, uint8_t type, uint64_t v);
int tlv_find_u64(const uint8_t *p, uint16_t len, uint8_t want_type, uint64_t *out);

uint8_t* tlv_put_i32(uint8_t *w, uint8_t type, int32_t v);

// Parsed frame view & app hook (so your code gets neat parameters)
typedef struct {
  uint8_t  rx_id, tx_id, msg_type, ver, flags, seq, reqid, rsv;
  const uint8_t *payload;
  uint16_t payload_len;
} comm_frame_t;

void comm_on_frame(const comm_frame_t *f);


