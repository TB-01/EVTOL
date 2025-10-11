#include "comm.h"
#include "stm32f0xx_hal.h"
#include <string.h>

/*** configuration ***/
// size of the RX circular buffer (power of two is convenient later)
#ifndef RX_DMA_SIZE
#define RX_DMA_SIZE 512
#endif

#ifndef TX_MAX_SIZE
#define TX_MAX_SIZE 256
#endif

/*** module state ***/
static UART_HandleTypeDef *g_huart = NULL;  // which UART we’re bound to
static uint8_t  rx_dma_buf[RX_DMA_SIZE];    // where DMA writes incoming bytes
static volatile uint16_t rx_dma_head = 0;   // snapshot of DMA write index
static uint16_t rx_cons = 0;                // how far we have consumed (parser will use later)

typedef enum { S_FIND_START=0, S_READ_LEN, S_READ_FRAME } parse_state_t;
static parse_state_t pstate = S_FIND_START;
static uint16_t start_pos = 0;  // where we saw Start
static uint16_t want_len  = 0;  // total frame length (from Start to end), per length field

volatile uint32_t g_frame_count = 0;  // <-- add this line here (global, not static)

//TX Buffer and State
static uint8_t tx_buf[TX_MAX_SIZE];
static volatile uint8_t tx_busy = 0;
static void on_tx_done(void){ tx_busy = 0; }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){ if (huart==g_huart) on_tx_done(); }


/*** tiny helpers ***/

// --- little-endian writers ---
static void put_u16le(uint8_t *p, uint16_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put_u32le(uint8_t *p, uint32_t v){ p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

// --- tiny TLV writer, used for ping payload ---
uint8_t* tlv_put_u32(uint8_t *w, uint8_t type, uint32_t v){
    *w++ = type; *w++ = 4; *w++ = 0;
    *w++ = (uint8_t)v; *w++ = (uint8_t)(v>>8); *w++ = (uint8_t)(v>>16); *w++ = (uint8_t)(v>>24);
    return w;
}

uint8_t* tlv_put_i32(uint8_t *w, uint8_t type, int32_t v){
    *w++ = type; *w++ = 4; *w++ = 0;
    *w++ = (uint8_t)(v      );
    *w++ = (uint8_t)(v >>  8);
    *w++ = (uint8_t)(v >> 16);
    *w++ = (uint8_t)(v >> 24);
    return w;
}

// --- CRC-32C (Castagnoli), small bitwise version ---
static uint32_t crc32c(const uint8_t *p, size_t n){
    uint32_t crc = 0xFFFFFFFFu;
    while(n--){
        crc ^= *p++;
        for(int k=0;k<8;k++)
            crc = (crc>>1) ^ (0x82F63B78u & (-(int)(crc & 1)));
    }
    return ~crc;
}

// 1) ask the DMA "head" position (where it has written up to)
//    NDTR = remaining transfers; head = (size - NDTR) modulo size
static inline uint16_t dma_head_now(void){
    uint16_t remaining = __HAL_DMA_GET_COUNTER(g_huart->hdmarx);
    return (uint16_t)((RX_DMA_SIZE - remaining) & (RX_DMA_SIZE - 1));
}

// 2) read one byte from the circular buffer at index 'idx' (no consume)
static inline uint8_t circ_peek(uint16_t idx){
    return rx_dma_buf[idx % RX_DMA_SIZE];
}

// copy N bytes from circular buffer at idx into linear dst
static void circ_read(uint16_t idx, uint8_t *dst, uint16_t n){
    while (n--) { *dst++ = rx_dma_buf[idx++ % RX_DMA_SIZE]; }
}

// how many bytes are available from 'from' to current head (modulo)
static int have_bytes(uint16_t from, uint16_t need){
    int32_t avail = (int32_t)((rx_dma_head - from + RX_DMA_SIZE) % RX_DMA_SIZE);
    return (avail >= (int32_t)need);
}

// read little-endian u16 at circular position
static uint16_t read_u16le_circ(uint16_t idx){
    uint8_t b0 = circ_peek(idx);
    uint8_t b1 = circ_peek((uint16_t)(idx+1));
    return (uint16_t)(b0 | ((uint16_t)b1<<8));
}


int uart_tx_dma(const uint8_t *data, uint16_t len){
    if (!g_huart) return -1;
    while (tx_busy) { /* single-buffer, simple */ }
    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(g_huart, (uint8_t*)data, len) != HAL_OK){
        tx_busy = 0;
        return -2;
    }
    return 0;
}

int tlv_find_u32(const uint8_t *p, uint16_t len, uint8_t want_type, uint32_t *out)
{
    uint16_t i = 0;
    while (i + 3 <= len) {
        uint8_t t = p[i];
        uint16_t L = (uint16_t)(p[i+1] | (p[i+2] << 8));
        i += 3;
        if (i + L > len) break;
        if (t == want_type && L == 4) {
            const uint8_t *v = &p[i];
            *out = (uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
            return 1;
        }
        i += L;
    }
    return 0;
}

uint8_t* tlv_put_u64(uint8_t *w, uint8_t type, uint64_t v){
    *w++ = type; *w++ = 8; *w++ = 0;
    // little-endian
    *w++ = (uint8_t)(v      );
    *w++ = (uint8_t)(v >>  8);
    *w++ = (uint8_t)(v >> 16);
    *w++ = (uint8_t)(v >> 24);
    *w++ = (uint8_t)(v >> 32);
    *w++ = (uint8_t)(v >> 40);
    *w++ = (uint8_t)(v >> 48);
    *w++ = (uint8_t)(v >> 56);
    return w;
}

int tlv_find_u64(const uint8_t *p, uint16_t len, uint8_t want_type, uint64_t *out)
{
    uint16_t i = 0;
    while (i + 3 <= len) {
        uint8_t t = p[i];
        uint16_t L = (uint16_t)(p[i+1] | (p[i+2] << 8));
        i += 3;
        if (i + L > len) break;
        if (t == want_type && L == 8) {
            const uint8_t *v = &p[i];
            uint64_t x =  ((uint64_t)v[0])        |
                         (((uint64_t)v[1]) <<  8) |
                         (((uint64_t)v[2]) << 16) |
                         (((uint64_t)v[3]) << 24) |
                         (((uint64_t)v[4]) << 32) |
                         (((uint64_t)v[5]) << 40) |
                         (((uint64_t)v[6]) << 48) |
                         (((uint64_t)v[7]) << 56);
            *out = x;
            return 1;
        }
        i += L;
    }
    return 0;
}


/*** public API ***/

static uint8_t next_seq(void){ static uint8_t s=0; return ++s; }
enum { COMM_START_BYTE = 0x45, COMM_HDR_SIZE = 8, COMM_VER = 0x01 };

int comm_send(uint8_t rx_id, uint8_t tx_id, uint8_t msg_type,
              uint8_t flags, uint8_t reqid,
              const uint8_t *payload, uint16_t payload_len)
{
    const uint16_t total = 1 + 2 + COMM_HDR_SIZE + payload_len + 4;
    if (total > TX_MAX_SIZE) return -1;

    // ---> reserve the TX buffer BEFORE writing into it
	while (tx_busy) { /* spin or yield */ }
	tx_busy = 1;

    uint8_t *w = tx_buf;
    *w++ = COMM_START_BYTE;
    put_u16le(w, total); w += 2;

    const uint8_t seq = next_seq();
    *w++ = rx_id; *w++ = tx_id; *w++ = msg_type; *w++ = COMM_VER;
    *w++ = 0x01;  /* flags: FINAL bit set */
    *w++ = seq;
    *w++ = reqid;
    *w++ = 0;     /* reserved */

    if (payload_len && payload) memcpy(w, payload, payload_len);

    uint32_t crc = crc32c(tx_buf, 1+2+COMM_HDR_SIZE+payload_len);
    put_u32le(tx_buf + (1+2+COMM_HDR_SIZE+payload_len), crc);

    if (HAL_UART_Transmit_DMA(g_huart, tx_buf, total) != HAL_OK){
        tx_busy = 0; return -2;
    }
    return 0;
}

void comm_init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
    rx_dma_head = 0;
    rx_cons = 0;

    // start continuous RX into our circular buffer
    HAL_UART_Receive_DMA(g_huart, rx_dma_buf, RX_DMA_SIZE);

    // enable IDLE-line interrupt so we get poked when a burst of bytes ends
    __HAL_UART_ENABLE_IT(g_huart, UART_IT_IDLE);
}

void comm_on_uart_idle_irq(void)
{
    // acknowledge the IDLE event and snapshot the new DMA head
    __HAL_UART_CLEAR_IDLEFLAG(g_huart);
    rx_dma_head = dma_head_now();

    // (nothing else yet — we’ll parse in comm_poll() in a later step)
}

static void deliver_checked(const uint8_t *frame, uint16_t len){
    if (len < (1+2+COMM_HDR_SIZE+4)) return;            // min size check
    if (frame[0] != COMM_START_BYTE) return;

    uint16_t L = (uint16_t)(frame[1] | (frame[2]<<8));
    if (L != len) return;

    // CRC check: over Start..end-of-payload
    uint32_t crc_rx = (uint32_t)frame[len-4] | ((uint32_t)frame[len-3]<<8) |
                      ((uint32_t)frame[len-2]<<16) | ((uint32_t)frame[len-1]<<24);
    uint32_t crc = crc32c(frame, len-4);
    if (crc != crc_rx) return;

    // Build parsed view
    comm_frame_t f;
    f.rx_id = frame[3]; f.tx_id = frame[4]; f.msg_type = frame[5]; f.ver = frame[6];
    f.flags = frame[7]; f.seq = frame[8]; f.reqid = frame[9]; f.rsv = frame[10];
    f.payload = &frame[11];
    f.payload_len = (uint16_t)(len - (1+2+COMM_HDR_SIZE) - 4);

    comm_on_frame(&f);
}

void comm_poll(void)
{
	while (rx_cons != rx_dma_head){
	        switch (pstate){

	        case S_FIND_START: {
	            uint8_t b = circ_peek(rx_cons);
	            rx_cons = (uint16_t)((rx_cons + 1) % RX_DMA_SIZE);
	            if (b == 0x45) {  // COMM_START_BYTE later
	                start_pos = (uint16_t)((rx_cons - 1 + RX_DMA_SIZE) % RX_DMA_SIZE);
	                pstate = S_READ_LEN;
	            }
	        } break;

	        case S_READ_LEN: {
	            // need Start + 2 bytes of length available
	            if (!have_bytes(start_pos, 3)) return;  // wait for more
	            uint16_t L = read_u16le_circ((uint16_t)(start_pos + 1));

	            // sanity window: min 15 bytes (Start+Len+8B header+4B CRC), max 512 for now
	            if (L < 15 || L > 512){
	                // bad length -> resync by shifting one byte
	                start_pos = (uint16_t)((start_pos + 1) % RX_DMA_SIZE);
	                pstate = S_FIND_START;
	                break;
	            }
	            want_len = L;
	            pstate = S_READ_FRAME;
	        } break;

	        case S_READ_FRAME: {
	            if (!have_bytes(start_pos, want_len)) return;  // not all bytes here yet

	            static uint8_t scratch[TX_MAX_SIZE];
	            if (want_len <= sizeof(scratch)){
	                circ_read(start_pos, scratch, want_len);
	                deliver_checked(scratch, want_len);
	            }

	            // advance past the frame and search again
	            rx_cons = (uint16_t)((start_pos + want_len) % RX_DMA_SIZE);
	            pstate = S_FIND_START;
	        } break;

	        } // switch
	    }
}
