#include "crsf.h"
#include "usart.h"

#define RX_BUFFER_SIZE 4096
#define CRSF_MAX_FRAME_SIZE 64

CRSF_State_t crsf;
uint8_t crsf_rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_tail = 0;

// States
typedef enum {
    CRSF_STATE_SYNC = 0,
    CRSF_STATE_LEN,
    CRSF_STATE_PAYLOAD
} CRSF_ParserState_t;

static CRSF_ParserState_t parser_state = CRSF_STATE_SYNC;
static uint8_t frame_buf[CRSF_MAX_FRAME_SIZE];
static uint8_t frame_idx = 0;
static uint8_t expected_len = 0;
static uint32_t broken_frames_counter = 0;
static uint32_t valid_frames_counter = 0;

/* CRC8 table */
static const uint8_t crsf_crc8_tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x4D, 0x98, 0x32, 0xE7, 0xB3, 0x66, 0xCC, 0x19, 0x64, 0xB1, 0x1B, 0xCE, 0x9A, 0x4F, 0xE5, 0x30,
    0x1F, 0xCA, 0x60, 0xB5, 0xE1, 0x34, 0x9E, 0x4B, 0x36, 0xE3, 0x49, 0x9C, 0xC8, 0x1D, 0xB7, 0x62,
    0xE9, 0x3C, 0x96, 0x43, 0x17, 0xC2, 0x68, 0xBD, 0xC0, 0x15, 0xBF, 0x6A, 0x3E, 0xEB, 0x41, 0x94,
    0xBB, 0x6E, 0xC4, 0x11, 0x45, 0x90, 0x3A, 0xEF, 0x92, 0x47, 0xED, 0x38, 0x6C, 0xB9, 0x13, 0xC6,
    0x9A, 0x4F, 0xE5, 0x30, 0x64, 0xB1, 0x1B, 0xCE, 0xB3, 0x66, 0xCC, 0x19, 0x4D, 0x98, 0x32, 0xE7,
    0xC8, 0x1D, 0xB7, 0x62, 0x36, 0xE3, 0x49, 0x9C, 0xE1, 0x34, 0x9E, 0x4B, 0x1F, 0xCA, 0x60, 0xB5,
    0x3E, 0xEB, 0x41, 0x94, 0xC0, 0x15, 0xBF, 0x6A, 0x17, 0xC2, 0x68, 0xBD, 0xE9, 0x3C, 0x96, 0x43,
    0x6C, 0xB9, 0x13, 0xC6, 0x92, 0x47, 0xED, 0x38, 0x45, 0x90, 0x3A, 0xEF, 0xBB, 0x6E, 0xC4, 0x11,
    0xD7, 0x02, 0xA8, 0x7D, 0x29, 0xFC, 0x56, 0x83, 0xFE, 0x2B, 0x81, 0x54, 0x00, 0xD5, 0x7F, 0xAA,
    0x85, 0x50, 0xFA, 0x2F, 0x7B, 0xAE, 0x04, 0xD1, 0xAC, 0x79, 0xD3, 0x06, 0x52, 0x87, 0x2D, 0xF8,
    0x73, 0xA6, 0x0C, 0xD9, 0x8D, 0x58, 0xF2, 0x27, 0x5A, 0x8F, 0x25, 0xF0, 0xA4, 0x71, 0xDB, 0x0E,
    0x21, 0xF4, 0x5E, 0x8B, 0xDF, 0x0A, 0xA0, 0x75, 0x08, 0xDD, 0x77, 0xA2, 0xF6, 0x23, 0x89, 0x5C
};

static uint8_t CRSF_CalcCRC(uint8_t *data, uint8_t len) {
	uint8_t crc = 0;
	    while (len >= 4) {
	        crc = crsf_crc8_tab[crc ^ *data++];
	        crc = crsf_crc8_tab[crc ^ *data++];
	        crc = crsf_crc8_tab[crc ^ *data++];
	        crc = crsf_crc8_tab[crc ^ *data++];
	        len -= 4;
	    }
	    while (len--) {
	        crc = crsf_crc8_tab[crc ^ *data++];
	    }
	    return crc;
}

void CRSF_Init(void) {
	// Init state
    crsf.is_connected = false;
    crsf.link_quality = 0;
    crsf.rssi_dbm = -120;
    crsf.last_valid_frame_time = 0;
    rx_tail = 0;

    parser_state = CRSF_STATE_SYNC;
    frame_idx = 0;

    for (int i=0; i<16; i++) crsf.raw_channels[i] = 992;

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_UART_Receive_DMA(&huart2, crsf_rx_buffer, RX_BUFFER_SIZE);
}

void CRSF_UpdateFailsafe(uint32_t current_time_ms) {
    if (crsf.is_connected && (current_time_ms - crsf.last_valid_frame_time > CRSF_FAILSAFE_TIME)) {
        crsf.is_connected = false;
        crsf.link_quality = 0;
        crsf.raw_channels[CRSF_MAP_THROTTLE] = 172; // 0
        crsf.raw_channels[CRSF_MAP_STEERING] = 992; // center
    }
}

static void CRSF_UnpackChannels(uint8_t* payload) {
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 0;

    for (int n = 0; n < 16; n++) {
        while (bitsMerged < 11) {
            readValue |= (uint32_t)payload[readByteIndex++] << bitsMerged;
            bitsMerged += 8;
        }
        crsf.raw_channels[n] = readValue & 0x07FF;
        readValue >>= 11;
        bitsMerged -= 11;
    }
}

static void CRSF_ParseByte(uint8_t b) {
    uint32_t current_time = HAL_GetTick();

    switch (parser_state) {
        case CRSF_STATE_SYNC:
            if (b == CRSF_START_BYTE) {
                frame_buf[0] = b;
                frame_idx = 1;
                parser_state = CRSF_STATE_LEN;
            }
            break;

        case CRSF_STATE_LEN:
            if (b >= 2 && b <= 62) {
                frame_buf[1] = b;
                expected_len = b;
                frame_idx = 2;
                parser_state = CRSF_STATE_PAYLOAD;
            } else {
                parser_state = CRSF_STATE_SYNC;
            }
            break;

        case CRSF_STATE_PAYLOAD:
            frame_buf[frame_idx++] = b;

            if (frame_idx >= expected_len + 2) {
            	// Last byte on frame
                uint8_t type = frame_buf[2];
                uint8_t crc_calc = CRSF_CalcCRC(&frame_buf[2], expected_len - 1);
                uint8_t crc_received = frame_buf[expected_len + 1];

                if (crc_calc == crc_received) { // valid sign
                	valid_frames_counter++;
                    // Update channels
                    if (type == CRSF_FRAMETYPE_RC && expected_len == 24) {
                        CRSF_UnpackChannels(&frame_buf[3]);
                        crsf.last_valid_frame_time = current_time;
                        crsf.is_connected = true;
                    }
                    // LQ
                    else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) {
                        crsf.rssi_dbm = -(int8_t)frame_buf[3];
                        crsf.link_quality = frame_buf[5];

                        crsf.last_valid_frame_time = current_time;
                        crsf.is_connected = true;
                    }
                } else {
                	broken_frames_counter++;
                }

                parser_state = CRSF_STATE_SYNC;
                frame_idx = 0;
            }
            break;
    }
}

void CRSF_Update(void) {
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
	    __HAL_UART_CLEAR_OREFLAG(&huart2);
	}

	uint16_t rx_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (rx_head >= RX_BUFFER_SIZE) rx_head = 0;

    while (rx_tail != rx_head) {
        CRSF_ParseByte(crsf_rx_buffer[rx_tail]);
        rx_tail++;
        if (rx_tail >= RX_BUFFER_SIZE) rx_tail = 0;
    }
}

uint16_t CRSF_GetPWM(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;
    int32_t raw = crsf.raw_channels[channel_index];
    int32_t pwm = ((raw - 172) * 1000) / 1639 + 1000;
    if (pwm < 1000) return 1000;
    if (pwm > 2000) return 2000;
    return (uint16_t)pwm;
}
