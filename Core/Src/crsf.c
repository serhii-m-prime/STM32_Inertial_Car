#include "crsf.h"
#include "usart.h"
#include "vehicle_config.h"

#define RX_BUFFER_SIZE 4096
#define CRSF_MAX_FRAME_SIZE 64

CRSF_State_t crsf;
volatile uint8_t crsf_rx_buffer[RX_BUFFER_SIZE];
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
//static uint32_t broken_frames_counter = 0;
//static uint32_t valid_frames_counter = 0;

/* CRC8 table */
static const uint8_t crsf_crc8_tab[256] = {
		0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
		0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
		0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
		0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
		0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
		0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
		0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
		0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
		0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
		0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
		0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
		0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
		0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
		0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
		0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
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
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)crsf_rx_buffer, RX_BUFFER_SIZE);
}

void CRSF_UpdateFailsafe(uint32_t current_time_ms) {
    if (crsf.is_connected &&
        (current_time_ms - crsf.last_valid_frame_time > vehicle_config.failsafe_timeout_ms)) {
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

static bool CRSF_ParseByte(uint8_t b) {
	bool rc_frame_ready = false;
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
//                	valid_frames_counter++;
                    // Update channels
                    if (type == CRSF_FRAMETYPE_RC && expected_len == 24) {
                        CRSF_UnpackChannels(&frame_buf[3]);
                        crsf.last_valid_frame_time = HAL_GetTick();
                        crsf.is_connected = true;
                        rc_frame_ready = true;
                    }
                    // LQ
                    else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) {
                        crsf.rssi_dbm = -(int8_t)frame_buf[3];
                        crsf.link_quality = frame_buf[5];

                        crsf.last_valid_frame_time = HAL_GetTick();
                        crsf.is_connected = true;
                    }
                }
//                else {
//                	broken_frames_counter++;
//                }

                parser_state = CRSF_STATE_SYNC;
                frame_idx = 0;
            }
            break;
    }
    return rc_frame_ready;
}

bool CRSF_Update(void) {
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
	    __HAL_UART_CLEAR_OREFLAG(&huart2);
	}

	uint16_t rx_head = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (rx_head >= RX_BUFFER_SIZE) rx_head = 0;

    bool any_rc_frame_parsed = false;

    while (rx_tail != rx_head) {
        /* If at least one byte completes a valid RC frame, raise the flag */
		if (CRSF_ParseByte(crsf_rx_buffer[rx_tail])) {
			any_rc_frame_parsed = true;
		}
        rx_tail++;
        if (rx_tail >= RX_BUFFER_SIZE) rx_tail = 0;
    }

    return any_rc_frame_parsed;
}

uint16_t CRSF_GetPWM(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;
    int32_t raw = crsf.raw_channels[channel_index];
    int32_t pwm = ((raw - 172) * 1000) / 1639 + 1000;
    if (pwm < 1000) return 1000;
    if (pwm > 2000) return 2000;
    return (uint16_t)pwm;
}
