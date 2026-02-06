#ifndef NSLP_DMA_H
#define NSLP_DMA_H

#include "stm32f3xx_hal.h"
#include <stddef.h>

// Frame constants
#define FRAME_START             0x7E
#define FRAME_START_SIZE        1
#define HEADER_SIZE             4   // sender + receiver + type + size
#define CHECKSUM_SIZE           4

// Payload and packet sizes
#define MAX_PAYLOAD_SIZE        255
#define MAX_PACKET_SIZE         (FRAME_START_SIZE + HEADER_SIZE + MAX_PAYLOAD_SIZE + CHECKSUM_SIZE)

// Packet building offsets
#define TX_HEADER_OFFSET        5   // [0] FRAME_START, [1] SENDER, [2] RECEIVER, [3] TYPE, [4] SIZE
#define PAYLOAD_OFFSET          (FRAME_START_SIZE + HEADER_SIZE)

// CRC
#define CRC_SIZE                4

// UART timeout (ms) for polling fallback
#define UART_TIMEOUT_MS         130

// Polling mode fallback buffer
#define POLLING_MAX_PAYLOAD     255
#define POLLING_BUFFER_SIZE     (HEADER_SIZE + POLLING_MAX_PAYLOAD + CHECKSUM_SIZE)

// Max instances (can be raised if needed)
#define NSLP_MAX_INSTANCES      4

// Packet structure
typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t type;
    uint8_t size;
    uint8_t *payload;
} nslp_packet_t;

// Instance structure
typedef struct {
    UART_HandleTypeDef *uart;
    CRC_HandleTypeDef  *crc;

    uint8_t *rx_buffer;
    uint8_t *tx_buffer;

    nslp_packet_t **tx_queue;
    uint16_t tx_queue_length;

    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_count;
    uint8_t  tx_busy;

    nslp_packet_t rx_packet;
    uint8_t rx_payload[MAX_PAYLOAD_SIZE];

    void (*rx_callback)(nslp_packet_t *);

    // Instance-specific RX active flag
    uint8_t rx_active;

    // Local address
    uint8_t address;
} nslp_instance_t;

// Global RX activity flag (legacy)
extern volatile uint8_t nslp_rx_active;

// Public API
void nslp_init(nslp_instance_t *inst, UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc,
               uint16_t queue_length, uint8_t address);
void nslp_send_packet(nslp_instance_t *inst, nslp_packet_t *packet);
void nslp_set_rx_callback(nslp_instance_t *inst, void (*callback)(nslp_packet_t *));
nslp_packet_t* receive_packet(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc);

// Utility: estimate minimal TX delay for all queued packets (baudrate read from UART init)
uint32_t nslp_min_delay(nslp_instance_t *inst);

// Dispatcher functions (must be called inside HAL callbacks)
void nslp_uart_tx_cplt_handler(UART_HandleTypeDef *huart);
void nslp_uart_rx_event_handler(UART_HandleTypeDef *huart, uint16_t size);
void nslp_uart_error_handler(UART_HandleTypeDef *huart);

#endif // NSLP_DMA_H
