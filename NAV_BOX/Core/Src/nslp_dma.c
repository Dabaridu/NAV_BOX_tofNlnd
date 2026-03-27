#include "nslp_dma.h"
#include <string.h>
#include <stdlib.h>

volatile uint8_t nslp_rx_active = 0;  // Global legacy flag

// Registry of active instances
static nslp_instance_t *instances[NSLP_MAX_INSTANCES] = {0};

static nslp_instance_t* find_instance(UART_HandleTypeDef *huart) {
    for (int i = 0; i < NSLP_MAX_INSTANCES; i++) {
        if (instances[i] && instances[i]->uart == huart) {
            return instances[i];
        }
    }
    return NULL;
}

void nslp_init(nslp_instance_t *inst, UART_HandleTypeDef *huart,
               CRC_HandleTypeDef *hcrc, uint16_t queue_length, uint8_t address) {
    inst->uart = huart;
    inst->crc  = hcrc;
    inst->address = address;

    inst->rx_buffer = malloc(MAX_PACKET_SIZE);
    inst->tx_buffer = malloc(MAX_PACKET_SIZE);
    inst->tx_queue  = malloc(sizeof(nslp_packet_t *) * queue_length);

    inst->tx_queue_length = queue_length;
    inst->tx_head = 0;
    inst->tx_tail = 0;
    inst->tx_count = 0;
    inst->tx_busy = 0;
    inst->rx_callback = NULL;
    inst->rx_active = 0;

    // Register instance
    for (int i = 0; i < NSLP_MAX_INSTANCES; i++) {
        if (!instances[i]) {
            instances[i] = inst;
            break;
        }
    }

    __HAL_UART_ENABLE_IT(inst->uart, UART_IT_IDLE);//USER ADD COMMENT: recieve to idle interup enable
    HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT); //USER ADD COMMENT: half transfer interupt disable
}

void nslp_set_rx_callback(nslp_instance_t *inst, void (*callback)(nslp_packet_t *)) {
    inst->rx_callback = callback;
}

static void start_tx(nslp_instance_t *inst) {
    if (inst->tx_count == 0) return;

    nslp_packet_t *p = inst->tx_queue[inst->tx_tail];
    if (!p || !p->payload || p->size > MAX_PAYLOAD_SIZE) {
        inst->tx_tail = (inst->tx_tail + 1) % inst->tx_queue_length;
        if (inst->tx_count > 0) {
            inst->tx_count--;
        }
        inst->tx_busy = 0;
        return;
    }

    size_t packet_size = HEADER_SIZE + p->size;
    size_t total_size  = FRAME_START_SIZE + packet_size + CHECKSUM_SIZE;
    if (total_size > MAX_PACKET_SIZE) {
        inst->tx_tail = (inst->tx_tail + 1) % inst->tx_queue_length;
        if (inst->tx_count > 0) {
            inst->tx_count--;
        }
        inst->tx_busy = 0;
        return;
    }

    inst->tx_buffer[0] = FRAME_START;
    inst->tx_buffer[1] = inst->address;      // sender
    inst->tx_buffer[2] = p->receiver;        // receiver
    inst->tx_buffer[3] = p->type;
    inst->tx_buffer[4] = p->size;

    memcpy(&inst->tx_buffer[TX_HEADER_OFFSET], p->payload, p->size);

    uint32_t crc = HAL_CRC_Calculate(inst->crc,
                                     (uint32_t *)&inst->tx_buffer[1],
                                     HEADER_SIZE + p->size);
    memcpy(&inst->tx_buffer[TX_HEADER_OFFSET + p->size], &crc, CRC_SIZE);

    inst->tx_busy = 1;
    HAL_UART_Transmit_DMA(inst->uart, inst->tx_buffer, total_size);
}

void nslp_send_packet(nslp_instance_t *inst, nslp_packet_t *packet) {
    if (!packet || packet->size > MAX_PAYLOAD_SIZE ||
        inst->tx_count >= inst->tx_queue_length) return;

    inst->tx_queue[inst->tx_head] = packet;
    inst->tx_head = (inst->tx_head + 1) % inst->tx_queue_length;
    inst->tx_count++;

    if (!inst->tx_busy) {
        start_tx(inst);
    }
}

uint32_t nslp_min_delay(nslp_instance_t *inst) {
    if (!inst || inst->tx_count == 0 || inst->uart == NULL) return UART_TIMEOUT_MS;

    uint32_t baudrate = inst->uart->Init.BaudRate;
    if (baudrate == 0) return 0;

    uint32_t total_time_ms = 0;
    uint16_t idx = inst->tx_tail;

    for (uint16_t i = 0; i < inst->tx_count; i++) {
        nslp_packet_t *p = inst->tx_queue[idx];
        if (!p) continue;

        // Calculate total packet size
        uint16_t packet_size = FRAME_START_SIZE + HEADER_SIZE + p->size + CHECKSUM_SIZE;

        // Convert to bits (assuming 10 bits per byte: 1 start + 8 data + 1 stop)
        uint32_t bits = packet_size * 10;

        // Transmission time in ms
        uint32_t tx_time_ms = (1000UL * bits) / baudrate;

        // Add UART time + fixed 25ms overhead
        total_time_ms += tx_time_ms + 25;

        idx = (idx + 1) % inst->tx_queue_length;
    }

    return total_time_ms + UART_TIMEOUT_MS;
}

void nslp_uart_tx_cplt_handler(UART_HandleTypeDef *huart) {
    nslp_instance_t *inst = find_instance(huart);
    if (!inst) return;

    inst->tx_tail = (inst->tx_tail + 1) % inst->tx_queue_length;
    inst->tx_count--;
    inst->tx_busy = 0;
    start_tx(inst);
}

void nslp_uart_rx_event_handler(UART_HandleTypeDef *huart, uint16_t size) {
    nslp_instance_t *inst = find_instance(huart);
    if (!inst) return;

    nslp_rx_active = 1;
    inst->rx_active = 1;

    if (inst->rx_buffer[0] != FRAME_START) {
        nslp_rx_active = 0;
        inst->rx_active = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
        __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT);
        return;
    }

    uint8_t sender   = inst->rx_buffer[1];
    uint8_t receiver = inst->rx_buffer[2];
    uint8_t type     = inst->rx_buffer[3];
    uint8_t payload_size = inst->rx_buffer[4];

    // Ignore packets not for us (unless broadcast)
    if (receiver != inst->address && receiver != 0xFF) {
        nslp_rx_active = 0;
        inst->rx_active = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
        __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT);
        return;
    }

    uint32_t received_crc;
    memcpy(&received_crc, &inst->rx_buffer[PAYLOAD_OFFSET + payload_size], CRC_SIZE);

    __HAL_CRC_DR_RESET(inst->crc);
    uint32_t computed_crc = HAL_CRC_Calculate(inst->crc,
                                              (uint32_t *)&inst->rx_buffer[1],
                                              HEADER_SIZE + payload_size);

    if (received_crc != computed_crc) {
        nslp_rx_active = 0;
        inst->rx_active = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
        __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT);
        return;
    }

    memcpy(inst->rx_payload, &inst->rx_buffer[PAYLOAD_OFFSET], payload_size);

    inst->rx_packet.sender   = sender;
    inst->rx_packet.receiver = receiver;
    inst->rx_packet.type     = type;
    inst->rx_packet.size     = payload_size;
    inst->rx_packet.payload  = inst->rx_payload;

    if (inst->rx_callback) {
        inst->rx_callback(&inst->rx_packet);
    }

    nslp_rx_active = 0;
    inst->rx_active = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT);
}

void nslp_uart_error_handler(UART_HandleTypeDef *huart) {
    nslp_instance_t *inst = find_instance(huart);
    if (!inst) return;

    nslp_rx_active = 0;
    inst->rx_active = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(inst->uart, inst->rx_buffer, MAX_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(inst->uart->hdmarx, DMA_IT_HT);
}
