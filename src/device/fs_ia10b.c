#include "fs_ia10b.h"

#include <string.h>

#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define IBUS_HEADER_0 0x20u
#define IBUS_HEADER_1 0x40u
#define IBUS_CHANNEL_MIN 800u
#define IBUS_CHANNEL_MAX 2200u
#define IBUS_DMA_RX_BUF_LEN 64u
#define IBUS_RX_RESTART_INTERVAL_MS 200u

__attribute__((aligned(32))) static uint8_t s_dma_rx_buf[IBUS_DMA_RX_BUF_LEN];
static uint8_t s_frame[FS_IA10B_IBUS_FRAME_LEN];
static uint8_t s_frame_index = 0u;
static uint32_t s_last_restart_ms = 0u;
static volatile FsIa10bData s_data;
static volatile FsIa10bDebug s_debug;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void ibus_rx_event_callback(uint16_t size);
static void ibus_error_callback(void);
static bool ibus_restart_receive(void);
static void ibus_invalidate_dma_buffer(uint16_t size);
static bool ibus_uart_init_inverted(void);
static void ibus_feed_byte(uint8_t byte);
static bool ibus_check_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);
static bool ibus_channels_in_range(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);
static void ibus_parse_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void ibus_init(void) {
    memset((void*)&s_data, 0, sizeof(s_data));
    memset((void*)&s_debug, 0, sizeof(s_debug));
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));

    s_frame_index = 0u;
    s_last_restart_ms = 0u;

    uart_register_rx_event_callback(&huart5, ibus_rx_event_callback);
    uart_register_error_callback(&huart5, ibus_error_callback);
    (void)ibus_uart_init_inverted();
}

void ibus_maintain(void) {
    uint32_t now = HAL_GetTick();

    if(s_data.valid && (now - s_data.last_update_ms) <= IBUS_RX_RESTART_INTERVAL_MS) {
        return;
    }

    if((now - s_last_restart_ms) < IBUS_RX_RESTART_INTERVAL_MS) {
        return;
    }

    s_frame_index = 0u;
    (void)uart_abort_receive_dma(&huart5);
    (void)ibus_restart_receive();
}

bool ibus_get_data(FsIa10bData* out) {
    uint32_t primask;

    if(out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_data;
    if(primask == 0u) {
        __enable_irq();
    }

    return out->valid;
}

bool ibus_get_debug(FsIa10bDebug* out) {
    uint32_t primask;

    if(out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_debug;
    if(primask == 0u) {
        __enable_irq();
    }

    return true;
}

bool ibus_is_online(uint32_t timeout_ms) {
    uint32_t elapsed;

    if(!s_data.valid) {
        return false;
    }

    elapsed = HAL_GetTick() - s_data.last_update_ms;
    return elapsed <= timeout_ms;
}

uint16_t ibus_get_channel(uint8_t index) {
    uint16_t value = 0u;
    uint32_t primask;

    if(index >= FS_IA10B_CHANNEL_COUNT) {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    value = s_data.channel[index];
    if(primask == 0u) {
        __enable_irq();
    }

    return value;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void ibus_rx_event_callback(uint16_t size) {
    uint16_t i;

    if(size > IBUS_DMA_RX_BUF_LEN) {
        size = IBUS_DMA_RX_BUF_LEN;
    }

    ibus_invalidate_dma_buffer(size);

    for(i = 0u; i < size; ++i) {
        ibus_feed_byte(s_dma_rx_buf[i]);
    }

    ibus_restart_receive();
}

static void ibus_error_callback(void) {
    s_frame_index = 0u;
    s_data.error_count++;

    (void)uart_abort_receive_dma(&huart5);
    ibus_restart_receive();
}

static bool ibus_restart_receive(void) {
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));
    ibus_invalidate_dma_buffer(IBUS_DMA_RX_BUF_LEN);
    s_last_restart_ms = HAL_GetTick();
    return uart_receive_to_idle_dma(&huart5, s_dma_rx_buf, IBUS_DMA_RX_BUF_LEN);
}

static void ibus_invalidate_dma_buffer(uint16_t size) {
    uintptr_t start = ((uintptr_t)s_dma_rx_buf) & ~(uintptr_t)31U;
    uintptr_t end = ((uintptr_t)s_dma_rx_buf + size + 31U) & ~(uintptr_t)31U;

    if(size == 0u) {
        return;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

static bool ibus_uart_init_inverted(void) {
    (void)uart_abort_receive_dma(&huart5);
    (void)HAL_UART_DeInit(&huart5);

    /**
     * 实测这套 FS-iA10B -> UART5_RX 的接线下，
     * i.BUS 帧内容本身是正确的，但 RX 电平极性与普通非反相 UART 假设相反
     *
     * 非反相读取时只能看到稳定乱码；启用 RX 反相后即可稳定解出：
     * 20 40 ... checksum
     *
     * 因此这里固定使用：
     * - 115200
     * - 8N1
     * - RX pin level invert enabled
     */
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

    if(HAL_UART_Init(&huart5) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
        return false;
    }

    memset(s_frame, 0, sizeof(s_frame));
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));
    memset((void*)&s_debug, 0, sizeof(s_debug));
    s_frame_index = 0u;
    ibus_restart_receive();
    return true;
}

static void ibus_feed_byte(uint8_t byte) {
    uint8_t i;

    s_debug.rx_byte_count++;
    s_debug.latest_byte = byte;
    for(i = 0u; i < (FS_IA10B_IBUS_FRAME_LEN - 1u); ++i) {
        s_debug.bytes[i] = s_debug.bytes[i + 1u];
    }
    s_debug.bytes[FS_IA10B_IBUS_FRAME_LEN - 1u] = byte;

    if(s_frame_index == 0u) {
        if(byte == IBUS_HEADER_0) {
            s_debug.header0_count++;
            s_frame[0] = byte;
            s_frame_index = 1u;
        }
        return;
    }

    if(s_frame_index == 1u) {
        if(byte == IBUS_HEADER_1) {
            s_debug.header01_count++;
            s_frame[1] = byte;
            s_frame_index = 2u;
        }
        else if(byte == IBUS_HEADER_0) {
            s_frame[0] = byte;
            s_frame_index = 1u;
        }
        else {
            s_frame_index = 0u;
        }
        return;
    }

    s_frame[s_frame_index] = byte;
    s_frame_index++;

    if(s_frame_index >= FS_IA10B_IBUS_FRAME_LEN) {
        if(ibus_check_frame(s_frame) && ibus_channels_in_range(s_frame)) {
            ibus_parse_frame(s_frame);
        }
        else {
            s_data.error_count++;
        }

        s_frame_index = 0u;
    }
}

static bool ibus_check_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    uint16_t checksum = 0xFFFFu;
    uint16_t rx_checksum;
    uint8_t i;

    if(frame[0] != IBUS_HEADER_0 || frame[1] != IBUS_HEADER_1) {
        return false;
    }

    for(i = 0u; i < 30u; ++i) {
        checksum = (uint16_t)(checksum - frame[i]);
    }

    rx_checksum = (uint16_t)frame[30] | ((uint16_t)frame[31] << 8);
    return checksum == rx_checksum;
}

static bool ibus_channels_in_range(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    uint8_t i;

    for(i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) {
        uint8_t offset = (uint8_t)(2u + i * 2u);
        uint16_t value = (uint16_t)frame[offset] | ((uint16_t)frame[offset + 1u] << 8);

        if(value < IBUS_CHANNEL_MIN || value > IBUS_CHANNEL_MAX) {
            return false;
        }
    }

    return true;
}

static void ibus_parse_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    FsIa10bData data;
    uint8_t i;

    memset(&data, 0, sizeof(data));

    for(i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) {
        uint8_t offset = (uint8_t)(2u + i * 2u);
        data.channel[i] = (uint16_t)frame[offset] | ((uint16_t)frame[offset + 1u] << 8);
    }

    data.valid = true;
    data.last_update_ms = HAL_GetTick();
    data.frame_count = s_data.frame_count + 1u;
    data.error_count = s_data.error_count;

    s_data = data;
}
