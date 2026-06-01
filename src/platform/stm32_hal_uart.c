#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief UART 回调函数注册表
 * @param huart UART 句柄
 * @param tx_complete 发送完成回调函数指针
 * @param rx_complete 接收完成回调函数指针
 * @param rx_event 接收事件回调函数指针，参数为本次接收的数据长度
 * @param error 错误回调函数指针
 */
typedef struct {
    UART_HandleTypeDef* huart;
    void (*tx_complete)(void);
    void (*rx_complete)(void);
    void (*rx_event)(uint16_t size);
    void (*error)(void);
} UartCallbackRegistry;

/**
 * @brief USART1 发送完成回调函数指针
 *
 * 日志模块使用 USART1 DMA 异步发送；
 * 发送完成后通过该指针通知日志队列继续输出
 */
static UartCallbackRegistry s_uart_callbacks[] = {
    { .huart = &huart1, .tx_complete = NULL, .rx_complete = NULL, .rx_event = NULL, .error = NULL },
    { .huart = &huart5, .tx_complete = NULL, .rx_complete = NULL, .rx_event = NULL, .error = NULL },
    { .huart = &huart10, .tx_complete = NULL, .rx_complete = NULL, .rx_event = NULL, .error = NULL },
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 通过 USART1 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
static UartCallbackRegistry* s_find_registry(UART_HandleTypeDef* huart);

/**
 * @brief 通过 USART1 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool uart1_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0 || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

/**
 * @brief 通过 USART7 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool uart1_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0 || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK;
}

/**
 * @brief 通过 USART7 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool uart7_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0 || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart7, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK;
}

/**
 * @brief 通过 USART10 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool uart10_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0 || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart10, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK;
}

/**
 * @brief 启动 UART 中断接收
 * @param huart UART 句柄
 * @param data 接收缓冲区
 * @param len 接收长度
 * @return bool `true` 表示启动成功
 */
bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    if(huart == NULL || data == NULL || len == 0) {
        return false;
    }

    return HAL_UART_Receive_IT(huart, data, len) == HAL_OK;
}

/**
 * @brief 启动 UART 空闲行接收 DMA
 * @param huart UART 句柄
 * @param data 接收缓冲区
 * @param len 接收长度
 * @return bool `true` 表示启动成功
 */
bool uart_receive_to_idle_dma(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef status;

    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, data, len);
    if(status != HAL_OK) {
        return false;
    }

    if(huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

/**
 * @brief 中止 UART 中断接收
 * @param huart UART 句柄
 * @return bool `true` 表示中止成功
 */
bool uart_abort_receive_it(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive_IT(huart) == HAL_OK;
}

/**
 * @brief 中止 UART 空闲行接收 DMA
 * @param huart UART 句柄
 * @return bool `true` 表示中止成功
 */
bool uart_abort_receive_dma(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive(huart) == HAL_OK;
}

/**
 * @brief 注册 UART 发送完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL) {
        registry->tx_complete = callback;
    }
}

/**
 * @brief 注册 UART 接收完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL) {
        registry->rx_complete = callback;
    }
}

/**
 * @brief 注册 UART 接收事件回调
 * @param huart UART 句柄
 * @param callback 回调函数，参数为本次接收的数据长度
 */
void uart_register_rx_event_callback(UART_HandleTypeDef* huart, void (*callback)(uint16_t size)) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL) {
        registry->rx_event = callback;
    }
}

/**
 * @brief 注册 UART 错误回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL) {
        registry->error = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL UART 发送完成回调入口
 * @param huart 触发回调的 UART 句柄
 */
static UartCallbackRegistry* s_find_registry(UART_HandleTypeDef* huart) {
    uint32_t i;

    if(huart == NULL) {
        return NULL;
    }

    for(i = 0u; i < (sizeof(s_uart_callbacks) / sizeof(s_uart_callbacks[0])); i++) {
        if(s_uart_callbacks[i].huart == huart) {
            return &s_uart_callbacks[i];
        }
    }

    return NULL;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL && registry->tx_complete != NULL) {
        registry->tx_complete();
    }
}

/**
 * @brief HAL UART 接收完成回调入口
 * @param huart 触发回调的 UART 句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL && registry->rx_complete != NULL) {
        registry->rx_complete();
    }
}

/**
 * @brief HAL UART 接收事件回调入口
 * @param huart 触发回调的 UART 句柄
 * @param Size 本次接收的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry != NULL && registry->rx_event != NULL) {
        registry->rx_event(Size);
    }
}

/**
 * @brief HAL UART 错误回调入口
 * @param huart 触发回调的 UART 句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    UartCallbackRegistry* registry = s_find_registry(huart);
    if(registry == NULL) {
        return;
    }

    if(huart == &huart1 && registry->tx_complete != NULL) {
        registry->tx_complete();
    }

    if(registry->error != NULL) {
        registry->error();
    }
}
