#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief USART1 发送完成回调函数
 */
static void(*uart_tx_complete_callback)(void) = NULL;
/**
 * @brief UART5 接收完成回调函数
 */
static void(*uart_rx_complete_callback)(void) = NULL;
/**
 * @brief UART5 错误回调函数
 */
static void(*uart_error_callback)(void) = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

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
 * @brief 注册 UART 发送完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart_tx_complete_callback = callback;
    }
}

/**
 * @brief 注册 UART 接收完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart5) {
        uart_rx_complete_callback = callback;
    }
}

/**
 * @brief 注册 UART 错误回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart5) {
        uart_error_callback = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL UART 发送完成回调入口
 * @param huart 触发回调的 UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        if(uart_tx_complete_callback != NULL) {
            uart_tx_complete_callback();
        }
    }
}

/**
 * @brief HAL UART 接收完成回调入口
 * @param huart 触发回调的 UART 句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart5) {
        if(uart_rx_complete_callback != NULL) {
            uart_rx_complete_callback();
        }
    }
}

/**
 * @brief HAL UART 错误回调入口
 * @param huart 触发回调的 UART 句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        if(uart_tx_complete_callback != NULL) {
            uart_tx_complete_callback();
        }
    }

    if(huart == &huart5) {
        if(uart_error_callback != NULL) {
            uart_error_callback();
        }
    }
}
