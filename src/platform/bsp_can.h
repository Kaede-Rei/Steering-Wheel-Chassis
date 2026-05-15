#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "main.h"
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define can can_interface

#define BSP_CAN_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_DLC, "Invalid DLC") \
    X(FILTER_CONFIG_FAILED, "CAN Filter Config Failed") \
    X(START_FAILED, "CAN Start Failed") \
    X(NOTIFICATION_FAILED, "CAN Notification Activation Failed") \
    X(TX_MAILBOX_TIMEOUT, "CAN TX Mailbox Timeout") \
    X(TX_FAILED, "CAN TX Failed") \
    X(RX_FAILED, "CAN RX Failed") \
    X(NO_CALLBACK_SLOT, "No CAN RX Callback Slot")

#define X(name, str) BSP_CAN_##name,
typedef enum {
    BSP_CAN_STATUS_TABLE
} BspCanStatus;
#undef X

typedef void (*BspCanRxCallback)(CAN_HandleTypeDef* hcan,
                                 const CAN_RxHeaderTypeDef* header,
                                 const uint8_t data[8],
                                 void* user);

#define X(name, str) BspCanStatus name;
extern const struct BspCanInterface {
    struct {
        BSP_CAN_STATUS_TABLE
    };
    BspCanStatus (*filter_init)(void);
    BspCanStatus (*start)(CAN_HandleTypeDef* hcan);
    BspCanStatus (*send)(CAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);
    BspCanStatus (*register_rx_callback)(CAN_HandleTypeDef* hcan, BspCanRxCallback callback, void* user);
    const char* (*error_code_to_str)(BspCanStatus status);
} can_interface;
#undef X

// ! ========================= 兼 容 旧 代 码 的 函 数 声 明 ========================= ! //

BspCanStatus can_filter_init(void);
BspCanStatus can_start(CAN_HandleTypeDef* hcan);
BspCanStatus can_send(CAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);
BspCanStatus can_register_rx_callback(CAN_HandleTypeDef* hcan, BspCanRxCallback callback, void* user);
const char* can_error_code_to_str(BspCanStatus status);

#endif /* _BSP_CAN_H_ */
