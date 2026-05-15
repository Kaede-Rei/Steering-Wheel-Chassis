#include "bsp_can.h"
#include <stddef.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// ! ========================= 变 量 声 明 ========================= ! //

#define bc can_interface

#define BSP_CAN_TX_TIMEOUT_MS 2U
#define BSP_CAN_RX_CALLBACK_SLOT_NUM 4U
#define BSP_CAN_ALL_TX_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

typedef struct {
    CAN_HandleTypeDef* hcan;
    BspCanRxCallback callback;
    void* user;
} BspCanRxCallbackSlot;

static BspCanRxCallbackSlot s_rx_slots[BSP_CAN_RX_CALLBACK_SLOT_NUM];

#define X(name, str) .name = BSP_CAN_##name,
const struct BspCanInterface can_interface = {
    {
        BSP_CAN_STATUS_TABLE
    },
    .filter_init = can_filter_init,
    .start = can_start,
    .send = can_send,
    .register_rx_callback = can_register_rx_callback,
    .error_code_to_str = can_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static BspCanStatus bsp_can_config_one_filter(CAN_HandleTypeDef* hcan,
    uint32_t filter_bank,
    uint32_t slave_start_filter_bank);
static void bsp_can_dispatch_rx(CAN_HandleTypeDef* hcan,
    const CAN_RxHeaderTypeDef* header,
    const uint8_t data[8]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BspCanStatus can_filter_init(void) {
    BspCanStatus status;

    /* bxCAN 双 CAN 设备常见约定：Bank 0~13 分配给 CAN1，Bank 14~27 分配给 CAN2。 */
    status = bsp_can_config_one_filter(&hcan1, 0U, 14U);
    if(status != bc.OK) return status;

    status = bsp_can_config_one_filter(&hcan2, 14U, 14U);
    if(status != bc.OK) return status;

    return bc.OK;
}

BspCanStatus can_start(CAN_HandleTypeDef* hcan) {
    if(hcan == NULL) return bc.INVALID_PARAM;

    if(HAL_CAN_Start(hcan) != HAL_OK) return bc.START_FAILED;

    if(HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return bc.NOTIFICATION_FAILED;
    }

    return bc.OK;
}

BspCanStatus can_send(CAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    if(hcan == NULL || data == NULL) return bc.INVALID_PARAM;
    if(len > 8U) return bc.INVALID_DLC;

    CAN_TxHeaderTypeDef tx_header = { 0 };
    uint32_t tx_mailbox = 0U;

    tx_header.StdId = id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    const uint32_t start_tick = HAL_GetTick();
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        if((HAL_GetTick() - start_tick) >= BSP_CAN_TX_TIMEOUT_MS) {
            (void)HAL_CAN_AbortTxRequest(hcan, BSP_CAN_ALL_TX_MAILBOXES);
            return bc.TX_MAILBOX_TIMEOUT;
        }
    }

    if(HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t*)data, &tx_mailbox) != HAL_OK) {
        (void)HAL_CAN_AbortTxRequest(hcan, BSP_CAN_ALL_TX_MAILBOXES);
        return bc.TX_FAILED;
    }

    HAL_Delay(1);
    return bc.OK;
}

BspCanStatus can_register_rx_callback(CAN_HandleTypeDef* hcan, BspCanRxCallback callback, void* user) {
    if(hcan == NULL || callback == NULL) return bc.INVALID_PARAM;

    for(uint8_t i = 0U; i < BSP_CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback == callback) {
            s_rx_slots[i].user = user;
            return bc.OK;
        }
    }

    for(uint8_t i = 0U; i < BSP_CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].callback == NULL) {
            s_rx_slots[i].hcan = hcan;
            s_rx_slots[i].callback = callback;
            s_rx_slots[i].user = user;
            return bc.OK;
        }
    }

    return bc.NO_CALLBACK_SLOT;
}

#define X(name, str) case BSP_CAN_##name: return str;
const char* can_error_code_to_str(BspCanStatus status) {
    switch(status) {
        BSP_CAN_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

/**
 * HAL 接收中断回调：
 * 1. 从 FIFO0 取出所有积压报文，避免 500Hz 控制任务中反馈堆积；
 * 2. 不在 platform 层硬编码 DM/DJI，而是通过注册表分发给上层设备模块。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        memset(rx_data, 0, sizeof(rx_data));
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }
        bsp_can_dispatch_rx(hcan, &rx_header, rx_data);
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static BspCanStatus bsp_can_config_one_filter(CAN_HandleTypeDef* hcan,
    uint32_t filter_bank,
    uint32_t slave_start_filter_bank) {
    if(hcan == NULL) return bc.INVALID_PARAM;

    CAN_FilterTypeDef filter_config = { 0 };

    /* 接收所有标准帧/扩展帧，具体业务过滤放到设备解析层。 */
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = 0x0000U;
    filter_config.FilterIdLow = 0x0000U;
    filter_config.FilterMaskIdHigh = 0x0000U;
    filter_config.FilterMaskIdLow = 0x0000U;
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config.FilterBank = filter_bank;
    filter_config.SlaveStartFilterBank = slave_start_filter_bank;
    filter_config.FilterActivation = ENABLE;

    if(HAL_CAN_ConfigFilter(hcan, &filter_config) != HAL_OK) {
        return bc.FILTER_CONFIG_FAILED;
    }

    return bc.OK;
}

static void bsp_can_dispatch_rx(CAN_HandleTypeDef* hcan,
    const CAN_RxHeaderTypeDef* header,
    const uint8_t data[8]) {
    for(uint8_t i = 0U; i < BSP_CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback != NULL) {
            s_rx_slots[i].callback(hcan, header, data, s_rx_slots[i].user);
        }
    }
}
