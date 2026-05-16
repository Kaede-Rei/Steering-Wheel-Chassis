#include "bsp_can.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define bc can_interface

#define BSP_CAN_TX_TIMEOUT_MS 2U
#define BSP_CAN_RX_CALLBACK_SLOT_NUM 4U

typedef struct {
    BspCanHandle* hcan;
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

static uint32_t bsp_can_len_to_dlc(uint8_t len);
static BspCanStatus bsp_can_config_global_filter(BspCanHandle* hcan);
static void bsp_can_dispatch_rx(BspCanHandle* hcan, const BspCanRxHeader* header, const uint8_t data[8]);
static void bsp_can_enable_transceiver(BspCanHandle* hcan);

BspCanStatus can_filter_init(void) {
    BspCanStatus status;

    status = bsp_can_config_global_filter(&hfdcan1);
    if(status != bc.OK) {
        return status;
    }

    status = bsp_can_config_global_filter(&hfdcan2);
    if(status != bc.OK) {
        return status;
    }

    return bc.OK;
}

BspCanStatus can_start(BspCanHandle* hcan) {
    if(hcan == NULL) {
        return bc.INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigInterruptLines(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
        return bc.NOTIFICATION_FAILED;
    }

    if(HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        return bc.NOTIFICATION_FAILED;
    }

    bsp_can_enable_transceiver(hcan);

    if(HAL_FDCAN_Start(hcan) != HAL_OK) {
        return bc.START_FAILED;
    }

    return bc.OK;
}

BspCanStatus can_send(BspCanHandle* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    if(hcan == NULL || data == NULL) {
        return bc.INVALID_PARAM;
    }
    if(len > 8U) {
        return bc.INVALID_DLC;
    }

    FDCAN_TxHeaderTypeDef tx_header = { 0 };
    const uint32_t start_tick = HAL_GetTick();

    tx_header.Identifier = id;
    tx_header.IdType = (id <= 0x7FFU) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = bsp_can_len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    while(HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0U) {
        if((HAL_GetTick() - start_tick) >= BSP_CAN_TX_TIMEOUT_MS) {
            return bc.TX_MAILBOX_TIMEOUT;
        }
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, (uint8_t*)data) != HAL_OK) {
        return bc.TX_FAILED;
    }

    return bc.OK;
}

BspCanStatus can_register_rx_callback(BspCanHandle* hcan, BspCanRxCallback callback, void* user) {
    if(hcan == NULL || callback == NULL) {
        return bc.INVALID_PARAM;
    }

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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        memset(rx_data, 0, sizeof(rx_data));
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }
        bsp_can_dispatch_rx(hfdcan, &rx_header, rx_data);
    }
}

static uint32_t bsp_can_len_to_dlc(uint8_t len) {
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    return dlc_table[len];
}

static BspCanStatus bsp_can_config_global_filter(BspCanHandle* hcan) {
    if(hcan == NULL) {
        return bc.INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hcan,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
        return bc.FILTER_CONFIG_FAILED;
    }

    return bc.OK;
}

static void bsp_can_dispatch_rx(BspCanHandle* hcan, const BspCanRxHeader* header, const uint8_t data[8]) {
    for(uint8_t i = 0U; i < BSP_CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback != NULL) {
            s_rx_slots[i].callback(hcan, header, data, s_rx_slots[i].user);
        }
    }
}

static void bsp_can_enable_transceiver(BspCanHandle* hcan) {
    if(hcan == NULL) {
        return;
    }

    if(hcan->Instance == FDCAN1) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
    else if(hcan->Instance == FDCAN2) {
        HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);
    }
}
