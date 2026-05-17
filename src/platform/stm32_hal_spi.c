#include "stm32_hal_spi.h"

// ! ========================= 变 量 声 明 ========================= ! //

static void(*spi_tx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
static void(*spi_txrx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
static void(*spi_error_callback)(SPI_HandleTypeDef* hspi) = 0;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool spi_write(const uint8_t* data, uint32_t len) {
    if((data == 0) || (len == 0U) || (len > UINT16_MAX)) {
        return false;
    }

    return HAL_SPI_Transmit_DMA(&hspi6, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

void spi_register_tx_complete_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi6) {
        spi_tx_complete_callback = callback;
    }
}

void spi_register_txrx_complete_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_txrx_complete_callback = callback;
    }
}

void spi_register_error_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_error_callback = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi6) {
        if(spi_tx_complete_callback != 0) {
            spi_tx_complete_callback(hspi);
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_txrx_complete_callback != 0) {
            spi_txrx_complete_callback(hspi);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_error_callback != 0) {
            spi_error_callback(hspi);
        }
    }

    if(hspi == &hspi6) {
        if(spi_tx_complete_callback != 0) {
            spi_tx_complete_callback(hspi);
        }
    }
}
