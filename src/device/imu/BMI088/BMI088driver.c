#include "imu/BMI088/inc/BMI088driver.h"
#include "imu/BMI088/inc/BMI088reg.h"
#include "imu/BMI088/inc/BMI088Middleware.h"
#include <string.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)

#define BMI088_DMA_GYRO_FRAME_LEN    7U
#define BMI088_DMA_ACCEL_FRAME_LEN   8U
#define BMI088_SPI_DUMMY_BYTE        0x55U

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t* return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);
static void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data);
static void BMI088_accel_read_burst(uint8_t reg, uint8_t* buf, uint8_t len);
static void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
static void BMI088_gyro_read_burst(uint8_t reg, uint8_t* buf, uint8_t len);
static void BMI088_release_all_cs(void);
static bool BMI088_start_gyro_dma(void);
static bool BMI088_start_accel_dma(void);
static void BMI088_parse_gyro_dma_buffer(void);
static void BMI088_parse_accel_dma_buffer(void);
static void BMI088_copy_vector3(float dst[3], const float src[3]);
static void BMI088_dma_prepare_txrx(uint16_t len);
static void BMI088_dma_maintain_before_start(uint16_t len);
static void BMI088_dma_maintain_after_finish(uint16_t len);

#elif defined(BMI088_USE_IIC)

#endif

static volatile bmi088_dma_state_t s_bmi088_dma_state = BMI088_DMA_IDLE;
static volatile uint8_t s_gyro_pending = 0U;
static volatile uint8_t s_accel_pending = 0U;
static volatile uint8_t s_gyro_ready = 0U;
static volatile uint8_t s_accel_ready = 0U;

__attribute__((aligned(32))) static uint8_t s_bmi088_tx[BMI088_DMA_ACCEL_FRAME_LEN];
__attribute__((aligned(32))) static uint8_t s_bmi088_rx[BMI088_DMA_ACCEL_FRAME_LEN];

static float s_gyro_dma[3];
static float s_accel_dma[3];

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

uint8_t BMI088_init(void) {
    uint8_t error = BMI088_NO_ERROR;

    BMI088_GPIO_init();
    BMI088_com_init();
    BMI088_async_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

uint8_t bmi088_accel_init(void) {
    uint8_t res = 0U;

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for(uint8_t write_reg_num = 0U; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        BMI088_accel_write_single_reg(
            write_BMI088_accel_reg_data_error[write_reg_num][0],
            write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(
            write_BMI088_accel_reg_data_error[write_reg_num][0],
            &res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if(res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void) {
    uint8_t res = 0U;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for(uint8_t write_reg_num = 0U; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        BMI088_gyro_write_single_reg(
            write_BMI088_gyro_reg_data_error[write_reg_num][0],
            write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(
            write_BMI088_gyro_reg_data_error[write_reg_num][0],
            &res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if(res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_read(float gyro[3], float accel[3], float* temperate) {
    BMI088_read_accel(accel);
    BMI088_read_gyro(gyro);
    BMI088_read_temp(temperate);
}

void BMI088_read_gyro(float gyro[3]) {
    uint8_t buf[6] = { 0U };
    int16_t raw_value = 0;

    BMI088_gyro_read_burst(BMI088_GYRO_X_L, buf, sizeof(buf));

    raw_value = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    gyro[0] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    gyro[1] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    gyro[2] = raw_value * BMI088_GYRO_SEN;
}

void BMI088_read_accel(float accel[3]) {
    uint8_t buf[6] = { 0U };
    int16_t raw_value = 0;

    BMI088_accel_read_burst(BMI088_ACCEL_XOUT_L, buf, sizeof(buf));

    raw_value = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    accel[0] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    accel[1] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    accel[2] = raw_value * BMI088_ACCEL_SEN;
}

void BMI088_read_temp(float* temperature) {
    uint8_t buf[2] = { 0U };
    int16_t raw_value = 0;

    BMI088_accel_read_burst(BMI088_TEMP_M, buf, sizeof(buf));

    raw_value = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if(raw_value > 1023) {
        raw_value -= 2048;
    }

    *temperature = raw_value * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088_async_init(void) {
    s_bmi088_dma_state = BMI088_DMA_IDLE;
    s_gyro_pending = 0U;
    s_accel_pending = 0U;
    s_gyro_ready = 0U;
    s_accel_ready = 0U;
    memset(s_bmi088_tx, 0, sizeof(s_bmi088_tx));
    memset(s_bmi088_rx, 0, sizeof(s_bmi088_rx));
    memset(s_gyro_dma, 0, sizeof(s_gyro_dma));
    memset(s_accel_dma, 0, sizeof(s_accel_dma));
    BMI088_release_all_cs();
}

void BMI088_notify_gyro_data_ready(void) {
    s_gyro_pending = 1U;
}

void BMI088_notify_accel_data_ready(void) {
    s_accel_pending = 1U;
}

void BMI088_EXTI_Callback(uint16_t gpio_pin) {
    if(gpio_pin == GYRO_INT_Pin) {
        BMI088_notify_gyro_data_ready();
    }
    else if(gpio_pin == ACC_INT_Pin) {
        BMI088_notify_accel_data_ready();
    }
}

bool BMI088_async_poll(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    if(s_gyro_pending != 0U) {
        s_gyro_pending = 0U;
        if(BMI088_start_gyro_dma()) {
            return true;
        }

        s_gyro_pending = 1U;
        return false;
    }

    if(s_accel_pending != 0U) {
        s_accel_pending = 0U;
        if(BMI088_start_accel_dma()) {
            return true;
        }

        s_accel_pending = 1U;
        return false;
    }

    return false;
}

bool BMI088_async_get_gyro(float gyro[3]) {
    if((gyro == NULL) || (s_gyro_ready == 0U)) {
        return false;
    }

    BMI088_copy_vector3(gyro, s_gyro_dma);
    s_gyro_ready = 0U;
    return true;
}

bool BMI088_async_get_accel(float accel[3]) {
    if((accel == NULL) || (s_accel_ready == 0U)) {
        return false;
    }

    BMI088_copy_vector3(accel, s_accel_dma);
    s_accel_ready = 0U;
    return true;
}

bool BMI088_async_is_busy(void) {
    return s_bmi088_dma_state != BMI088_DMA_IDLE;
}

bmi088_dma_state_t BMI088_async_state(void) {
    return s_bmi088_dma_state;
}

void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi != BMI088_get_spi_handle()) {
        return;
    }

    if(s_bmi088_dma_state == BMI088_DMA_GYRO) {
        BMI088_GYRO_NS_H();
        BMI088_dma_maintain_after_finish(BMI088_DMA_GYRO_FRAME_LEN);
        BMI088_parse_gyro_dma_buffer();
        s_gyro_ready = 1U;
    }
    else if(s_bmi088_dma_state == BMI088_DMA_ACCEL) {
        BMI088_ACCEL_NS_H();
        BMI088_dma_maintain_after_finish(BMI088_DMA_ACCEL_FRAME_LEN);
        BMI088_parse_accel_dma_buffer();
        s_accel_ready = 1U;
    }

    s_bmi088_dma_state = BMI088_DMA_IDLE;
}

void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if(hspi != BMI088_get_spi_handle()) {
        return;
    }

    BMI088_release_all_cs();
    s_bmi088_dma_state = BMI088_DMA_IDLE;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t* return_data) {
    BMI088_read_write_byte(reg | 0x80U);
    *return_data = BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
    BMI088_read_write_byte(reg | 0x80U);

    while(len != 0U) {
        *buf = BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
        buf++;
        len--;
    }
}

static void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_ACCEL_NS_L();
    BMI088_write_single_reg(reg, data);
    BMI088_ACCEL_NS_H();
}

static void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg | 0x80U);
    BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
    *data = BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
    BMI088_ACCEL_NS_H();
}

static void BMI088_accel_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg | 0x80U);
    BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);

    while(len != 0U) {
        *buf = BMI088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
        buf++;
        len--;
    }

    BMI088_ACCEL_NS_H();
}

static void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_L();
    BMI088_write_single_reg(reg, data);
    BMI088_GYRO_NS_H();
}

static void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
    BMI088_GYRO_NS_L();
    BMI088_read_single_reg(reg, data);
    BMI088_GYRO_NS_H();
}

static void BMI088_gyro_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    BMI088_GYRO_NS_L();
    BMI088_read_muli_reg(reg, buf, len);
    BMI088_GYRO_NS_H();
}

static void BMI088_release_all_cs(void) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
}

static bool BMI088_start_gyro_dma(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    s_bmi088_dma_state = BMI088_DMA_GYRO;
    BMI088_dma_prepare_txrx(BMI088_DMA_GYRO_FRAME_LEN);
    s_bmi088_tx[0] = BMI088_GYRO_X_L | 0x80U;
    BMI088_dma_maintain_before_start(BMI088_DMA_GYRO_FRAME_LEN);
    BMI088_GYRO_NS_L();

    if(BMI088_SPI_TransmitReceive_DMA(
        s_bmi088_tx,
        s_bmi088_rx,
        BMI088_DMA_GYRO_FRAME_LEN) != HAL_OK) {
        BMI088_GYRO_NS_H();
        s_bmi088_dma_state = BMI088_DMA_IDLE;
        return false;
    }

    return true;
}

static bool BMI088_start_accel_dma(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    s_bmi088_dma_state = BMI088_DMA_ACCEL;
    BMI088_dma_prepare_txrx(BMI088_DMA_ACCEL_FRAME_LEN);
    s_bmi088_tx[0] = BMI088_ACCEL_XOUT_L | 0x80U;
    BMI088_dma_maintain_before_start(BMI088_DMA_ACCEL_FRAME_LEN);
    BMI088_ACCEL_NS_L();

    if(BMI088_SPI_TransmitReceive_DMA(
        s_bmi088_tx,
        s_bmi088_rx,
        BMI088_DMA_ACCEL_FRAME_LEN) != HAL_OK) {
        BMI088_ACCEL_NS_H();
        s_bmi088_dma_state = BMI088_DMA_IDLE;
        return false;
    }

    return true;
}

static void BMI088_parse_gyro_dma_buffer(void) {
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[2] << 8) | s_bmi088_rx[1]);
    s_gyro_dma[0] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[4] << 8) | s_bmi088_rx[3]);
    s_gyro_dma[1] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[6] << 8) | s_bmi088_rx[5]);
    s_gyro_dma[2] = raw_value * BMI088_GYRO_SEN;
}

static void BMI088_parse_accel_dma_buffer(void) {
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[3] << 8) | s_bmi088_rx[2]);
    s_accel_dma[0] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[5] << 8) | s_bmi088_rx[4]);
    s_accel_dma[1] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[7] << 8) | s_bmi088_rx[6]);
    s_accel_dma[2] = raw_value * BMI088_ACCEL_SEN;
}

static void BMI088_copy_vector3(float dst[3], const float src[3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

static void BMI088_dma_prepare_txrx(uint16_t len) {
    memset(s_bmi088_tx, BMI088_SPI_DUMMY_BYTE, len);
    memset(s_bmi088_rx, 0, len);
}

static void BMI088_dma_maintain_before_start(uint16_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t tx_start = ((uintptr_t)s_bmi088_tx) & ~(uintptr_t)31U;
    uintptr_t rx_start = ((uintptr_t)s_bmi088_rx) & ~(uintptr_t)31U;
    int32_t tx_size = (int32_t)((((uintptr_t)s_bmi088_tx + len + 31U) & ~(uintptr_t)31U) - tx_start);
    int32_t rx_size = (int32_t)((((uintptr_t)s_bmi088_rx + len + 31U) & ~(uintptr_t)31U) - rx_start);

    SCB_CleanDCache_by_Addr((uint32_t*)tx_start, tx_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_start, rx_size);
#else
    (void)len;
#endif
}

static void BMI088_dma_maintain_after_finish(uint16_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t rx_start = ((uintptr_t)s_bmi088_rx) & ~(uintptr_t)31U;
    int32_t rx_size = (int32_t)((((uintptr_t)s_bmi088_rx + len + 31U) & ~(uintptr_t)31U) - rx_start);

    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_start, rx_size);
#else
    (void)len;
#endif
}

#elif defined(BMI088_USE_IIC)

#endif
