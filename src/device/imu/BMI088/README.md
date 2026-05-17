# BMI088

提供 BMI088 驱动的两种使用方式：

- 阻塞式接口
- `IT + DMA` 异步接口

## CubeMX 配置

### SPI2

- Master
- 8-bit
- Mode 3 或 Mode 0
- NSS 软件控制
- TX DMA: Normal
- RX DMA: Normal
- Memory Increment: Enable
- Peripheral Increment: Disable
- Data Width: Byte
- Priority: High / Very High

### EXTI

BMI088 中断引脚配置为高有效，MCU 侧使用 Rising Edge

当前驱动内部已经按高有效配置寄存器：

- accel: `BMI088_INT1_IO_CTRL = ENABLE | PP | HIGH`
- gyro: `BMI088_GYRO_INT3_INT4_IO_CONF = PP | HIGH`

## 用法一：阻塞式接口

适合初始化验证、低频轮询和快速联调

```c
#include "imu/BMI088/inc/BMI088driver.h"

void app_init(void) {
    uint8_t err = BMI088_init();
    if(err != BMI088_NO_ERROR) {
        // error handle
    }
}

void app_loop(void) {
    float gyro[3];
    float accel[3];
    float temp = 0.0f;

    BMI088_read(gyro, accel, &temp);

    // 也可以按需单独读取
    // BMI088_read_gyro(gyro);
    // BMI088_read_accel(accel);
    // BMI088_read_temp(&temp);
}
```

### 阻塞式接口说明

- `BMI088_init()`：完成 GPIO/SPI 基础准备，并阻塞式初始化 accel/gyro 寄存器
- `BMI088_read()`：一次读出 gyro、accel、temp
- `BMI088_read_gyro()` / `BMI088_read_accel()` / `BMI088_read_temp()`：按需分开读取

## 用法二：IT + DMA

推荐用于运行阶段的高频采样

### 1. 初始化

```c
#include "imu/BMI088/inc/BMI088driver.h"

void app_init(void) {
    uint8_t err = BMI088_init();
    if(err != BMI088_NO_ERROR) {
        // error handle
    }

    BMI088_async_init();
}
```

### 2. EXTI 回调中只置标志

不要在 EXTI 回调里直接访问 SPI

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BMI088_EXTI_Callback(GPIO_Pin);
}
```

如果你不想直接透传，也可以手动调用：

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GYRO_INT_Pin) {
        BMI088_notify_gyro_data_ready();
    } else if(GPIO_Pin == ACC_INT_Pin) {
        BMI088_notify_accel_data_ready();
    }
}
```

### 3. 在主循环或高频任务中调度 DMA

```c
void app_loop(void) {
    float gyro[3];
    float accel[3];

    BMI088_async_poll();

    if(BMI088_async_get_gyro(gyro)) {
        // consume latest gyro sample
    }

    if(BMI088_async_get_accel(accel)) {
        // consume latest accel sample
    }
}
```

### 4. 在 HAL SPI DMA 回调中转发

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BMI088_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    BMI088_SPI_ErrorCallback(hspi);
}
```

错误回调里驱动会自动：

- 拉高 accel/gyro 的 CS
- 将 DMA 状态机恢复为 `BMI088_DMA_IDLE`

## DMA 时序说明

### gyro

读取 6 字节数据时，DMA 总长度为 `7`：

- `TX[0] = BMI088_GYRO_X_L | 0x80`
- `RX[0]` 无效
- `RX[1..6]` 为 XYZ 三轴数据

### accel

读取 6 字节数据时，DMA 总长度为 `8`：

- `TX[0] = BMI088_ACCEL_XOUT_L | 0x80`
- `RX[0]` 无效
- `RX[1]` dummy
- `RX[2..7]` 为 XYZ 三轴数据

## H7 DCache 注意事项

如果 STM32H7 开启了 DCache，而 DMA buffer 放在 cacheable RAM 中，可能会出现：

- DMA 已完成，但程序读到的还是旧数据
- 数据偶发正确、偶发错误

当前驱动已经做了两件事：

- DMA buffer 32 字节对齐
- DMA 前 clean TX cache，DMA 前后 invalidate RX cache

如果后续你已经使用 MPU 划分了 non-cacheable 区域，也可以把 DMA buffer 放到那块区域中

## 对接提醒

- 当前默认 SPI 实例在 [BMI088Middleware.c](/D:/Desktop/Learn/AgroTech/China-AI-Robot-Competition/Wheel-Steer-Chassis/src/device/imu/BMI088/BMI088Middleware.c) 中由 `BMI088_USING_SPI_UNIT` 指向 `hspi2`
- 如果你的工程已经实现了 `HAL_GPIO_EXTI_Callback()`、`HAL_SPI_TxRxCpltCallback()` 或 `HAL_SPI_ErrorCallback()`，只需要把 BMI088 的转发逻辑合并进去，不要重复定义多个同名 HAL 回调
- 温度读取当前仍保持阻塞式，这通常已经足够，因为温度不需要像陀螺仪和加速度计那样高频刷新
