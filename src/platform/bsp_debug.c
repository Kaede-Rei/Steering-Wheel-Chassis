#include "bsp_debug.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
// ========================================================
// 1. GCC 编译器的 printf 底层重定向
// ========================================================
int _write(int file, char *ptr, int len)
{
    // 将 printf 产生的数据通过 USART1 轮询发送出去
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

// ========================================================
// 2. VOFA+ FireWater 协议发送函数
// ========================================================
void VOFA_Send_FireWater(float target_rpm, float real_rpm)
{
    // FireWater 协议的核心规则：数据用逗号隔开，结尾必须是换行符 \n
    printf("%.2f,%.2f\n", target_rpm, real_rpm);
}