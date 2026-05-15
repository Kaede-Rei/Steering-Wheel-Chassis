#ifndef _BSP_DEBUG_H_
#define _BSP_DEBUG_H_

#include "main.h"
#include <stdio.h> // printf 所需的标准库

// VOFA+ FireWater 协议数据发送函数
void VOFA_Send_FireWater(float target_rpm, float real_rpm);

#endif /* _BSP_DEBUG_H_ */