#ifndef _entry_h_
#define _entry_h_


// ! system:
#include "main.h" // IWYU pragma: keep
#include <stdio.h>

// ! platform:
#include "bsp_can.h"
// #include "bsp_debug.h"
#include "bsp_led.h"
// #include "jy901.h"

// ! infra:
#include "delay.h"

// ! domain:


// ! device:
#include "chassis.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"

// ! service:


// ! app:



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

static inline void entry_init(void) {
    printf("System Init...\r\n");

    delay_ms_init(HAL_GetTick);

    if(can.filter_init() != can.OK) Error_Handler();
    if(can.start(&hcan1) != can.OK) Error_Handler();
    if(can.start(&hcan2) != can.OK) Error_Handler();

    delay_ms(2000);

    if(chassis.init() != chassis.OK) Error_Handler();
    delay_ms(100);
    DM_Motor_Set_Angle_Rad(1, 1.0f);
    delay_ms(100);
    DM_Motor_Set_Angle_Rad(2, 1.0f);
    delay_ms(100);
    DM_Motor_Set_Angle_Rad(3, 1.0f);
    delay_ms(100);
    DM_Motor_Set_Angle_Rad(4, 1.0f);
    delay_ms(100);
    // chassis.set_velocity(0.0f, 0.0f, 3.14f);

    printf("System Init OK!\r\n");
}

static inline void entry_loop(void) {
    static ms_t led_task = 0;
    static ms_t chassis_task = 0;
    if(delay_nb_ms(&led_task, 500)) led.toggle();
    // if(delay_nb_ms(&chassis_task, 2)) chassis.task_500hz();
}

#endif
