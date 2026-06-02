#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>


// ! app ! //
#include "competition.h"
#include "remote.h"


// ! service ! //
#include "assemble/assemble.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "arm.h"
#include "line_sensor.h"
#include "visual_comms.h"

// ! device ! //
#include "imu/imu.h"
#include "rgb_led/rgb_led.h"
#include "gw_gray.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro_bias = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro_corrected = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };
static uint8_t remote_tick = 0;
static uint8_t led_state = 0u;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

static inline bool entry_competition_is_scan_state(CompetitionState state) {
    return state == COMPETITION_STATE_A_SCAN
        || state == COMPETITION_STATE_B_SCAN
        || state == COMPETITION_STATE_C_SCAN;
}

static inline void entry_competition_consume_visual_inputs(void) {
    MissionCommand command = MISSION_COMMAND_NONE;
    MissionRecognitionResult recognition = { 0 };
    MissionUavHandoffAck handoff_ack = { 0 };
    const ms_t now_ms = delay_now_ms();
    const CompetitionState competition_state = competition.get_state_id();
    const MissionRuntime* mission_state = mission.get_state();

    if(visual_comms.consume_command(&command)) {
        mission.touch_dependency(MISSION_DEPENDENCY_ID_VISION, now_ms);
        (void)competition.handle_command(command);
    }

    if(visual_comms.consume_recognition(&recognition)) {
        mission.touch_dependency(MISSION_DEPENDENCY_ID_VISION, now_ms);
        (void)mission.note_recognition(&recognition, now_ms);

        if(entry_competition_is_scan_state(competition_state)
            && mission_state != NULL
            && recognition.zone == mission_state->current_zone) {
            if(visual_comms.recognition_is_stale(&recognition) || recognition.sex == FLOWER_SEX_UNKNOWN) {
                (void)competition.post_event(COMPETITION_EVENT_STALE_OR_UNKNOWN_RESULT, 0u);
            }
            else if(recognition.sex == FLOWER_SEX_FEMALE) {
                (void)competition.post_event(COMPETITION_EVENT_FEMALE_RESULT, 0u);
            }
            else {
                (void)competition.post_event(COMPETITION_EVENT_MALE_RESULT, 0u);
            }
        }
    }

    if(visual_comms.consume_uav_handoff_ack(&handoff_ack)) {
        mission.touch_dependency(MISSION_DEPENDENCY_ID_UAV_HANDOFF, now_ms);
        (void)mission.note_uav_handoff_ack(&handoff_ack, now_ms);

        if(competition_state == COMPETITION_STATE_GO_D_HANDOFF) {
            if(handoff_ack.status == MISSION_UAV_HANDOFF_SUCCESS) {
                (void)competition.post_event(COMPETITION_EVENT_ACTION_COMPLETE, 0u);
            }
            else if(handoff_ack.status == MISSION_UAV_HANDOFF_FAIL_TERMINAL) {
                (void)competition.post_event(COMPETITION_EVENT_TERMINAL_FAULT, MISSION_FAULT_UAV_HANDOFF_FAIL_TERMINAL);
            }
        }
    }
}

/**
 * @brief 程序初始化入口函数
 *
 * 该函数由 main 初始化完成后调用；
 * 负责装配各服务并清零底盘速度命令
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;
    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    delay_ms(100);

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb init step done");
    delay_ms(100);

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu init step done");
    delay_ms(100);

    if(assemble_chassis() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT chassis init step done");
    delay_ms(100);

    if(assemble_light() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT light init step done");
    delay_ms(100);

    if(assemble_remote() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT remote init step done");
    delay_ms(100);

    if(assemble_comms() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT visual comms init step done");
    delay_ms(100);

    remote_init();
    chassis_yaw_hold_set_target(0.0f);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

    if(assemble_arm() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT arm init step done");
    delay_ms(100);

    log_info("System initialized successfully");
    delay_ms(500);

    ArmStatus error_code = arm.move_position(0.1f, 0.00f, 0.3f, 3.14f);
    if(error_code != ARM_OK) {
        log_error("Failed to move arm to initial position: %s", arm.status_str(error_code));
    }

    CompetitionStatus competition_status = competition.init();
    if(competition_status != competition.OK) {
        log_error("BOOT competition init failed: %s", competition.status_str(competition_status));
        return;
    }
    log_info("BOOT competition init step done");
}

/**
 * @brief 程序主循环入口函数
 *
 * 该函数在 while(1) 中持续调用；
 * 根据定时器事件执行底盘、遥控、IMU 和日志任务
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    visual_comms.process();

    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        if(imu.update() == IMU_STATUS_OK) {
            accel = imu.get_acc();
            gyro = imu.get_gyro();
            gyro_bias = imu_get_gyro_bias();
            gyro_corrected = imu_get_gyro_corrected();
            angle = imu.get_angle();
        }

        chassis.process();

        if(remote_tick++ % 5 == 0) {
            remote_process();
            remote_tick = 0;

            const ms_t now_ms = delay_now_ms();
            RemoteCommand remote_command = { 0 };
            if(remote_get_command(&remote_command)) {
                mission.touch_dependency(MISSION_DEPENDENCY_ID_REMOTE_LINK, now_ms);
            }

            gw_gray_update();
            line_sensor_update();

            const LineSensorState* line_state = line_sensor_get_state();
            if(line_state != NULL && line_state->source_update_count > 0u) {
                mission.touch_dependency(MISSION_DEPENDENCY_ID_LINE_SENSOR, now_ms);
            }
        }
    }

    entry_competition_consume_visual_inputs();
    competition.process();

    // ! 周期性任务 ! //
    if(delay_nb_ms(&heartbeat_task, 1000)) {
        RemoteCommand remote_command;
        const bool chassis_ready = chassis.is_ready();
        const bool remote_online = remote_get_command(&remote_command);

        uint8_t target_state = chassis_ready ? 1u : 0u;
        if(target_state == 1u && remote_online)
            target_state = 2u;

        if(led_state != target_state) {
            if(target_state == 2u)
                rgb_led.fill(0U, 0U, 255U);
            else if(target_state == 1u)
                rgb_led.fill(0U, 255U, 0U);
            else
                rgb_led.fill(255U, 0U, 0U);
            if(rgb_led.show() == RGB_LED_STATUS_OK)
                led_state = target_state;
            log_info("Chassis %s, Remote %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline");
        }
    }

    if(delay_nb_ms(&log_task, 1000)) {
        FiveDofArmPose arm_pose = arm.get_current_pose() != NULL ? *arm.get_current_pose() : (FiveDofArmPose){ 0 };
        log_info("Arm Pose: (%.2f, %.2f, %.2f)", arm_pose.position.x, arm_pose.position.y, arm_pose.position.z);
    }
}

#endif
