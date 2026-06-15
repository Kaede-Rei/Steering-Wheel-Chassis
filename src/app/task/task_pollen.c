#include "task_pollen.h"

#include "arm.h"
#include "asrpro.h"
#include "delay.h"
#include "log.h"
#include "pollen_route.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define TASK_ARM_SPEED_RAD_S 12.56f
#define TASK_POLLEN_BROADCAST_WAIT_MS 4000u
#define TASK_POLLEN_STEP_FEEDBACK_DELAY_MS 100u
#define TASK_POLLEN_REACH_TOLERANCE_RAD 0.15f
#define TASK_POLLEN_STEP_INTERVAL_MS 300u
#define TASK_POLLEN_STEP_TIMEOUT_MS 10000u
#define TASK_POLLEN_PREPOSE_TIMEOUT_MS 10000u

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void pollen_sequence_reset(TaskPollenSequenceContext* sequence);
static bool pollen_load_sequence(TaskPollenSequenceContext* sequence, uint8_t nav_index);
static bool pollen_step_reached(const FiveDofArmJointArray* target);
static void pollen_log_step_timeout(uint8_t step, const FiveDofArmJointArray* target);
static AsrProCmd pollen_make_x_broadcast_cmd(XFlowerType flowers);
static AsrProCmd pollen_make_x_side_broadcast_cmd(XFlowerType flowers);
static AsrProCmd pollen_make_y_broadcast_cmd(YFlowerType flowers);
static bool pollen_make_broadcast_cmd(AreaType area, const NavPoint* point, AsrProCmd* out_cmd);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 重置授粉上下文
 * @param pollen 授粉上下文
 */
void task_pollen_reset(TaskPollenContext* pollen) {
    if(pollen == NULL)
        return;

    pollen_sequence_reset(&pollen->sequence);
}

/**
 * @brief 开始授粉动作
 * @param pollen 授粉上下文
 * @param nav_index 当前导航点索引
 * @param area 当前区域
 * @param point 当前导航点
 */
void task_pollen_start(TaskPollenContext* pollen, uint8_t nav_index, AreaType area, const NavPoint* point) {
    if(pollen == NULL)
        return;

    TaskPollenSequenceContext* sequence;
    AsrProCmd cmd;

    sequence = &pollen->sequence;

    if(pollen_load_sequence(sequence, nav_index) == false)
        log_error("Pollen sequence missing at nav point %u", nav_index);

    if(pollen_make_broadcast_cmd(area, point, &cmd)) {
        sequence->broadcast_pending = true;
        sequence->broadcast_cmd = (uint8_t)cmd;
    }

    if(point != NULL && point->pre_detect_joints.exist) {
        sequence->prepose_waiting = true;
        sequence->prepose_target = point->pre_detect_joints.joints;
        sequence->prepose_start_ms = delay_now_ms();
        sequence->last_feedback_ms = 0u;
    }
    else {
        sequence->prepose_waiting = false;
    }
}

/**
 * @brief 执行授粉机械臂非阻塞动作序列
 * @param pollen 授粉上下文
 * @return TaskPollenResult 授粉处理结果
 */
TaskPollenResult task_pollen_process(TaskPollenContext* pollen) {
    TaskPollenSequenceContext* sequence;
    const FiveDofArmJointArray* target;
    ArmStatus arm_status;
    uint32_t now = delay_now_ms();


    if(pollen == NULL)
        return TASK_POLLEN_RESULT_ERROR;

    sequence = &pollen->sequence;

    if(sequence->prepose_waiting) {
        if(pollen_step_reached(&sequence->prepose_target)) {
            sequence->prepose_waiting = false;
            sequence->last_feedback_ms = 0u;
        }
        else {
            if((now - sequence->prepose_start_ms) >= TASK_POLLEN_PREPOSE_TIMEOUT_MS) {
                log_error("Pollen prepose wait timeout");
                return TASK_POLLEN_RESULT_ERROR;
            }

            return TASK_POLLEN_RESULT_RUNNING;
        }
    }

    if(sequence->broadcast_pending) {
        (void)asrpro_broadcast((AsrProCmd)sequence->broadcast_cmd);
        sequence->broadcast_pending = false;
        sequence->broadcast_waiting = true;
        sequence->broadcast_start_ms = now;
        return TASK_POLLEN_RESULT_RUNNING;
    }

    if(sequence->broadcast_waiting) {
        if((now - sequence->broadcast_start_ms) < TASK_POLLEN_BROADCAST_WAIT_MS)
            return TASK_POLLEN_RESULT_RUNNING;

        sequence->broadcast_waiting = false;
    }

    if(sequence->step_count == 0u)
        return TASK_POLLEN_RESULT_FINISHED;
    if(sequence->current_step >= sequence->step_count) {
        pollen_sequence_reset(sequence);
        return TASK_POLLEN_RESULT_FINISHED;
    }

    if(sequence->step_interval_waiting) {
        if((now - sequence->step_interval_start_ms) < TASK_POLLEN_STEP_INTERVAL_MS)
            return TASK_POLLEN_RESULT_RUNNING;

        sequence->step_interval_waiting = false;
    }

    target = &sequence->steps[sequence->current_step];
    if(sequence->step_started == false) {
        arm_status = arm.move_joints(target, TASK_ARM_SPEED_RAD_S);
        if(arm_status != ARM_OK) {
            log_error("Pollen arm step %u start failed: %s", sequence->current_step, arm.status_str(arm_status));
            return TASK_POLLEN_RESULT_ERROR;
        }

        sequence->step_started = true;
        sequence->step_start_ms = now;
        sequence->last_feedback_ms = 0u;
        return TASK_POLLEN_RESULT_RUNNING;
    }

    if((now - sequence->step_start_ms) < TASK_POLLEN_STEP_FEEDBACK_DELAY_MS)
        return TASK_POLLEN_RESULT_RUNNING;

    if(pollen_step_reached(target)) {
        sequence->current_step++;
        sequence->step_started = false;
        sequence->last_feedback_ms = 0u;

        if(sequence->current_step >= sequence->step_count) {
            pollen_sequence_reset(sequence);
            return TASK_POLLEN_RESULT_FINISHED;
        }

        sequence->step_interval_waiting = true;
        sequence->step_interval_start_ms = now;

        return TASK_POLLEN_RESULT_RUNNING;
    }

    if((now - sequence->step_start_ms) >= TASK_POLLEN_STEP_TIMEOUT_MS) {
        pollen_log_step_timeout(sequence->current_step, target);
        return TASK_POLLEN_RESULT_ERROR;
    }

    return TASK_POLLEN_RESULT_RUNNING;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 重置授粉机械臂动作序列上下文
 * @param sequence 授粉机械臂动作序列上下文
 */
static void pollen_sequence_reset(TaskPollenSequenceContext* sequence) {
    if(sequence == NULL)
        return;

    memset(sequence, 0, sizeof(*sequence));
}

/**
 * @brief 从静态动作表加载当前导航点的授粉动作序列
 * @param sequence 授粉机械臂动作序列上下文
 * @param nav_index 当前导航点索引
 * @return bool `true` 表示加载成功
 */
static bool pollen_load_sequence(TaskPollenSequenceContext* sequence, uint8_t nav_index) {
    PollenActionSequence action_sequence;
    uint8_t i;

    if(sequence == NULL)
        return false;

    pollen_sequence_reset(sequence);
    if(pollen_route_get(nav_index, &action_sequence) == false)
        return false;

    if(action_sequence.step_count > TASK_POLLEN_MAX_STEPS)
        return false;

    sequence->step_count = action_sequence.step_count;
    for(i = 0u; i < action_sequence.step_count; i++) {
        sequence->steps[i] = action_sequence.steps[i];
    }

    return true;
}

/**
 * @brief 判断授粉机械臂当前步骤是否到位
 * @param target 当前步目标关节角
 * @return bool `true` 表示已到位
 */
static bool pollen_step_reached(const FiveDofArmJointArray* target) {
    const FiveDofArmJointArray* current;
    uint8_t i;

    if(target == NULL)
        return false;

    current = arm.get_current_joints();
    if(current == NULL || current->dof != target->dof)
        return false;

    for(i = 0u; i < target->dof; i++) {
        if(fabsf(current->q[i] - target->q[i]) > TASK_POLLEN_REACH_TOLERANCE_RAD)
            return false;
    }

    return true;
}

static void pollen_log_step_timeout(uint8_t step, const FiveDofArmJointArray* target) {
    const FiveDofArmJointArray* current;
    float max_err = 0.0f;
    uint8_t max_index = 0u;
    uint8_t i;

    log_error("Pollen arm step %u timeout", step);

    if(target == NULL) {
        log_error("Pollen target is NULL");
        return;
    }

    current = arm.get_current_joints();
    if(current == NULL) {
        log_error("Pollen current joints invalid");
        return;
    }

    if(current->dof != target->dof) {
        log_error("Pollen dof mismatch current=%u target=%u", current->dof, target->dof);
        return;
    }

    for(i = 0u; i < target->dof; i++) {
        float err = fabsf(current->q[i] - target->q[i]);
        if(err > max_err) {
            max_err = err;
            max_index = i;
        }
        log_error("Pollen joint %u current=%.3f target=%.3f err=%.3f",
                  i,
                  current->q[i],
                  target->q[i],
                  err);
    }

    log_error("Pollen max err joint=%u err=%.3f tolerance=%.3f",
              max_index,
              max_err,
              TASK_POLLEN_REACH_TOLERANCE_RAD);
}

/**
 * @brief 将横向花型信息编码为语音播报指令
 * @param flowers 横向花型信息
 * @return AsrProCmd 语音播报指令
 */
static AsrProCmd pollen_make_x_broadcast_cmd(XFlowerType flowers) {
    return (AsrProCmd)(X_000 + ((flowers.left ? 1 : 0) << 2) + ((flowers.mid ? 1 : 0) << 1) + (flowers.right ? 1 : 0));
}

/**
 * @brief 将 C 区左右两侧花型信息编码为语音播报指令
 * @param flowers 横向花型信息
 * @return AsrProCmd 语音播报指令
 */
static AsrProCmd pollen_make_x_side_broadcast_cmd(XFlowerType flowers) {
    return (AsrProCmd)(X_00 + ((flowers.left ? 1 : 0) << 1) + (flowers.right ? 1 : 0));
}

/**
 * @brief 将纵向花型信息编码为语音播报指令
 * @param flowers 纵向花型信息
 * @return AsrProCmd 语音播报指令
 */
static AsrProCmd pollen_make_y_broadcast_cmd(YFlowerType flowers) {
    return (AsrProCmd)(Y_000 + ((flowers.up ? 1 : 0) << 2) + ((flowers.mid ? 1 : 0) << 1) + (flowers.down ? 1 : 0));
}

/**
 * @brief 按当前点位信息发送对应语音播报
 * @param area 当前区域
 * @param point 当前导航点
 */
static bool pollen_make_broadcast_cmd(AreaType area, const NavPoint* point, AsrProCmd* out_cmd) {
    if(point == NULL || out_cmd == NULL)
        return false;

    switch(area) {
        case AREA_A:
            *out_cmd = pollen_make_y_broadcast_cmd(point->y_flowers);
            return true;

        case AREA_B:
            *out_cmd = pollen_make_x_broadcast_cmd(point->x_flowers);
            return true;

        case AREA_C:
            *out_cmd = pollen_make_x_side_broadcast_cmd(point->x_flowers);
            return true;

        default:
            return false;
    }
}
