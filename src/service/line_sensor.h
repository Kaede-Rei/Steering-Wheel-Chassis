#ifndef _line_sensor_h_
#define _line_sensor_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 线传感器解释结果状态
 *
 * bit 位方向约定：
 * - bit0 = 最左侧探头
 * - bit7 = 最右侧探头
 * - `lateral_error > 0` 表示黑线整体偏机器人右侧
 * - `lateral_error < 0` 表示黑线整体偏机器人左侧
 * - `heading_error > 0` 表示前排黑线中心比后排更靠右
 * - `heading_error < 0` 表示前排黑线中心比后排更靠左
 *
 * 代表性 mask 行为：
 * - `front=0x03, back=0x03` -> 左偏，有效，`lateral_error < 0`
 * - `front=0x18, back=0x18` -> 居中，有效，`lateral_error ~= 0`
 * - `front=0xC0, back=0xC0` -> 右偏，有效，`lateral_error > 0`
 * - `front=0x00, back=0x00` -> 白场，无线，`line_detected=false`，非 invalid
 * - `front=0x81, back=0x81` -> 同排分裂双峰，判为 contradictory/invalid
 * - `front=0x03, back=0xC0` -> 前后中心相互矛盾，判为 contradictory/invalid
 * - `front=0xFF, back=0xFF` -> 全宽黑线，判为 checkpoint + cross-line
 */
typedef enum {
    LINE_SENSOR_STATUS_NOT_READY = 0,
    LINE_SENSOR_STATUS_NO_LINE,
    LINE_SENSOR_STATUS_TRACK_VALID,
    LINE_SENSOR_STATUS_INVALID_PATTERN,
} LineSensorStatus;

typedef struct {
    bool line_detected;
    bool state_valid;
    bool invalid_pattern;
    bool checkpoint_detected;
    bool cross_line_detected;
    uint8_t front_mask;
    uint8_t back_mask;
    float lateral_error;
    float heading_error;
    uint32_t timestamp_us;
    uint32_t source_update_count;
    LineSensorStatus status;
} LineSensorState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 基于当前灰度传感器快照更新一次线传感器解释状态
 */
void line_sensor_update(void);

/**
 * @brief 获取线传感器状态只读视图
 * @return const LineSensorState* 当前状态快照
 */
const LineSensorState* line_sensor_get_state(void);

/**
 * @brief 判断当前解释结果是否为可跟踪的有效线
 * @return true 当前状态可直接用于导航纠偏
 * @return false 当前状态不可直接用于导航纠偏
 */
bool line_sensor_has_valid_line(void);

/**
 * @brief 判断当前是否命中过点/检查点样式
 * @return true 前后两排都检测到全宽黑带
 * @return false 当前不是检查点样式
 */
bool line_sensor_is_checkpoint(void);

/**
 * @brief 判断当前是否检测到横向宽黑带/十字线样式
 * @return true 至少有一排检测到宽黑带
 * @return false 当前不是横向宽黑带样式
 */
bool line_sensor_is_cross_line(void);

/**
 * @brief 判断当前是否为后续安全逻辑可消费的矛盾/非法模式
 * @return true 当前模式矛盾或非法
 * @return false 当前模式不是非法模式
 */
bool line_sensor_is_invalid(void);

#ifdef COMPETITION_VERIFY_MODE
/**
 * @brief 为仓库级验证注入一次灰度黑线 mask 输入
 * @param front_mask 前排黑线 mask
 * @param back_mask 后排黑线 mask
 * @param sample_ready false 时模拟底层数据尚未 ready
 */
void line_sensor_verify_set_masks(uint8_t front_mask, uint8_t back_mask, bool sample_ready);

/**
 * @brief 清除验证注入，恢复使用真实 gw_gray 数据
 */
void line_sensor_verify_clear_masks(void);
#endif

#endif
