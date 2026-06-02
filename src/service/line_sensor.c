#include "line_sensor.h"

#include "gw_gray.h"

#include <math.h>
#include <stddef.h>

// ! ========================= 宏 定 义 ========================= ! //

#define LINE_SENSOR_PROBE_COUNT                8u
#define LINE_SENSOR_CENTER_INDEX               3.5f
#define LINE_SENSOR_WIDE_RUN_MIN_BITS          6u
#define LINE_SENSOR_CONTRADICT_DELTA_THRESHOLD 4.0f

// ! ========================= 私 有 Typedef 声 明 ========================= ! //

typedef struct {
    bool has_black;
    bool valid_cluster;
    bool wide_cluster;
    uint8_t black_count;
    uint8_t cluster_count;
    float center;
} LineStripInterpretation;

typedef struct {
    bool ready;
    uint8_t front_mask;
    uint8_t back_mask;
    uint32_t timestamp_us;
    uint32_t update_count;
} LineSensorSample;

#ifdef COMPETITION_VERIFY_MODE
typedef struct {
    bool enabled;
    bool sample_ready;
    uint8_t front_mask;
    uint8_t back_mask;
    uint32_t update_count;
} LineSensorVerifyOverride;
#endif

// ! ========================= 变 量 声 明 ========================= ! //

static LineSensorState s_line_sensor_state = { 0 };

#ifdef COMPETITION_VERIFY_MODE
static LineSensorVerifyOverride s_verify_override = { 0 };
#endif

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static LineSensorSample line_sensor_read_sample(void);
static LineStripInterpretation line_sensor_interpret_strip(uint8_t mask);
static void line_sensor_mark_not_ready(LineSensorState* state);
static void line_sensor_mark_no_line(LineSensorState* state);
static void line_sensor_mark_invalid(LineSensorState* state, const LineSensorSample* sample);
static float line_sensor_bit_position(uint8_t bit_index);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void line_sensor_update(void) {
    const LineSensorSample sample = line_sensor_read_sample();
    const LineStripInterpretation front = line_sensor_interpret_strip(sample.front_mask);
    const LineStripInterpretation back = line_sensor_interpret_strip(sample.back_mask);
    LineSensorState next_state = { 0 };

    next_state.front_mask = sample.front_mask;
    next_state.back_mask = sample.back_mask;
    next_state.timestamp_us = sample.timestamp_us;
    next_state.source_update_count = sample.update_count;

    if(!sample.ready) {
        line_sensor_mark_not_ready(&next_state);
        s_line_sensor_state = next_state;
        return;
    }

    next_state.line_detected = front.has_black || back.has_black;
    next_state.cross_line_detected = front.wide_cluster || back.wide_cluster;
    next_state.checkpoint_detected = front.wide_cluster && back.wide_cluster;

    if(front.cluster_count > 1u || back.cluster_count > 1u) {
        line_sensor_mark_invalid(&next_state, &sample);
        s_line_sensor_state = next_state;
        return;
    }

    if(front.valid_cluster && back.valid_cluster && fabsf(front.center - back.center) > LINE_SENSOR_CONTRADICT_DELTA_THRESHOLD) {
        line_sensor_mark_invalid(&next_state, &sample);
        s_line_sensor_state = next_state;
        return;
    }

    if(!next_state.line_detected) {
        line_sensor_mark_no_line(&next_state);
        s_line_sensor_state = next_state;
        return;
    }

    next_state.state_valid = true;
    next_state.invalid_pattern = false;
    next_state.status = LINE_SENSOR_STATUS_TRACK_VALID;

    if(front.valid_cluster && back.valid_cluster) {
        next_state.lateral_error = (front.center + back.center) * 0.5f;
        next_state.heading_error = front.center - back.center;
    } else if(front.valid_cluster) {
        next_state.lateral_error = front.center;
        next_state.heading_error = 0.0f;
    } else if(back.valid_cluster) {
        next_state.lateral_error = back.center;
        next_state.heading_error = 0.0f;
    }

    s_line_sensor_state = next_state;
}

const LineSensorState* line_sensor_get_state(void) {
    return &s_line_sensor_state;
}

bool line_sensor_has_valid_line(void) {
    return s_line_sensor_state.status == LINE_SENSOR_STATUS_TRACK_VALID;
}

bool line_sensor_is_checkpoint(void) {
    return s_line_sensor_state.checkpoint_detected;
}

bool line_sensor_is_cross_line(void) {
    return s_line_sensor_state.cross_line_detected;
}

bool line_sensor_is_invalid(void) {
    return s_line_sensor_state.invalid_pattern;
}

#ifdef COMPETITION_VERIFY_MODE
void line_sensor_verify_set_masks(uint8_t front_mask, uint8_t back_mask, bool sample_ready) {
    s_verify_override.enabled = true;
    s_verify_override.sample_ready = sample_ready;
    s_verify_override.front_mask = front_mask;
    s_verify_override.back_mask = back_mask;
    s_verify_override.update_count++;
}

void line_sensor_verify_clear_masks(void) {
    s_verify_override.enabled = false;
    s_verify_override.sample_ready = false;
    s_verify_override.front_mask = 0u;
    s_verify_override.back_mask = 0u;
    s_verify_override.update_count = 0u;
}
#endif

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static LineSensorSample line_sensor_read_sample(void) {
    LineSensorSample sample = { 0 };

#ifdef COMPETITION_VERIFY_MODE
    if(s_verify_override.enabled) {
        sample.ready = s_verify_override.sample_ready;
        sample.front_mask = s_verify_override.front_mask;
        sample.back_mask = s_verify_override.back_mask;
        sample.update_count = s_verify_override.update_count;
        sample.timestamp_us = s_verify_override.update_count;
        return sample;
    }
#endif

    {
        const GwGrayState* gray_state = gw_gray_get_state();
        if(gray_state == NULL || gray_state->update_count == 0u) {
            return sample;
        }

        sample.ready = true;
        sample.front_mask = gray_state->front_black;
        sample.back_mask = gray_state->back_black;
        sample.timestamp_us = gray_state->timestamp_us;
        sample.update_count = gray_state->update_count;
    }

    return sample;
}

static LineStripInterpretation line_sensor_interpret_strip(uint8_t mask) {
    LineStripInterpretation result = { 0 };
    float weighted_sum = 0.0f;
    bool in_cluster = false;

    for(uint8_t bit = 0u; bit < LINE_SENSOR_PROBE_COUNT; ++bit) {
        const bool black = ((mask >> bit) & 0x01u) != 0u;
        if(black) {
            result.has_black = true;
            result.black_count++;
            weighted_sum += line_sensor_bit_position(bit);
            if(!in_cluster) {
                result.cluster_count++;
                in_cluster = true;
            }
        } else {
            in_cluster = false;
        }
    }

    if(result.black_count > 0u) {
        result.center = weighted_sum / (float)result.black_count;
    }

    result.valid_cluster = result.has_black && result.cluster_count == 1u;
    result.wide_cluster = result.valid_cluster && result.black_count >= LINE_SENSOR_WIDE_RUN_MIN_BITS;

    return result;
}

static void line_sensor_mark_not_ready(LineSensorState* state) {
    state->line_detected = false;
    state->state_valid = false;
    state->invalid_pattern = false;
    state->checkpoint_detected = false;
    state->cross_line_detected = false;
    state->lateral_error = 0.0f;
    state->heading_error = 0.0f;
    state->status = LINE_SENSOR_STATUS_NOT_READY;
}

static void line_sensor_mark_no_line(LineSensorState* state) {
    state->line_detected = false;
    state->state_valid = false;
    state->invalid_pattern = false;
    state->lateral_error = 0.0f;
    state->heading_error = 0.0f;
    state->status = LINE_SENSOR_STATUS_NO_LINE;
}

static void line_sensor_mark_invalid(LineSensorState* state, const LineSensorSample* sample) {
    (void)sample;
    state->line_detected = true;
    state->state_valid = false;
    state->invalid_pattern = true;
    state->checkpoint_detected = false;
    state->cross_line_detected = false;
    state->lateral_error = 0.0f;
    state->heading_error = 0.0f;
    state->status = LINE_SENSOR_STATUS_INVALID_PATTERN;
}

static float line_sensor_bit_position(uint8_t bit_index) {
    return (float)bit_index - LINE_SENSOR_CENTER_INDEX;
}
