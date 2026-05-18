#include "remote.h"

#include "chassis.h"
#include "fs_ia10b.h"

#include <string.h>

#define REMOTE_CH_RIGHT_X 0u
#define REMOTE_CH_RIGHT_Y 1u
#define REMOTE_CH_LEFT_Y 2u
#define REMOTE_CH_LEFT_X  3u

#define REMOTE_CH_SWA 4u
#define REMOTE_CH_SWB 5u
#define REMOTE_CH_SWC 6u
#define REMOTE_CH_SWD 7u

#define REMOTE_CH_VRA 8u
#define REMOTE_CH_VRB 9u

#define REMOTE_CENTER      1500
#define REMOTE_SPAN        500.0f
#define REMOTE_DEADBAND    30u
#define REMOTE_TIMEOUT_MS  100u

#define REMOTE_MAX_VX_MPS   2.0f
#define REMOTE_MAX_VY_MPS   2.0f
#define REMOTE_MAX_WZ_RAD_S 6.0f

#define REMOTE_BRAKE_THRESHOLD 1200u

static RemoteCommand s_command = { 0 };

static float remote_channel_to_norm(uint16_t value, uint16_t deadband);

void remote_init(void) {
    memset(&s_command, 0, sizeof(s_command));
}

void remote_process(void) {
    FsIa10bData rc_data;

    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS)) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = false;
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    if(rc_data.channel[REMOTE_CH_VRA] <= REMOTE_BRAKE_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = true;
        (void)chassis.brake();
        return;
    }

    s_command.vx = remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND) * REMOTE_MAX_VX_MPS;
    s_command.vy = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND) * REMOTE_MAX_VY_MPS;
    s_command.wz = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND) * REMOTE_MAX_WZ_RAD_S;
    s_command.online = true;

    (void)chassis.set_velocity(s_command.vx, s_command.vy, s_command.wz);
}

bool remote_get_command(RemoteCommand* out) {
    if(out == NULL) {
        return false;
    }

    *out = s_command;
    return s_command.online;
}

static float remote_channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - REMOTE_CENTER;
    float normalized;

    if(diff < 0) {
        if((uint32_t)(-diff) <= deadband) {
            return 0.0f;
        }
    }
    else {
        if((uint32_t)diff <= deadband) {
            return 0.0f;
        }
    }

    normalized = (float)diff / REMOTE_SPAN;
    if(normalized > 1.0f) {
        return 1.0f;
    }
    if(normalized < -1.0f) {
        return -1.0f;
    }

    return normalized;
}
