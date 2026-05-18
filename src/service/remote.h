#ifndef _remote_h_
#define _remote_h_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float vx;
    float vy;
    float wz;
    bool online;
} RemoteCommand;

void remote_init(void);
void remote_process(void);
bool remote_get_command(RemoteCommand* out);

#endif
