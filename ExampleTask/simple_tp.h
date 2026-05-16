#ifndef SIMPLE_TP_H
#define SIMPLE_TP_H

#include <stdint.h>

#define TINY_DP(accel, period)  ((accel) * (period) * (period) * 0.001)

typedef struct simple_tp {
    int     enable;     // trajectory planner enable
    int     active;     // motion active flag (set by update fn)
    float   pos_cmd;    // commanded target position
    float   curr_pos;   // current planned position (output)
    float   curr_vel;   // current planned velocity (output)
    float   vel;        // max velocity
    float   acc;        // max accel / decel
} simple_tp_t;

void simple_tp_update(simple_tp_t *tp, int32_t period);

#endif // SIMPLE_TP_H
