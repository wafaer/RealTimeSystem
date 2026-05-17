#include "simple_tp.h"
#include <math.h>

void simple_tp_update(simple_tp_t *tp, int32_t period)
{
    double max_dv, tiny_dp, pos_err, vel_req;
    double period_s = (double)period * 1e-9;

    tp->active = 0;
    max_dv = tp->acc * period_s;
    tiny_dp = TINY_DP(tp->acc, period_s);

    if (tp->enable) {
        pos_err = tp->pos_cmd - tp->curr_pos;

        if (pos_err > tiny_dp) {
            vel_req = -max_dv + sqrt(2 * tp->acc * pos_err + max_dv * max_dv);
            tp->active = 1;
        } else if (pos_err < -tiny_dp) {
            vel_req = max_dv - sqrt(2 * tp->acc * (-pos_err) + max_dv * max_dv);
            tp->active = 1;
        } else {
            vel_req = 0;
            tp->enable = 0;
        }
    } else {
        vel_req = 0;
        tp->pos_cmd = tp->curr_pos;
    }

    if (vel_req > tp->vel) {
        vel_req = tp->vel;
    } else if (vel_req < -tp->vel) {
        vel_req = -tp->vel;
    }

    if (vel_req > tp->curr_vel + max_dv) {
        tp->curr_vel += max_dv;
    } else if (vel_req < tp->curr_vel - max_dv) {
        tp->curr_vel -= max_dv;
    } else {
        tp->curr_vel = vel_req;
    }

    if (tp->curr_vel != 0) {
        tp->active = 1;
    }

    tp->curr_pos += tp->curr_vel * period_s;
}
