/// \file simple_tp.c
///
/// 简易梯形曲线轨迹规划器实现。
///
/// 采用梯形速度曲线（T型加减速），每个调度周期根据目标位置与当前位置的
/// 偏差计算满足加速度约束的速度指令。

#include "simple_tp.h"
#include <math.h>

/// \brief 更新简易轨迹规划器状态（梯形曲线速度规划）。
///
/// 在 enable == 1 时，根据当前位置与目标位置的偏差计算下一刻的速度指令；
/// 在 enable == 0 时保持原地不动。每次调用均更新 curr_vel 和 curr_pos。
///
/// 算法分为以下阶段：
/// 1. 计算加速度约束下单周期最大速度变化量（max_dv）和最小位置阈值（tiny_dp）。
/// 2. 根据偏差方向选择速度计算公式（正/负方向分别处理）。
/// 3. 速度限幅：vel_req 不得超过 tp->vel 设置的最大速度。
/// 4. 速度斜坡：curr_vel 以 max_dv 为步长向 vel_req 靠拢。
/// 5. 若 curr_vel 非零则标记为 active，最后推进位置。
///
/// \param tp     指向 simple_tp_t 轨迹状态结构体的指针。
/// \param period 当前调度周期（纳秒），用于计算 period_s（秒）。
void simple_tp_update(simple_tp_t *tp, int32_t period) {
    double max_dv, tiny_dp, pos_err, vel_req;
    double period_s = (double)period * 1e-9;

    tp->active = 0;
    max_dv = tp->acc * period_s;
    tiny_dp = TINY_DP(tp->acc, period_s);

    if (tp->enable) {
        pos_err = tp->pos_cmd - tp->curr_pos;

        if (pos_err > tiny_dp) {
            // 正向运动：采用加速公式。
            vel_req = -max_dv + sqrt(2 * tp->acc * pos_err + max_dv * max_dv);
            tp->active = 1;
        } else if (pos_err < -tiny_dp) {
            // 负向运动：采用对称的减速/反向公式。
            vel_req = max_dv - sqrt(2 * tp->acc * (-pos_err) + max_dv * max_dv);
            tp->active = 1;
        } else {
            // 已到达目标（偏差在阈值内），停止运动并清除使能。
            vel_req = 0;
            tp->enable = 0;
        }
    } else {
        // 未使能时保持原地（pos_cmd 锁定为 curr_pos）。
        vel_req = 0;
        tp->pos_cmd = tp->curr_pos;
    }

    // 速度限幅：vel_req 不得超过配置的最大速度。
    if (vel_req > tp->vel) {
        vel_req = tp->vel;
    } else if (vel_req < -tp->vel) {
        vel_req = -tp->vel;
    }

    // 速度斜坡：以 max_dv 为步长逐步逼近目标速度（防止加速度突变）。
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

    // 按当前速度推进位置。
    tp->curr_pos += tp->curr_vel * period_s;
}
