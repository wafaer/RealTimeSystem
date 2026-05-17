/// \file simple_tp.h
///
/// 简易梯形曲线轨迹规划器公共接口。

#ifndef SIMPLE_TP_H
#define SIMPLE_TP_H

#include <stdint.h>

/// \brief 位置阈值宏。
///
/// 用于判断当前位置是否已到达目标位置（偏差小于此值视为到达）。
/// 计算公式：accel * period^2 * 0.001。
///
/// \param accel   最大加速度。
/// \param period  调度周期（纳秒）。
#define TINY_DP(accel, period)  ((accel) * (period) * (period) * 0.001)

/// \brief 简易梯形轨迹规划器状态结构体。
///
/// 成员分为三类：配置输入（enable / pos_cmd / vel / acc）、
/// 状态输出（curr_pos / curr_vel / active）、内部状态（curr_pos 在规划过程中被推进）。
typedef struct simple_tp {
    int   enable;     ///< 轨迹规划器使能位，1=启用，0=保持原地。
    int   active;      ///< 运动活跃标志，由 simple_tp_update 更新。
    float pos_cmd;     ///< 命令目标位置（输入）。
    float curr_pos;    ///< 当前规划位置（输出）。
    float curr_vel;    ///< 当前规划速度（输出）。
    float vel;         ///< 最大允许速度（配置参数）。
    float acc;         ///< 最大加速度/减速度（配置参数）。
} simple_tp_t;

/// \brief 更新简易轨迹规划器状态。
///
/// \param tp     指向 simple_tp_t 结构体的指针。
/// \param period 当前调度周期（纳秒）。
void simple_tp_update(simple_tp_t *tp, int32_t period);

#endif  // SIMPLE_TP_H
