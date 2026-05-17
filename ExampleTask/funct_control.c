/// \file funct_control.c
///
/// 控制函数实现。
///
/// 该函数由 HAL 线程按固定周期调用，负责在一个周期内完成轨迹规划、
/// PID 位置控制、被控对象仿真，并将结果写回 HAL 引脚。

#include "task.h"
#include "rtapi/rtapi.h"
#include <math.h>

/// \brief 控制函数（HAL 线程回调）。
///
/// 在持有 cmd_mutex 的前提下执行以下 4 个步骤：
/// 1. 轨迹规划：调用 simple_tp_update 根据当前指令更新轨迹段状态。
/// 2. PID 位置控制：计算位置误差并执行 PID 更新。
/// 3. 电机仿真：对 PID 输出做限幅（[-100, 100]），按一阶系统仿真实际位置
///    （速度 = 输出，位置 = 速度对时间积分）。
/// 4. 将结果写回 HAL 引脚，并更新诊断计数器（心跳和周期计数）。
///
/// \param arg    用户参数（本函数不使用，保留兼容性）。
/// \param period 当前线程的调度周期（纳秒）。
void taskController(void *arg, long period) {
    (void)arg;
    float pos_err;
    float period_sec;
    float sim_output;

    if (!taskShared || !taskHalData) return;

    period_sec = (float)period * 1e-9f;

    rtapi_mutex_get(&taskShared->cmd_mutex);

    // ---- 1. 轨迹规划 ----
    simple_tp_update(&taskShared->tp, (int32_t)period);

    // ---- 2. PID 位置控制 ----
    pos_err = taskShared->tp.curr_pos - taskShared->actual_pos;
    pid_update(&taskShared->pid, pos_err, period_sec);

    // ---- 3. 电机仿真 ----
    // 将 PID 输出限幅到安全范围内。
    sim_output = taskShared->pid.output;
    if (sim_output >  100.0f) sim_output =  100.0f;
    if (sim_output < -100.0f) sim_output = -100.0f;

    // 一阶被控对象模型：速度 = 输出，位置 = 速度对时间积分。
    taskShared->actual_pos += sim_output * period_sec;

    // ---- 4. 将结果写回 HAL 引脚 ----
    *(hal_float_t *)taskHalData->planned_pos    = taskShared->tp.curr_pos;
    *(hal_float_t *)taskHalData->planned_vel    = taskShared->tp.curr_vel;
    *(hal_float_t *)taskHalData->actual_pos_out = taskShared->actual_pos;
    *(hal_float_t *)taskHalData->pid_out        = taskShared->pid.output;

    // 更新诊断计数器。
    taskShared->cycle_count++;
    taskShared->heartbeat++;

    rtapi_mutex_give(&taskShared->cmd_mutex);
}
