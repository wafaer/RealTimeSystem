/// \file funct_command.c
///
/// 命令处理函数实现。
///
/// 该函数由 HAL 线程按固定周期调用，负责从 HAL 参数区读取外部命令输入，
/// 同步 PID 增益，并将运行状态（心跳、周期计数）写回 HAL 引脚。

#include "task.h"
#include "rtapi/rtapi.h"

/// \brief 命令处理函数（HAL 线程回调）。
///
/// 在持有 cmd_mutex 的前提下执行以下操作：
/// 1. 从 HAL 参数区读取 enable / pos_cmd / vel_cmd。
/// 2. 将使能位和位置指令写入轨迹发生器。
/// 3. 速度指令带有上下限保护（>0 且 <= 10000.0）。
/// 4. 将 HAL 参数区的 PID 增益（kp / ki / kd）同步到共享内存中的 PID 结构体。
/// 5. 将运行状态（heartbeat、cycle_count）写回 HAL 输出引脚。
///
/// \param arg    用户参数（本函数不使用，保留兼容性）。
/// \param period 当前线程的调度周期（纳秒），本函数不使用。
void taskCommandHandler(void *arg, long period) {
    (void)arg;
    hal_bit_t enable;
    float pos_cmd, vel_cmd;
    pid_ctrl_t pid_snap;

    if (!taskShared || !taskHalData) return;

    enable = *(hal_bit_t *)taskHalData->enable;
    pos_cmd = *(hal_float_t *)taskHalData->pos_cmd;
    vel_cmd = *(hal_float_t *)taskHalData->vel_cmd;

    rtapi_mutex_get(&taskShared->cmd_mutex);

    taskShared->tp.enable = (int)enable;
    taskShared->tp.pos_cmd = pos_cmd;

    // 速度指令带上下限保护。
    if (vel_cmd > 0.0f && vel_cmd <= 10000.0f) {
        taskShared->tp.vel = vel_cmd;
    }

    // 将 HAL 参数区的 PID 增益同步到共享内存结构体。
    pid_snap = taskShared->pid;
    taskShared->pid.kp = *taskHalData->pid_kp;
    taskShared->pid.ki = *taskHalData->pid_ki;
    taskShared->pid.kd = *taskHalData->pid_kd;

    // 将运行状态写回 HAL 输出引脚（若已连接）。
    if (taskHalData->heartbeat_out)
        *taskHalData->heartbeat_out = taskShared->heartbeat;
    if (taskHalData->cycle_count_out)
        *taskHalData->cycle_count_out = taskShared->cycle_count;

    rtapi_mutex_give(&taskShared->cmd_mutex);
}
