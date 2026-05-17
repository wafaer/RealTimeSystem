/// \file task.h
///
/// 实时伺服任务公共接口定义。
///
/// 包含 PID 控制结构体、共享内存布局、HAL 引脚/参数结构体及函数声明。

#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtapi/rtapi_mutex.h"
#include "hal/hal.h"
#include "simple_tp.h"

/// 默认调度周期（纳秒），1 ms。
#define DEFAULT_TASK_PERIOD_NSEC   1000000

/// 任务共享内存的 System V IPC 键值（ASCII "TSK1"）。
#define TASK_SHM_KEY     0x54534B31    // "TSK1"

/// 共享内存头部标识魔数（ASCII "TSHM"），用于识别本模块创建的段。
#define TASK_SHM_MAGIC    0x5453484Du  // "TSHM"
/// 共享内存版本号，用于段格式兼容性检测。
#define TASK_SHM_VERSION  1u

/// PID 增益及输出限幅常量。
///@{
#define PID_KP_MIN  0.0f
#define PID_KP_MAX  10000.0f
#define PID_KI_MIN  0.0f
#define PID_KI_MAX  1000.0f
#define PID_KD_MIN  0.0f
#define PID_KD_MAX  10000.0f
#define PID_OUT_MAX  1000.0f
#define PID_OUT_MIN  -1000.0f
///@}

/// \brief 增量式 PID 控制器结构体。
///
/// kp/ki/kd 为用户配置的增益；prev_error/integral/output 为内部状态，
/// 由 pid_init / pid_update / pid_reset 维护。
typedef struct {
    float kp;           ///< 比例增益。
    float ki;           ///< 积分增益。
    float kd;           ///< 微分增益。
    float prev_error;    ///< 上一次采样的位置误差。
    float integral;      ///< 误差积分累计值。
    float output;        ///< PID 输出（电压/力矩指令）。
} pid_ctrl_t;

/// \brief 初始化 PID 控制器状态。
///
/// 将所有增益和内部状态复位为 0。
///
/// \param p 指向 pid_ctrl_t 的指针。
static inline void pid_init(pid_ctrl_t *p) {
    p->kp = 0.0f; p->ki = 0.0f; p->kd = 0.0f;
    p->prev_error = 0.0f; p->integral = 0.0f; p->output = 0.0f;
}

/// \brief 执行一次 PID 计算（位置式）。
///
/// 计算公式：output = kp * error + ki * integral(error) + kd * d(error)/dt。
///
/// \param p           指向 pid_ctrl_t 的指针。
/// \param error       当前采样时刻的位置误差（目标位置 - 实际位置）。
/// \param period_sec  采样周期（秒）。
static inline void pid_update(pid_ctrl_t *p, float error, float period_sec) {
    float p_term, d_term;
    p->integral += error * period_sec;
    p_term = p->kp * error;
    d_term = (period_sec > 0.0f)
                 ? p->kd * (error - p->prev_error) / period_sec
                 : 0.0f;
    p->output = p_term + p->ki * p->integral + d_term;
    p->prev_error = error;
}

/// \brief 重置 PID 内部状态（积分和输出）。
///
/// 保留增益 kp/ki/kd，仅清除 prev_error / integral / output。
///
/// \param p 指向 pid_ctrl_t 的指针。
static inline void pid_reset(pid_ctrl_t *p) {
    p->prev_error = 0.0f;
    p->integral  = 0.0f;
    p->output    = 0.0f;
}

/// \brief 任务共享内存结构体。
///
/// 分配于 System V 共享内存，可被 HAL 线程和外部组件同时访问。
/// 前 12 字节为头部标识（magic/version/creator_pid），用于孤儿段检测；
/// 须保持在结构体最前端。
typedef struct {
    /// \name 头部标识（须保持在最前）
    ///@{
    uint32_t shm_magic;       ///< TASK_SHM_MAGIC，已初始化时写入。
    uint32_t shm_version;     ///< TASK_SHM_VERSION，当前版本号。
    pid_t    creator_pid;     ///< 创建者的 getpid()。
    ///@}

    /// 访问共享内存的互斥锁。
    rtapi_mutex_t cmd_mutex;

    /// 梯形轨迹规划器状态。
    simple_tp_t  tp;
    /// PID 控制器状态。
    pid_ctrl_t   pid;

    /// 仿真电机实际位置（由一阶系统积分得到）。
    float        actual_pos;
    /// PID 输出（力矩/电压指令）。
    float        pid_output;

    /// 心跳计数器（每个周期递增），用于检测任务是否运行。
    int          heartbeat;
    /// 周期计数（每个周期递增）。
    int          cycle_count;
    /// 上一次线程唤醒时间戳（纳秒）。
    long long    last_wakeup_ns;
} task_shared_t;

/// \brief 任务 HAL 引脚和参数结构体。
///
/// 所有成员由 hal_malloc 分配于 HAL 共享内存，通过 HAL pin/param 机制
/// 与外部组件交换数据。
typedef struct {
    /// \name 输入引脚（由外部 HAL 组件写入）
    ///@{
    void *enable;         ///< HAL_BIT — 总使能信号。
    void *pos_cmd;        ///< HAL_FLOAT — 目标位置指令。
    void *vel_cmd;        ///< HAL_FLOAT — 最大速度（运行时可调）。
    ///@}

    /// \name 输出引脚（由外部 HAL 组件读取）
    ///@{
    void *planned_pos;    ///< HAL_FLOAT — 轨迹规划后的目标位置。
    void *planned_vel;    ///< HAL_FLOAT — 轨迹规划后的目标速度。
    void *actual_pos_out; ///< HAL_FLOAT — 仿真得到的实际位置。
    void *pid_out;        ///< HAL_FLOAT — PID 输出。
    ///@}

    /// \name HAL 参数（HAL_RW，可由外部组件读写）
    ///@{
    hal_float_t *tp_vel;         ///< 轨迹规划最大速度。
    hal_float_t *tp_acc;         ///< 轨迹规划最大加速度。
    hal_float_t *pid_kp;         ///< PID 比例增益。
    hal_float_t *pid_ki;         ///< PID 积分增益。
    hal_float_t *pid_kd;         ///< PID 微分增益。
    hal_s32_t   *heartbeat_out;  ///< 心跳输出（诊断用）。
    hal_s32_t   *cycle_count_out;///< 周期计数输出（诊断用）。
    ///@}
} task_hal_data_t;

///@{
/// 全局变量，定义于 task.c。
extern int mot_comp_id;
extern int emc_shmem_id;
extern task_shared_t *taskShared;
extern task_hal_data_t *taskHalData;
///@}

/// \brief 启动任务线程（入口函数，由调用者执行）。
///
/// \return 0 成功；-1 失败。
int  task_thread_main(void);

/// \brief 退出任务线程并释放资源。
void task_thread_exit(void);

/// \brief 设置任务调度周期。
///
/// \param nsec 新的周期值（纳秒）。
void taskSetCycleTime(unsigned long nsec);

/// \brief 命令处理函数（HAL 线程回调）。
///
/// \param arg    用户参数（不使用）。
/// \param period 当前线程调度周期（纳秒）。
extern void taskCommandHandler(void *arg, long period);

/// \brief 控制函数（HAL 线程回调）。
///
/// \param arg    用户参数（不使用）。
/// \param period 当前线程调度周期（纳秒）。
extern void taskController(void *arg, long period);

#ifdef __cplusplus
}
#endif

#endif  // TASK_H
