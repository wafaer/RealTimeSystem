/// \file timer.h
///
/// 实时系统计时器接口定义。
///
/// 提供 etime/esleep C 接口以及 C++ 层面的 RCS_TIMER 类，
/// 支持周期性同步等待、超时控制及调度负载统计。

#ifndef REALTIMESYSTEM_TIMER_H
#define REALTIMESYSTEM_TIMER_H

#include <stdio.h>

#include "_timer.h"

class RCS_SEMAPHORE;

/// \brief 用户自定义计时同步回调函数类型。
///
/// \param _arg 传递给回调函数的用户参数。
/// \return 0 同步成功；非 0 终止等待。
typedef int (*RCS_TIMERFUNC)(void *_arg);

/// \brief 周期定时器类。
///
/// 提供周期性同步等待、超时控制及调度负载统计功能。
/// 支持外部时间基准回调函数，用于与非系统时钟事件同步。
class RCS_TIMER {
  public:
    /// \brief 构造定时器（仅指定超时）。
    ///
    /// \param timeout  周期超时时长（秒）。
    /// \param function 可选的外部同步回调函数。
    /// \param arg      传递给回调函数的用户参数。
    RCS_TIMER(double timeout, RCS_TIMERFUNC function =
        (RCS_TIMERFUNC)NULL, void *arg = NULL);

    /// \brief 构造定时器（从配置文件初始化）。
    ///
    /// \param process_name       进程名称。
    /// \param timer_config_file  配置文件路径。
    RCS_TIMER(const char *process_name, const char *timer_config_file);

    /// \brief 构造定时器（指定超时和配置文件）。
    ///
    /// \param _timeout            周期超时时长（秒）。
    /// \param process_name       进程名称。
    /// \param timer_config_file  配置文件路径。
    RCS_TIMER(double _timeout, const char *process_name,
              const char *timer_config_file);

    /// \brief 销毁定时器，释放相关资源。
    ~RCS_TIMER();

    /// \brief 等待直到周期结束或回调函数返回。
    ///
    /// \return 0 成功；正数 错过的周期数；-1 发生错误。
    int wait();

    /// \brief 获取周期调度的 CPU 负载。
    ///
    /// \return 0.0 表示全部时间在等待；1.0 表示等待前已耗尽全部时间；
    ///     介于两者之间表示平均负载百分比。
    double load();

    /// \brief 立即重置等待周期。
    ///
    /// 从调用时刻重新开始计时。
    void sync();

    /// 当前设定的超时时长（秒）。
    double timeout;

  private:
    /// \brief 内部初始化。
    ///
    /// \param _timeout 超时值。
    /// \param _id      定时器标识。
    void init(double _timeout, int _id);

    /// \brief 重置定时器内部状态。
    void zero_timer();

    /// \brief 设置超时时长。
    ///
    /// \param _timeout 新的超时值（秒）。
    void set_timeout(double _timeout);

    /// 用户提供的同步回调函数。
    RCS_TIMERFUNC function;
    /// 传递给回调函数的用户参数。
    void *arg;
    /// 上一次唤醒时刻（自 epoch 起）。
    double last_time;
    /// 定时器创建时的 epoch 时刻。
    double start_time;
    /// 累计空闲时间。
    double idle;
    /// 累计等待次数。
    int counts;
    /// 距上次真正睡眠以来的等待次数。
    int counts_since_real_sleep;
    /// 每次真正睡眠前的等待次数阈值。
    int counts_per_real_sleep;
    /// 距上次真正睡眠以来的时间。
    double time_since_real_sleep;
#ifdef USE_SEMS_FOR_TIMER
    /// 信号量指针数组（条件编译）。
    RCS_SEMAPHORE **sems;
#endif
    /// 信号量数量。
    int num_sems;
    /// 定时器标识符。
    int id;
    /// 时钟滴答周期值（秒）。
    double clk_tck_val;
};

#endif  // REALTIMESYSTEM_TIMER_H
