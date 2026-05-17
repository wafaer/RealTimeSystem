/// \file rtapi_uspace.h
///
/// RTAPI 用户空间实现的类型与数据结构定义。
///
/// 定义实时任务的声明周期状态机、rtapi_task 结构体、RtapiApp 应用基类
/// 以及相关的辅助宏与全局变量。

#ifndef RTAPI_USPACE_H
#define RTAPI_USPACE_H

#ifdef __linux__
#include <sys/fsuid.h>
#endif
#include <atomic>
#include <unistd.h>
#include <pthread.h>

/// \brief 将两个 timespec 相加，结果写入 result。
///
/// \param result 存放结果的 timespec 结构。
/// \param ta 被加数。
/// \param tb 加数。
/// 若纳秒部分超过 1 秒，则自动进位到秒字段。
inline void rtapi_timespec_add(timespec &result, const timespec &ta, const timespec &tb) {
    result.tv_sec = ta.tv_sec + tb.tv_sec;
    result.tv_nsec = ta.tv_nsec + tb.tv_nsec;
    if (result.tv_nsec >= 1000000000) {
        result.tv_sec++;
        result.tv_nsec -= 1000000000;
    }
}

/// \brief 比较两个 timespec 的大小。
///
/// \param ta 左侧操作数。
/// \param tb 右侧操作数。
/// \return ta < tb 返回 true，否则返回 false。
inline bool rtapi_timespec_less(const struct timespec &ta, const struct timespec &tb) {
    if(ta.tv_sec < tb.tv_sec) return 1;
    if(ta.tv_sec > tb.tv_sec) return 0;
    return ta.tv_nsec < tb.tv_nsec;
}

/// \brief 在 src 基础上推进 nsec 纳秒，结果写入 result。
///
/// \param result 存放结果的 timespec 结构。
/// \param src 起始时间。
/// \param nsec 要推进的纳秒数。
void rtapi_timespec_advance(struct timespec &result, const struct timespec &src, unsigned long nsec);

// -----------------------------------------------------------------------------
// 实时任务生命周期状态机。
//
// 每个 rtapi_task 在以下状态间转换：
//
//   EMPTY -> INIT (CAS) -> ALLOCATED -> RUNNING <-> PAUSED
//                            |            |            |
//                            v            v            v
//                         DELETED      DELETED      DELETED
//
// EMPTY:     槽位空闲，task_array[n] == NULL。
// INIT:      allocate_task_id() 设定的 CAS 占位符 (TASK_MAGIC_INIT)。
// ALLOCATED: 对象已构造，幻数已写入，可以启动。
// RUNNING:   pthread 已创建，正在执行 taskcode / rtapi_wait 循环。
// PAUSED:    任务阻塞在暂停信号量上，等待恢复。
// DELETED:   线程已取消并 join，对象已释放，槽位已归还。
// ERROR:     不可恢复的错误。
// -----------------------------------------------------------------------------

/// \brief 实时任务的生命周期状态枚举。
enum {
    TASK_S_EMPTY     = 0,   ///< 槽位空闲，未分配任务。
    TASK_S_INIT      = 1,   ///< CAS 占位符（对应 TASK_MAGIC_INIT）。
    TASK_S_ALLOCATED = 2,   ///< 对象已构造，幻数已写入。
    TASK_S_RUNNING   = 3,   ///< pthread 已激活，正在执行。
    TASK_S_PAUSED    = 4,   ///< 任务阻塞在暂停信号量上。
    TASK_S_DELETED   = 5,   ///< 线程已取消，对象已释放，槽位已归还。
    TASK_S_ERROR     = -1   ///< 不可恢复的错误。
};

/// \brief 临时提升进程权限的 RAII 作用域守卫。
///
/// 构造时提升至 root 权限，析构时恢复原权限。
struct WithRoot {
    WithRoot();
    ~WithRoot();
    /// 当前嵌套的权限提升深度，供嵌套调用正确配平。
    static std::atomic<int> level;
};

/// \brief 实时任务的核心数据结构。
///
/// 保存任务的调度参数、生命周期状态以及崩溃错误信息。
struct rtapi_task {
    rtapi_task();

    int magic;                       ///< 幻数，用于校验句柄有效性。
    int id;                          ///< 全局唯一的任务 ID。
    int owner;                       ///< 创建该任务的模块 ID。
    int uses_fp;                     ///< 是否使用浮点单元（非零表示使用）。
    size_t stacksize;                ///< 线程栈大小（字节）。
    int prio;                        ///< 任务优先级（数值越小优先级越高）。
    long period;                     ///< 调度周期（纳秒）。
    struct timespec nextstart;       ///< 下一次调度的起始时间。
    unsigned ratio;                  ///< 周期分频比。
    long pll_correction;              ///< PLL 相位微调值（纳秒）。
    long pll_correction_limit;       ///< PLL 校正值的上限。
    void *arg;                       ///< 传递给 taskcode 的用户参数。
    void (*taskcode)(void *);        ///< 任务入口函数指针。

    /// 当前所处生命周期阶段，取值为 TASK_S_* 枚举值之一。
    /// 构造函数将其初始化为 TASK_S_EMPTY。
    int state;
    /// 创建该任务的进程 PID，在 task_new() 中设置。
    pid_t owner_pid;
    /// 最近一次 OS 操作失败时捕获的 errno 值，正常时为 0。
    int error_code;
};

/// \brief RTAPI 用户空间应用基类。
///
/// 封装实时任务的创建、调度、暂停/恢复、删除等生命周期管理接口。
/// 具体调度策略（如 pthread 或 Xenomai 线程）由子类实现。
struct RtapiApp {
    /// \param policy 调度策略，默认为 SCHED_FIFO。
    explicit RtapiApp(int policy = SCHED_FIFO) : policy(policy), period(0) {}

    /// \brief 返回最高优先级值。
    virtual int prio_highest() const;
    /// \brief 返回最低优先级值。
    virtual int prio_lowest() const;
    /// \brief 返回相邻优先级之间的增量。
    int prio_higher_delta() const;
    /// \brief 同 prio_higher_delta()。
    int prio_bound(int prio) const;
    /// \brief 返回大于 prio 的下一个有效优先级。
    /// \param prio 当前优先级。
    /// \return 下一个更高的优先级，若不存在则返回 prio_lowest()。
    int prio_next_higher(int prio) const;
    /// \brief 返回小于 prio 的下一个有效优先级。
    /// \param prio 当前优先级。
    /// \return 下一个更低的优先级，若不存在则返回 prio_highest()。
    int prio_next_lower(int prio) const;
    /// \brief 设置全局时钟周期。
    /// \param period_nsec 周期长度（纳秒）。
    /// \return 实际设置的周期值（可能经四舍五入）。
    long clock_set_period(long int period_nsec);

    /// \brief 创建新任务。
    ///
    /// \param taskcode 任务入口函数。
    /// \param arg 传递给 taskcode 的参数。
    /// \param prio 任务优先级。
    /// \param owner 创建者模块 ID。
    /// \param stacksize 线程栈大小（字节）。
    /// \param uses_fp 是否使用浮点单元。
    /// \return 新任务的 ID，失败返回负值。
    int task_new(void (*taskcode)(void *), void *arg,
                 int prio, int owner, unsigned long int stacksize, int uses_fp);
    /// \brief 子类实现：真正分配并构造 rtapi_task 对象。
    virtual rtapi_task *do_task_new() = 0;

    /// \brief 原子地分配一个新的任务 ID（基于 CAS）。
    /// \return 分配到的任务 ID，失败返回负值。
    static int allocate_task_id();
    /// \brief 根据任务 ID 查找对应的 rtapi_task 指针。
    /// \param task_id 任务 ID。
    /// \return 指向任务的指针，未找到返回 nullptr。
    static struct rtapi_task *get_task(int task_id);

    /// \brief 报告意外的实时延迟。
    ///
    /// \param task 发生延迟的任务。
    /// \param nperiod 连续延迟的周期数，默认为 1。
    void unexpected_realtime_delay(rtapi_task *task, int nperiod = 1);

    /// \brief 删除指定任务，释放所有关联资源。
    /// \param id 要删除的任务 ID。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_delete(int id) = 0;
    /// \brief 启动任务按指定周期运行。
    ///
    /// \param task_id 任务 ID。
    /// \param period_nsec 周期长度（纳秒）。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_start(int task_id, unsigned long period_nsec) = 0;
    /// \brief 暂停指定任务的执行。
    /// \param task_id 任务 ID。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_pause(int task_id) = 0;
    /// \brief 恢复已暂停任务的执行。
    /// \param task_id 任务 ID。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_resume(int task_id) = 0;
    /// \brief 返回当前任务的 ID。
    /// \return 当前任务的 ID。
    virtual int task_self() = 0;

    /// \brief 获取 PLL 参考时钟的当前计数值。
    /// \return 参考时钟的累计计数。
    virtual long long task_pll_get_reference(void) = 0;
    /// \brief 设置 PLL 校正值。
    /// \param value 要设置的校正值。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_pll_set_correction(long value) = 0;
    /// \brief 获取 PLL 当前校正值。
    /// \param value 存放结果的指针。
    /// \return 成功返回 0，失败返回负值。
    virtual int task_pll_get_correction(long *value) = 0;

    /// \brief 等待下一个调度周期（子类实现）。
    virtual void wait() = 0;
    /// \brief 在指定文件描述符上运行回调循环（子类实现）。
    ///
    /// \param fd 监视的文件描述符。
    /// \param callback 每触发一次调用的回调函数。
    /// \return 成功返回 0，失败返回负值。
    virtual int run_threads(int fd, int (*callback)(int fd)) = 0;
    /// \brief 获取高精度时间戳（子类实现）。
    /// \return 距某个固定Epoch的纳秒数。
    virtual long long do_get_time(void) = 0;
    /// \brief 延迟指定纳秒后返回（子类实现）。
    /// \param ns 要延迟的纳秒数。
    virtual void do_delay(long ns) = 0;

    int policy;   ///< 调度策略（如 SCHED_FIFO）。
    long period;  ///< 当前配置的调度周期（纳秒）。
};

/// \brief 根据任务 ID 安全地获取任务指针的模板辅助函数。
///
/// \tparam T 任务结构体类型，默认为 rtapi_task。
/// \param task_id 任务 ID。
/// \return 转换为 T* 类型后的任务指针，若不存在则返回 nullptr。
template<class T = rtapi_task>
T *rtapi_get_task(int task_id) {
    return static_cast<T*>(RtapiApp::get_task(task_id));
}

/// 任务数组的最大槽位数。
#define MAX_TASKS  64
/// 任务结构体的幻数签名，用于校验句柄有效性。
#define TASK_MAGIC    21979
/// allocate_task_id() 中的 CAS 占位符，表示该槽位正在被初始化。
#define TASK_MAGIC_INIT   ((rtapi_task*)(-1))

/// 全局任务指针数组，按任务 ID 索引，下标 0 保留未使用。
extern struct rtapi_task *task_array[MAX_TASKS];

/// \brief 在当前作用域临时提升至 root 权限的宏。
///
/// 等价于在作用域开头声明 `WithRoot root;`，
/// 构造时提升权限，作用域结束时析构自动恢复。
#define WITH_ROOT WithRoot root

#endif //RTAPI_USPACE_H
