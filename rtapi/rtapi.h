/// \file rtapi.h
///
/// RTAPI C 语言层公开接口声明。
///
/// 定义消息级别枚举、消息结构体、rtapi_msg_handler 回调类型，
/// 以及共享内存、任务管理、时钟、打印等全部外部函数的声明。
/// 所有函数均可被 C/C++ 代码调用（已用 extern "C" 包裹）。

#ifndef RTAPI_H
#define RTAPI_H

#include <stdio.h>
#include <bits/types/clockid_t.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \brief 消息日志级别枚举，值越小越严重。
typedef enum {
    RTAPI_MSG_NONE = 0,   ///< 不输出任何消息。
    RTAPI_MSG_ERR,       ///< 错误。
    RTAPI_MSG_WARN,      ///< 警告。
    RTAPI_MSG_INFO,      ///< 信息。
    RTAPI_MSG_DBG,       ///< 调试。
    RTAPI_MSG_ALL        ///< 所有级别。
} msg_level_t;

/// \brief 固定大小的日志消息结构体。
typedef struct {
    msg_level_t level;                              ///< 本条消息的级别。
    char msg[1024 - sizeof(msg_level_t)];           ///< 消息正文（不足 1024 字节的剩余空间）。
} message_t;

/// \brief 消息处理回调函数类型。
///
/// \param level 消息级别。
/// \param fmt  printf 风格的格式字符串。
/// \param ap    可变参数列表（va_list）。
typedef void (*rtapi_msg_handler_t)(msg_level_t level, const char *fmt, va_list ap);

// -----------------------------------------------------------------------------
// 消息打印
// -----------------------------------------------------------------------------

/// \brief 按指定级别打印格式化消息（rtapi 版本）。
///
/// \param level 消息级别。
/// \param fmt   printf 风格格式字符串。
/// \param ...   可变参数。
extern void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/// \brief 按指定级别打印格式化消息（内部版本）。
///
/// \param level 消息级别。
/// \param fmt   printf 风格格式字符串。
/// \param ...   可变参数。
extern void print_msg(msg_level_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/// \brief 安全字符串格式化，边界检查更严格。
///
/// \param buf  输出缓冲区。
/// \param size 缓冲区大小（字节）。
/// \param fmt  printf 风格格式字符串。
/// \param ...  可变参数。
/// \return 写入的字符数（不含终止符），失败返回负值。
extern int rt_snprintf(char *buf, unsigned long int size,
                       const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/// \brief rt_snprintf 的 va_list 版本。
///
/// \param buffer 输出缓冲区。
/// \param size   缓冲区大小（字节）。
/// \param fmt    格式字符串。
/// \param args   已解包的可变参数列表。
/// \return 写入的字符数（不含终止符），失败返回负值。
extern int rt_vsnprintf(char *buffer, unsigned long int size,
                        const char *fmt, va_list args);

// -----------------------------------------------------------------------------
// 时钟管理
// -----------------------------------------------------------------------------

/// \brief 设置全局时钟周期。
///
/// \param nsecs 周期长度（纳秒）。
/// \return 实际设置的周期值（可能经四舍五入），失败返回负值。
extern long int rtapi_clock_set_period(long int nsecs);

/// \brief 获取高精度时间戳（纳秒级）。
/// \return 距固定 Epoch 的累计纳秒数。
extern long long int rtapi_get_time(void);

/// \brief 获取自系统启动以来的时钟滴答数。
/// \return 时钟滴答累计值。
extern long long int rtapi_get_clocks(void);

/// \brief 基于指定时钟的 nanosleep，兼容 Linux 内核与用户空间实现。
///
/// \param clock_id  使用的时钟 ID（如 CLOCK_MONOTONIC）。
/// \param flags     标志位，TIMER_ABSTIME 表示 prequest 为绝对时间。
/// \param prequest  睡眠时长（或绝对时间）。
/// \param remain    若被信号中断，存放剩余时间（可为 nullptr）。
/// \param pnow      若非空，存放当前实际时间。
/// \return 成功返回 0，被信号中断返回 -1 并设置 errno。
extern int rtapi_clock_nanosleep(clockid_t clock_id, int flags,
                                 const struct timespec *prequest,
                                 struct timespec *remain,
                                 const struct timespec *pnow);

// -----------------------------------------------------------------------------
// 优先级
// -----------------------------------------------------------------------------

/// \brief 返回最高调度优先级值。
/// \return 最高优先级（数值最小）。
extern int rtapi_prio_highest(void);

/// \brief 返回小于 prio 的下一个有效优先级。
///
/// \param prio 当前优先级。
/// \return 下一个更低的优先级，若不存在则返回 prio_highest()。
extern int rtapi_prio_next_lower(int prio);

// -----------------------------------------------------------------------------
// 模块初始化
// -----------------------------------------------------------------------------

/// \brief 初始化 RTAPI 模块（用户空间侧）。
///
/// \param modname 模块名称字符串。
/// \return 成功返回模块 ID（>= 0），失败返回负值。
extern int rtapi_init(const char *modname);

/// \brief 卸载 RTAPI 模块，释放相关资源。
///
/// \param module_id 要卸载的模块 ID。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_exit(int module_id);

// -----------------------------------------------------------------------------
// 共享内存
// -----------------------------------------------------------------------------

/// \brief 创建或获取一个共享内存块。
///
/// \param key       共享内存的标识 key。
/// \param module_id  调用模块的 ID。
/// \param size       内存块大小（字节）。
/// \return 共享内存句柄（>= 0），失败返回负值。
extern int rtapi_shmem_new(int key, int module_id,
                           unsigned long int size);

/// \brief 删除指定的共享内存块。
///
/// \param handle    shmem_new 返回的句柄。
/// \param module_id 调用模块的 ID。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_shmem_delete(int handle, int module_id);

/// \brief 获取共享内存块的用户空间指针。
///
/// \param handle 句柄。
/// \param ptr    输出参数，指向映射后指针的指针。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_shmem_getptr(int handle, void **ptr);

/// \brief 查询共享内存块的当前状态。
///
/// \param handle 句柄。
/// \return 状态值（具体含义由实现定义），失败返回负值。
extern int rtapi_shmem_getstate(int handle);

// -----------------------------------------------------------------------------
// 任务管理
// -----------------------------------------------------------------------------

/// \brief 创建新的实时任务（用户空间侧）。
///
/// \param taskcode   任务入口函数。
/// \param arg        传递给 taskcode 的参数。
/// \param prio       任务优先级。
/// \param owner      创建者模块 ID。
/// \param stacksize  线程栈大小（字节）。
/// \param uses_fp    是否使用浮点单元（非零表示使用）。
/// \return 新任务的 ID（>= 0），失败返回负值。
extern int rtapi_task_new(void (*taskcode)(void *), void *arg,
                          int prio, int owner,
                          unsigned long int stacksize, int uses_fp);

/// \brief 启动任务按指定周期运行。
///
/// \param task_id      任务 ID。
/// \param period_nsec  调度周期（纳秒）。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_start(int task_id, unsigned long int period_nsec);

/// \brief 暂停指定任务的执行。
///
/// \param task_id 任务 ID。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_pause(int task_id);

/// \brief 恢复已暂停任务的执行。
///
/// \param task_id 任务 ID。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_resume(int task_id);

/// \brief 删除指定任务并释放所有关联资源。
///
/// \param task_id 任务 ID。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_delete(int task_id);

/// \brief 返回当前任务的 ID。
/// \return 当前任务 ID。
extern int rtapi_task_self(void);

/// \brief 查询指定任务的当前状态。
///
/// \param task_id 任务 ID。
/// \return 任务状态值（对应 task_state_t 枚举），失败返回负值。
extern int rtapi_task_getstate(int task_id);

// -----------------------------------------------------------------------------
// PLL 校正
// -----------------------------------------------------------------------------

/// \brief 获取 PLL 参考时钟的累计计数值。
/// \return 参考时钟计数值。
extern long long rtapi_task_pll_get_reference(void);

/// \brief 设置 PLL 校正值，用于实时任务的时钟微调。
///
/// \param value 要设置的校正值。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_pll_set_correction(long value);

/// \brief 获取 PLL 当前校正值。
///
/// \param value 输出参数，存放当前校正值。
/// \return 成功返回 0，失败返回负值。
extern int rtapi_task_pll_get_correction(long *value);

// -----------------------------------------------------------------------------
// 线程与调度
// -----------------------------------------------------------------------------

/// \brief 创建 POSIX 线程。
///
/// \param thread    输出参数，存放新线程的 pthread_t。
/// \param func      线程入口函数。
/// \param arg       传递给 func 的参数。
/// \param priority  线程优先级。
/// \param detached  是否以 detached 状态创建（非零表示 detached）。
/// \return 成功返回 0，失败返回负值。
extern int create_thread(pthread_t *thread,
                        void *(*func)(void *), void *arg,
                        int priority, int detached);

/// \brief 任务等待下一个调度周期（由子类实现的具体等待逻辑）。
extern void rtapi_wait(void);

// -----------------------------------------------------------------------------
// printf 控制（用于多进程/多线程环境下的输出同步）
// -----------------------------------------------------------------------------

/// \brief 启用主进程/主线程的 printf 接管。
/// \return 成功返回 0，失败返回负值。
extern int printfMaster();

/// \brief 停止主进程/主线程的 printf 接管，恢复默认行为。
/// \return 成功返回 0，失败返回负值。
extern int stopprintf();

// -----------------------------------------------------------------------------
// 默认消息处理器
// -----------------------------------------------------------------------------

/// \brief rtapi 版本默认消息处理函数，输出至 stdout/stderr。
///
/// \param level 消息级别。
/// \param fmt   格式字符串。
/// \param ap    可变参数列表。
extern void default_rtapi_msg_handler(msg_level_t level,
                                      const char *fmt, va_list ap);

/// \brief 内部版本默认消息处理函数，输出至 stdout/stderr。
///
/// \param level 消息级别。
/// \param fmt   格式字符串。
/// \param ap    可变参数列表。
extern void default_msg_handler(msg_level_t level,
                                const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif //RTAPI_H
