/// \file uspace_rtapi_app.cpp
///
/// POSIX 用户空间 RTAPI 应用实现。
///
/// 实现基于 pthread/SCHED_FIFO 的实时任务调度器 PosixTask/Posix 子类，
/// 提供任务创建、启动、暂停/恢复、删除、PLL 校正等全部生命周期管理；
/// 同时实现 rtapi.h 中声明的所有 C 包装函数以及消息处理和时钟接口。

#ifdef __linux__
#include <sys/fsuid.h>
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <signal.h>
#include <semaphore.h>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#ifdef HAVE_SYS_IO_H
#include <sys/io.h>
#endif
#include <sys/resource.h>
#include <sys/mman.h>
#ifdef __linux__
#include <malloc.h>
#include <sys/prctl.h>
#endif
#ifdef __FreeBSD__
#include <pthread_np.h>
#endif
#include <chrono>
#include <stdarg.h>
#include <thread>

#include "hal/hal.h"
#include <boost/lockfree/queue.hpp>
#include "rtapi_uspace.h"

#define RTAPI_CLOCK (CLOCK_MONOTONIC)

/// WithRoot 的嵌套深度计数器，首次构造时提升至 root 权限，末次析构时恢复。
std::atomic<int> WithRoot::level;
/// 进程的有效 UID 和真实 UID，用于权限提升/恢复。
uid_t euid, ruid;

#include "rtapi/uspace_common.h"

/// 无锁消息队列，用于在多线程环境中异步传递日志消息。
boost::lockfree::queue<message_t, boost::lockfree::capacity<128>> rtapi_msg_queue;

// -----------------------------------------------------------------------------
// 权限守卫
// -----------------------------------------------------------------------------

/// 构造时递增权限深度计数；若为首次进入（level 从 0 变为 1），
/// 则在 Linux 上调用 setfsuid(euid) 提升至 root 权限。
WithRoot::WithRoot() {
    if (!level++) {
#ifdef __linux__
        setfsuid(euid);
#endif
    }
}

/// 析构时递减权限深度计数；若深度归零（level 从 1 变为 0），
/// 则恢复原始的真实用户 ID。
WithRoot::~WithRoot() {
    if (!--level) {
#ifdef __linux__
        setfsuid(ruid);
#endif
    }
}

// -----------------------------------------------------------------------------
// 内部工具命名空间
// -----------------------------------------------------------------------------

namespace {
    RtapiApp &App();

    /// 将当前线程名称设置为格式化字符串。
    ///
    /// \param fmt  printf 风格格式字符串。
    /// \param ap    已解包的可变参数列表。
    static void set_namef(const char *fmt, ...) {
        char *buf = NULL;
        va_list ap;
        va_start(ap, fmt);
        if (vasprintf(&buf, fmt, ap) < 0) {
            va_end(ap);
            return;
        }
        va_end(ap);

        int res = pthread_setname_np(pthread_self(), buf);
        if (res) {
            fprintf(stderr, "pthread_setname_np() failed for %s: %d\n", buf, res);
        }
        free(buf);
    }
}

/// 主线程的 pthread_t 句柄，用于后续引用主线程。
static pthread_t main_thread{};

// -----------------------------------------------------------------------------
// 模块与内存常量
// -----------------------------------------------------------------------------

/// 模块结构体，仅含幻数字段。
struct rtapi_module {
    int magic;
};

#define MODULE_MAGIC  30812   ///< rtapi_module 的幻数签名。
#define SHMEM_MAGIC   25453   ///< 共享内存句柄的幻数签名。

#define MAX_MODULES  64       ///< 最大模块数。
#define MODULE_OFFSET 32768   ///< 模块 ID 的偏移量。

// -----------------------------------------------------------------------------
// rtapi_task 构造函数
// -----------------------------------------------------------------------------

/// 默认构造函数，将所有字段零初始化，
/// 并将 state 初始化为 TASK_S_EMPTY，owner_pid 为 0，error_code 为 0。
rtapi_task::rtapi_task()
    : magic{}, id{}, owner{}, uses_fp{}, stacksize{}, prio{},
      period{}, nextstart{},
      ratio{}, pll_correction{}, pll_correction_limit{},
      arg{}, taskcode{},

      state(TASK_S_EMPTY),
      owner_pid(0),
      error_code(0)
{
}

// -----------------------------------------------------------------------------
// PosixTask — POSIX 线程实现的任务
// -----------------------------------------------------------------------------

namespace {

/// \brief 基于 POSIX pthread 的实时任务子类。
///
/// 在 rtapi_task 基础上增加 pthread 句柄以及暂停/恢复所需的两个信号量。
struct PosixTask : rtapi_task {
    PosixTask()
        : rtapi_task{}, thr{}, pause_sem{}, resume_sem{} {
        sem_init(&pause_sem, 0, 0);
        sem_init(&resume_sem, 0, 0);
    }

    ~PosixTask() {
        sem_destroy(&pause_sem);
        sem_destroy(&resume_sem);
    }

    pthread_t thr;       ///< 对应的 POSIX 线程句柄。
    sem_t pause_sem;     ///< task_pause() 写入；task 线程在 rtapi_wait() 中消费。
    sem_t resume_sem;    ///< task_resume() 写入；task 线程在恢复阻塞处消费。
};

// -----------------------------------------------------------------------------
// Posix — RtapiApp 的 POSIX/SCHED_FIFO 子类
// -----------------------------------------------------------------------------

/// \brief 基于 SCHED_FIFO 实时策略的 POSIX 调度器实现。
///
/// 实现 RtapiApp 中全部纯虚函数，提供 pthread 级别的任务生命周期管理。
struct Posix : RtapiApp {
    explicit Posix(int policy = SCHED_FIFO)
        : RtapiApp(policy), do_thread_lock(policy != SCHED_FIFO) {
        pthread_once(&key_once, init_key);
        if (do_thread_lock) {
            pthread_once(&lock_once, init_lock);
        }
    }

    // --- RtapiApp 纯虚函数实现 ---

    /// \brief 删除指定任务：取消线程、join、释放对象。
    /// \param id 任务 ID。
    /// \return 成功返回 0；任务不存在、无效或已删除返回负值。
    int task_delete(int id) override;
    /// \brief 创建 pthread 并将任务转入 RUNNING 状态。
    /// \param task_id     任务 ID。
    /// \param period_nsec 调度周期（纳秒）。
    /// \return 成功返回 0；状态非法或 pthread_create 失败返回负值。
    int task_start(int task_id, unsigned long period_nsec) override;
    /// \brief 暂停任务执行。
    /// \param task_id 任务 ID。
    /// \return 成功返回 0；任务未在 RUNNING 状态返回 -EINVAL。
    int task_pause(int task_id) override;
    /// \brief 恢复已暂停任务。
    /// \param task_id 任务 ID。
    /// \return 成功返回 0；任务未在 PAUSED 状态返回 -EINVAL。
    int task_resume(int task_id) override;
    /// \brief 返回当前调用线程对应的任务 ID。
    /// \return 任务 ID，不在任务上下文中返回 -EINVAL。
    int task_self() override;
    /// \brief 获取 PLL 参考时间戳。
    long long task_pll_get_reference(void) override;
    /// \brief 设置 PLL 校正值（限制在 period/100 范围内）。
    /// \param value 校正值。
    /// \return 成功返回 0；不在任务上下文中返回 -EINVAL。
    int task_pll_set_correction(long value) override;
    /// \brief 获取当前 PLL 校正值。
    /// \param value 输出参数，存放当前校正值。
    /// \return 成功返回 0；不在任务上下文中返回 -EINVAL。
    int task_pll_get_correction(long *value) override;
    /// \brief 等待下一个调度周期，由具体子类实现。
    void wait() override;
    /// \brief 分配并构造新的 PosixTask 对象。
    rtapi_task *do_task_new() override {
        return new PosixTask;
    }
    /// \brief 在指定 fd 上运行回调循环。
    /// \param fd       文件描述符。
    /// \param callback 每轮调用的回调函数。
    /// \return 成功返回 0。
    int run_threads(int fd, int (*callback)(int fd)) override;
    static void *wrapper(void *arg);

    /// 是否在任务入口函数外围加全局互斥锁（非 SCHED_FIFO 策略时启用）。
    bool do_thread_lock;

    /// 线程局部存储键，用于在 wrapper 中通过 pthread_getspecific 获取任务指针。
    static pthread_once_t key_once;
    static pthread_key_t key;
    static void init_key(void) {
        pthread_key_create(&key, NULL);
    }

    /// 非 SCHED_FIFO 策略下的全局互斥锁（延迟初始化）。
    static pthread_once_t lock_once;
    static pthread_mutex_t thread_lock;
    static void init_lock(void) {
        pthread_mutex_init(&thread_lock, NULL);
    }

    /// \brief 获取 CLOCK_MONOTONIC 纳秒时间戳。
    /// \return 距固定 Epoch 的累计纳秒数。
    long long do_get_time(void) override {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }

    /// \brief 纯延迟（忙等待或 nanosleep）。
    /// \param ns 要延迟的纳秒数。
    void do_delay(long ns) override;
};

/// \brief 工厂函数，创建 Posix 实例并记录日志。
static RtapiApp *makeApp() {
    rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX realtime\n");
    return new Posix(SCHED_FIFO);
}

/// \brief 返回全局唯一的 RtapiApp 实例（Meyers Singleton）。
RtapiApp &App() {
    static RtapiApp *app = makeApp();
    return *app;
}

}  // anonymous namespace

/// 全局任务指针数组，按任务 ID 索引。
struct rtapi_task *task_array[MAX_TASKS];

// -----------------------------------------------------------------------------
// RtapiApp — 优先级管理（模板方法实现）
// -----------------------------------------------------------------------------

/// \brief 返回当前调度策略的最高优先级（数值最小）。
int RtapiApp::prio_highest() const {
    return sched_get_priority_max(policy);
}

/// \brief 返回当前调度策略的最低优先级（数值最大）。
int RtapiApp::prio_lowest() const {
    return sched_get_priority_min(policy);
}

/// \brief 返回相邻优先级之间的增量。
int RtapiApp::prio_higher_delta() const {
    if (rtapi_prio_highest() > prio_lowest()) {
        return 1;
    }
    return -1;
}

/// \brief 将优先级限制在当前策略的合法范围内。
/// \param prio 原始优先级。
/// \return 在有效范围内的优先级值。
int RtapiApp::prio_bound(int prio) const {
    if (rtapi_prio_highest() > prio_lowest()) {
        if (prio >= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio < prio_lowest())
            return prio_lowest();
    } else {
        if (prio <= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio > prio_lowest())
            return prio_lowest();
    }
    return prio;
}

/// \brief 返回大于 prio 的下一个有效优先级。
/// \param prio 当前优先级。
/// \return 下一个更高的优先级，若无更高则返回 prio。
int RtapiApp::prio_next_higher(int prio) const {
    prio = prio_bound(prio);
    if (prio != rtapi_prio_highest())
        return prio + prio_higher_delta();
    return prio;
}

/// \brief 返回小于 prio 的下一个有效优先级。
/// \param prio 当前优先级。
/// \return 下一个更低的优先级，若无更低则返回 prio。
int RtapiApp::prio_next_lower(int prio) const {
    prio = prio_bound(prio);
    if (prio != prio_lowest())
        return prio - prio_higher_delta();
    return prio;
}

// -----------------------------------------------------------------------------
// 任务 ID 分配（基于 CAS）
// -----------------------------------------------------------------------------

/// \brief 原子地分配一个新的任务 ID，使用 CAS 保证多线程安全。
///
/// 遍历 task_array，尝试将空槽位（NULL）原子地替换为 TASK_MAGIC_INIT 占位符。
///
/// \return 分配到的任务 ID（>= 0），数组满时返回 -ENOSPC。
int RtapiApp::allocate_task_id() {
    for (int n = 0; n < MAX_TASKS; n++) {
        rtapi_task **taskptr = &(task_array[n]);
        if (__sync_bool_compare_and_swap(taskptr, (rtapi_task *)0, TASK_MAGIC_INIT))
            return n;
    }
    return -ENOSPC;
}

// -----------------------------------------------------------------------------
// RtapiApp::task_new — 分配并初始化实时任务
// -----------------------------------------------------------------------------

/// \brief 创建新任务：校验参数、分配槽位、构造任务对象、写入元数据。
///
/// 生命周期转换：EMPTY (CAS) -> INIT (CAS) -> ALLOCATED。
/// 调用方获得 task_id，可随后传入 task_start() 启动任务。
///
/// \param taskcode    任务入口函数。
/// \param arg         传递给 taskcode 的参数。
/// \param prio        任务优先级。
/// \param owner       创建者模块 ID。
/// \param stacksize   线程栈大小（字节），小于 1 MB 时自动设为 1 MB。
/// \param uses_fp     是否使用浮点单元。
/// \return 新任务的 ID（>= 0），参数非法或无空槽返回负值 errno。
int RtapiApp::task_new(void (*taskcode)(void *), void *arg,
                       int prio, int owner,
                       unsigned long int stacksize, int uses_fp) {
    if ((prio > rtapi_prio_highest()) || (prio < prio_lowest())) {
        return -EINVAL;
    }

    int n = allocate_task_id();
    if (n < 0) return n;

    struct rtapi_task *task = do_task_new();
    if (stacksize < (1024 * 1024)) stacksize = (1024 * 1024);
    task->id = n;
    task->owner = owner;
    task->uses_fp = uses_fp;
    task->arg = arg;
    task->stacksize = stacksize;
    task->taskcode = taskcode;
    task->prio = prio;
    task->magic = TASK_MAGIC;
    task->owner_pid = getpid();
    task->error_code = 0;
    task->state = TASK_S_ALLOCATED;
    task_array[n] = task;

    return n;
}

/// \brief 根据任务 ID 查找任务指针（含幻数校验）。
///
/// \param task_id 任务 ID。
/// \return 任务指针；无效 ID 或校验失败返回 nullptr。
rtapi_task *RtapiApp::get_task(int task_id) {
    if (task_id < 0 || task_id >= MAX_TASKS) return nullptr;
    rtapi_task *task = task_array[task_id];
    if (!task || task == TASK_MAGIC_INIT || task->magic != TASK_MAGIC)
        return nullptr;
    return task;
}

/// \brief 报告意外的实时延迟（每个会话仅打印一次）。
///
/// \param task    发生延迟的任务。
/// \param nperiod 连续延迟的周期数（当前未使用）。
void RtapiApp::unexpected_realtime_delay(rtapi_task *task, int /*nperiod*/) {
    static int printed = 0;
    if (!printed) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Unexpected realtime delay on main %d with period %ld\n"
                "This Message will only display once per session.\n"
                "Run the Latency Test and resolve before continuing.\n",
                task->id, task->period);
        printed = 1;
    }
}

// -----------------------------------------------------------------------------
// Posix::task_delete — 取消并清理 POSIX 任务
// -----------------------------------------------------------------------------

/// \brief 删除任务：若处于 PAUSED 状态则先唤醒，然后 cancel + join 线程。
///
/// \param id 任务 ID。
/// \return 成功返回 0；任务不存在、无效或已删除返回负值。
int Posix::task_delete(int id) {
    auto task = ::rtapi_get_task<PosixTask>(id);
    if (!task) return -EINVAL;

    if (task->state == TASK_S_DELETED)
        return -EALREADY;
    if (task->state == TASK_S_EMPTY || task->state == TASK_S_INIT)
        return -EINVAL;

    // 若任务处于 PAUSED，先唤醒使其能在下一次取消点响应 pthread_cancel。
    if (task->state == TASK_S_PAUSED) {
        sem_post(&task->resume_sem);
    }

    pthread_cancel(task->thr);
    pthread_join(task->thr, 0);

    task->magic = 0;
    task->state = TASK_S_DELETED;
    task_array[id] = 0;

    // PosixTask 析构函数会销毁信号量。
    delete task;

    return 0;
}

// -----------------------------------------------------------------------------
// 实时 CPU 亲和性检测
// -----------------------------------------------------------------------------

/// \brief 获取实时任务应绑定的 CPU 编号。
///
/// 按优先级依次尝试：RTAPI_CPU_NUMBER 环境变量 > isolcpus 配置 >
/// 进程当前 CPU 亲和性掩码中的最高编号。
///
/// \return 要绑定的 CPU 编号，无可用 CPU 返回 -1。
static int find_rt_cpu_number() {
    if (getenv("RTAPI_CPU_NUMBER")) return atoi(getenv("RTAPI_CPU_NUMBER"));

#ifdef __linux__
    cpu_set_t cpuset_orig;
    int r = sched_getaffinity(getpid(), sizeof(cpuset_orig), &cpuset_orig);
    if (r < 0)
        return 0;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    long top_probe = sysconf(_SC_NPROCESSORS_CONF);
    // 在旧版 glibc 中，向 sched_setaffinity 传入超出探测上限的 CPU 位
    // 会导致 EINVAL；因此仅设置到 "Configured Processors" 上限。
    for (long i = 0; i < top_probe && i < CPU_SETSIZE; i++) CPU_SET(i, &cpuset);

    r = sched_setaffinity(getpid(), sizeof(cpuset), &cpuset);
    if (r < 0)
        perror("sched_setaffinity");

    r = sched_getaffinity(getpid(), sizeof(cpuset), &cpuset);
    if (r < 0) {
        perror("sched_getaffinity");
        CPU_AND(&cpuset, &cpuset_orig, &cpuset);
    }

    int top = -1;
    for (int i = 0; i < CPU_SETSIZE; i++) {
        if (CPU_ISSET(i, &cpuset)) top = i;
    }
    return top;
#else
    return -1;
#endif
}

// -----------------------------------------------------------------------------
// Posix::task_start — 创建 pthread 并转入 RUNNING 状态
// -----------------------------------------------------------------------------

/// \brief 创建 pthread 并启动任务，进入 RUNNING 状态。
///
/// 状态门控：仅 ALLOCATED 状态可启动；重复启动返回 -EALREADY。
/// pthread_create 失败时状态回退至 ALLOCATED，以便 task_delete 正常清理。
///
/// \param task_id      任务 ID。
/// \param period_nsec  调度周期（纳秒）。
/// \return 成功返回 0；状态非法或 pthread_create 失败返回负值 errno。
int Posix::task_start(int task_id, unsigned long int period_nsec) {
    auto task = ::rtapi_get_task<PosixTask>(task_id);
    if (!task) return -EINVAL;

    if (task->state == TASK_S_RUNNING)
        return -EALREADY;
    if (task->state != TASK_S_ALLOCATED) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task_start: task %d in state %d, expected ALLOCATED (%d)\n",
            task_id, task->state, TASK_S_ALLOCATED);
        return -EINVAL;
    }

    task->period = period_nsec;

    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = task->prio;

    // 将 PLL 校正范围限制在周期时间的 ±1%。
    task->pll_correction_limit = period_nsec / 100;
    task->pll_correction = 0;

    int nprocs = sysconf(_SC_NPROCESSORS_ONLN);

    pthread_attr_t attr;
    int ret;
    if ((ret = pthread_attr_init(&attr)) != 0) {
        task->error_code = ret;
        return -ret;
    }
    if ((ret = pthread_attr_setstacksize(&attr, task->stacksize)) != 0) {
        task->error_code = ret;
        pthread_attr_destroy(&attr);
        return -ret;
    }
    if ((ret = pthread_attr_setschedpolicy(&attr, policy)) != 0) {
        task->error_code = ret;
        pthread_attr_destroy(&attr);
        return -ret;
    }
    if ((ret = pthread_attr_setschedparam(&attr, &param)) != 0) {
        task->error_code = ret;
        pthread_attr_destroy(&attr);
        return -ret;
    }
    if ((ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0) {
        task->error_code = ret;
        pthread_attr_destroy(&attr);
        return -ret;
    }
    if (nprocs > 1) {
        const static int rt_cpu_number = find_rt_cpu_number();
        if (rt_cpu_number != -1) {
#ifdef __FreeBSD__
            cpuset_t cpuset;
#else
            cpu_set_t cpuset;
#endif
            CPU_ZERO(&cpuset);
            CPU_SET(rt_cpu_number, &cpuset);
            if ((ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset)) != 0) {
                task->error_code = ret;
                pthread_attr_destroy(&attr);
                return -ret;
            }
        }
    }

    // 先转入 RUNNING 再创建线程，使 task_pause/task_delete 在启动期间可以安全引用。
    task->state = TASK_S_RUNNING;

    if ((ret = pthread_create(&task->thr, &attr, &wrapper,
                               reinterpret_cast<void *>(task))) != 0) {
        task->state = TASK_S_ALLOCATED;
        task->error_code = ret;
        pthread_attr_destroy(&attr);
        return -ret;
    }

    pthread_attr_destroy(&attr);
    return 0;
}

// -----------------------------------------------------------------------------
// 静态成员初始化
// -----------------------------------------------------------------------------

pthread_once_t Posix::key_once = PTHREAD_ONCE_INIT;
pthread_once_t Posix::lock_once = PTHREAD_ONCE_INIT;
pthread_key_t Posix::key;
pthread_mutex_t Posix::thread_lock;

// -----------------------------------------------------------------------------
// Posix::wrapper — 线程入口函数
// -----------------------------------------------------------------------------

/// \brief pthread 入口函数，设置线程名、亲和性、调度参数后调用 taskcode。
///
/// \param arg 指向 rtapi_task 的指针。
void *Posix::wrapper(void *arg) {
    struct rtapi_task *task;
    task = (struct rtapi_task *)arg;
    rtapi_print_msg(RTAPI_MSG_INFO, "main %p period = %lu ratio=%u\n",
            task, task->period, task->ratio);

    pthread_setspecific(key, arg);
    set_namef("rtapi_app:T#%d", task->id);

    Posix &papp = reinterpret_cast<Posix &>(App());
    if (papp.do_thread_lock)
        pthread_mutex_lock(&papp.thread_lock);

    struct timespec now;
    clock_gettime(RTAPI_CLOCK, &now);
    rtapi_timespec_advance(task->nextstart, now,
                           task->period + task->pll_correction);

    (task->taskcode)(task->arg);

    rtapi_print_msg(RTAPI_MSG_ERR,
                    "ERROR: reached end of wrapper for main %d\n", task->id);
    return nullptr;
}

// -----------------------------------------------------------------------------
// Posix — PLL 接口实现
// -----------------------------------------------------------------------------

/// \brief 获取当前任务的 PLL 参考时间戳（nextstart 的累计纳秒值）。
long long Posix::task_pll_get_reference(void) {
    struct rtapi_task *task =
        reinterpret_cast<rtapi_task *>(pthread_getspecific(key));
    if (!task) return 0;
    return task->nextstart.tv_sec * 1000000000LL + task->nextstart.tv_nsec;
}

/// \brief 设置 PLL 校正值，自动裁剪到允许范围。
/// \param value 要设置的校正值。
/// \return 成功返回 0；不在任务上下文中返回 -EINVAL。
int Posix::task_pll_set_correction(long value) {
    struct rtapi_task *task =
        reinterpret_cast<rtapi_task *>(pthread_getspecific(key));
    if (!task) return -EINVAL;
    if (value > task->pll_correction_limit)
        value = task->pll_correction_limit;
    if (value < -(task->pll_correction_limit))
        value = -(task->pll_correction_limit);
    task->pll_correction = value;
    return 0;
}

/// \brief 获取当前 PLL 校正值。
/// \param value 输出参数，存放当前校正值。
/// \return 成功返回 0；不在任务上下文中返回 -EINVAL。
int Posix::task_pll_get_correction(long *value) {
    struct rtapi_task *task =
        reinterpret_cast<rtapi_task *>(pthread_getspecific(key));
    if (!task) return -EINVAL;
    *value = task->pll_correction;
    return 0;
}

// -----------------------------------------------------------------------------
// Posix — 暂停/恢复
// -----------------------------------------------------------------------------

/// \brief 暂停任务执行。
///
/// 向 pause_sem 写入信号，任务在 rtapi_wait() 中检测到后阻塞在 resume_sem 上。
///
/// \param task_id 任务 ID。
/// \return 成功返回 0；任务不存在或未处于 RUNNING 状态返回 -EINVAL。
int Posix::task_pause(int task_id) {
    auto task = ::rtapi_get_task<PosixTask>(task_id);
    if (!task) return -EINVAL;
    if (task->state != TASK_S_RUNNING) return -EINVAL;

    task->state = TASK_S_PAUSED;
    sem_post(&task->pause_sem);
    return 0;
}

/// \brief 恢复已暂停任务。
///
/// 向 resume_sem 写入信号，唤醒任务线程并将其状态重置为 RUNNING。
///
/// \param task_id 任务 ID。
/// \return 成功返回 0；任务不存在或未处于 PAUSED 状态返回 -EINVAL。
int Posix::task_resume(int task_id) {
    auto task = ::rtapi_get_task<PosixTask>(task_id);
    if (!task) return -EINVAL;
    if (task->state != TASK_S_PAUSED) return -EINVAL;

    sem_post(&task->resume_sem);
    return 0;
}

// -----------------------------------------------------------------------------
// Posix::task_self — 返回当前任务 ID
// -----------------------------------------------------------------------------

/// \brief 返回调用线程对应的任务 ID。
/// \return 任务 ID；调用线程不在任务上下文中返回 -EINVAL。
int Posix::task_self() {
    struct rtapi_task *task =
        reinterpret_cast<rtapi_task *>(pthread_getspecific(key));
    if (!task) return -EINVAL;
    return task->id;
}

// -----------------------------------------------------------------------------
// Posix::wait — 等待下一个调度周期
// -----------------------------------------------------------------------------

/// \brief 任务主循环的调度等待点。
///
/// 检查外部线程发来的暂停请求；若已过下一个启动时间则报告延迟，
/// 否则使用 clock_nanosleep 精确等待。在非 SCHED_FIFO 模式下，
/// 此处管理全局互斥锁的解锁/重新加锁。
void Posix::wait() {
    if (do_thread_lock)
        pthread_mutex_unlock(&thread_lock);
    pthread_testcancel();

    struct rtapi_task *task =
        reinterpret_cast<rtapi_task *>(pthread_getspecific(key));

    // 检查外部暂停请求：sem_trywait 为非阻塞，0 表示收到暂停信号。
    PosixTask *ptask = static_cast<PosixTask *>(task);
    if (sem_trywait(&ptask->pause_sem) == 0) {
        sem_wait(&ptask->resume_sem);
        ptask->state = TASK_S_RUNNING;
    }

    rtapi_timespec_advance(task->nextstart, task->nextstart,
                           task->period + task->pll_correction);
    struct timespec now;
    clock_gettime(RTAPI_CLOCK, &now);
    if (rtapi_timespec_less(task->nextstart, now)) {
        if (policy == SCHED_FIFO)
            unexpected_realtime_delay(task);
    } else {
        int res = rtapi_clock_nanosleep(RTAPI_CLOCK, TIMER_ABSTIME,
                                        &task->nextstart, nullptr, &now);
        if (res < 0) perror("clock_nanosleep");
    }
    if (do_thread_lock)
        pthread_mutex_lock(&thread_lock);
}

/// \brief 纯延迟指定纳秒。
/// \param ns 要延迟的纳秒数。
void Posix::do_delay(long ns) {
    struct timespec ts = {0, ns};
    rtapi_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr, nullptr);
}

// -----------------------------------------------------------------------------
// C 包装函数 — 优先级
// -----------------------------------------------------------------------------

/// \brief C 包装：返回最高优先级。
int rtapi_prio_highest(void) {
    return App().prio_highest();
}

/// \brief C 包装：返回最低优先级。
int rtapi_prio_lowest(void) {
    return App().prio_lowest();
}

/// \brief C 包装：返回大于 prio 的下一个优先级。
int rtapi_prio_next_higher(int prio) {
    return App().prio_next_higher(prio);
}

/// \brief C 包装：返回小于 prio 的下一个优先级。
int rtapi_prio_next_lower(int prio) {
    return App().prio_next_lower(prio);
}

// -----------------------------------------------------------------------------
// 时钟周期
// -----------------------------------------------------------------------------

/// \brief C 包装：设置全局时钟周期。
/// \param nsecs 周期长度（纳秒）。
/// \return 实际设置的周期值，重复设置返回 -EINVAL。
long rtapi_clock_set_period(long nsecs) {
    return App().clock_set_period(nsecs);
}

/// \brief RtapiApp 层次：禁止重复设置周期。
/// \param nsecs 周期长度（纳秒）。
/// \return 实际设置的周期值。
long RtapiApp::clock_set_period(long nsecs) {
    if (nsecs == 0) return period;
    if (period != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "attempt to set period twice\n");
        return -EINVAL;
    }
    period = nsecs;
    return period;
}

// -----------------------------------------------------------------------------
// C 包装函数 — 任务管理
// -----------------------------------------------------------------------------

/// \brief C 包装：创建新任务。
int rtapi_task_new(void (*taskcode)(void *), void *arg,
                   int prio, int owner,
                   unsigned long int stacksize, int uses_fp) {
    return App().task_new(taskcode, arg, prio, owner, stacksize, uses_fp);
}

/// \brief C 包装：删除任务。
int rtapi_task_delete(int id) {
    return App().task_delete(id);
}

/// \brief C 包装：启动任务。
int rtapi_task_start(int task_id, unsigned long period_nsec) {
    int ret = App().task_start(task_id, period_nsec);
    if (ret != 0) {
        errno = -ret;
        perror("rtapi_task_start()");
    }
    return ret;
}

/// \brief C 包装：暂停任务。
int rtapi_task_pause(int task_id) {
    return App().task_pause(task_id);
}

/// \brief C 包装：恢复任务。
int rtapi_task_resume(int task_id) {
    return App().task_resume(task_id);
}

/// \brief C 包装：返回当前任务 ID。
int rtapi_task_self() {
    return App().task_self();
}

/// \brief C 包装：获取任务状态。
/// \param task_id 任务 ID。
/// \return 任务状态（ TASK_S_* 枚举值），无效返回 TASK_S_ERROR。
int rtapi_task_getstate(int task_id) {
    if (task_id < 0 || task_id >= MAX_TASKS)
        return TASK_S_ERROR;
    rtapi_task *task = task_array[task_id];
    if (!task || task->magic != TASK_MAGIC)
        return TASK_S_ERROR;
    return task->state;
}

// -----------------------------------------------------------------------------
// C 包装函数 — PLL
// -----------------------------------------------------------------------------

/// \brief C 包装：获取 PLL 参考时间戳。
long long rtapi_task_pll_get_reference(void) {
    return App().task_pll_get_reference();
}

/// \brief C 包装：设置 PLL 校正值。
/// \param value 校正值。
/// \return 成功返回 0。
int rtapi_task_pll_set_correction(long value) {
    return App().task_pll_set_correction(value);
}

/// \brief C 包装：获取 PLL 校正值。
/// \param value 输出参数，存放校正值。
/// \return 成功返回 0。
int rtapi_task_pll_get_correction(long *value) {
    return App().task_pll_get_correction(value);
}

// -----------------------------------------------------------------------------
// C 包装函数 — 调度与时间
// -----------------------------------------------------------------------------

/// \brief C 包装：等待下一个调度周期。
void rtapi_wait(void) {
    App().wait();
}

/// \brief 在指定 fd 上运行回调循环。
/// \param fd       文件描述符。
/// \param callback 每轮调用的回调函数。
/// \return 成功返回 0。
int Posix::run_threads(int fd, int (*callback)(int fd)) {
    while (callback(fd)) { /* nothing */ }
    return 0;
}

/// \brief C 包装：运行线程循环（仿真版本）。
int sim_rtapi_run_threads(int fd, int (*callback)(int fd)) {
    return App().run_threads(fd, callback);
}

/// \brief C 包装：获取纳秒时间戳。
long long rtapi_get_time() {
    return App().do_get_time();
}

/// 最大允许的 rtapi_delay 时长（纳秒）。
long int rtapi_delay_max() { return 10000; }

/// \brief 延迟指定纳秒，超长时自动截断到 rtapi_delay_max。
/// \param ns 要延迟的纳秒数。
void rtapi_delay(long ns) {
    if (ns > rtapi_delay_max()) ns = rtapi_delay_max();
    App().do_delay(ns);
}

// -----------------------------------------------------------------------------
// timespec 辅助
// -----------------------------------------------------------------------------

/// 每秒对应的纳秒数常量。
const unsigned long ONE_SEC_IN_NS = 1000000000;

/// \brief 在 src 基础上推进 nsec 纳秒，结果写入 result。
/// \param result 存放结果的 timespec。
/// \param src    起始时间。
/// \param nsec   要推进的纳秒数。
void rtapi_timespec_advance(struct timespec &result,
                             const struct timespec &src,
                             unsigned long nsec) {
    time_t sec = src.tv_sec;
    while (nsec >= ONE_SEC_IN_NS) {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    nsec += src.tv_nsec;
    if (nsec >= ONE_SEC_IN_NS) {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    result.tv_sec = sec;
    result.tv_nsec = nsec;
}

// -----------------------------------------------------------------------------
// 消息队列与 printf 控制
// -----------------------------------------------------------------------------

/// 消息消费者线程句柄。
pthread_t queue_thread;

/// \brief 消息队列消费者线程函数，持续从无锁队列中取出消息并输出。
/// \param /*arg*/ 未使用参数。
void *queue_function(void * /*arg*/) {
    while (1) {
        pthread_testcancel();
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        rtapi_msg_queue.consume_all([](const message_t &m) {
            FILE *out = (m.level == RTAPI_MSG_ALL) ? stdout : stderr;
            fputs(m.msg, out);
        });
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    }
    return nullptr;
}

/// \brief 启动消息队列消费者线程，启用主进程的 printf 接管。
/// \return 成功返回 0，失败返回 -1。
int printfMaster() {
    int result;
    if ((result = pthread_create(&queue_thread, NULL,
                                  &queue_function, NULL)) != 0) {
        errno = result;
        printf("pthread_create (queue function) failed\n");
        return -1;
    }
    return result;
}

/// \brief 停止消息队列消费者线程，清空队列中残留消息。
int stopprintf() {
    if (queue_thread != NULL) {
        pthread_cancel(queue_thread);
        pthread_join(queue_thread, NULL);
        rtapi_msg_queue.consume_all([](const message_t &m) {
            fputs(m.msg, m.level == RTAPI_MSG_ALL ? stdout : stderr);
        });
    }
    return 0;
}

// -----------------------------------------------------------------------------
// 默认消息处理器
// -----------------------------------------------------------------------------

/// \brief rtapi 版本默认消息处理函数：将消息入队至无锁队列。
/// \param level 消息级别。
/// \param fmt   格式字符串。
/// \param ap    可变参数列表。
void default_rtapi_msg_handler(msg_level_t level,
                               const char *fmt, va_list ap) {
    message_t m;
    m.level = level;
    vsnprintf(m.msg, sizeof(m.msg), fmt, ap);
    rtapi_msg_queue.push(m);
}

/// \brief 内部版本默认消息处理函数：直接输出至 stdout/stderr。
/// \param level 消息级别。
/// \param fmt   格式字符串。
/// \param ap    可变参数列表。
void default_msg_handler(msg_level_t level,
                         const char *fmt, va_list ap) {
    message_t m;
    m.level = level;
    vsnprintf(m.msg, sizeof(m.msg), fmt, ap);
    FILE *out = (m.level == RTAPI_MSG_ALL) ? stdout : stderr;
    fputs(m.msg, out);
}

// -----------------------------------------------------------------------------
// clock_nanosleep 跨平台封装
// -----------------------------------------------------------------------------

/// \brief 基于 clock_nanosleep 的跨平台睡眠函数。
///
/// \param clock_id  使用的时钟 ID。
/// \param flags     TIMER_ABSTIME 表示 prequest 为绝对时间。
/// \param prequest  睡眠时长或绝对时间。
/// \param remain    若被信号中断，存放剩余时间（可为 nullptr）。
/// \param pnow      若非空，存放当前实际时间。
/// \return 成功返回 0，被信号中断返回 -1 并设置 errno。
int rtapi_clock_nanosleep(clockid_t clock_id, int flags,
                          const struct timespec *prequest,
                          struct timespec *remain,
                          const struct timespec *pnow) {
    (void)pnow;
#if defined(HAVE_CLOCK_NANOSLEEP)
    return clock_nanosleep(clock_id, flags, prequest, remain);
#else
    if (flags == 0)
        return nanosleep(prequest, remain);
    if (flags != TIMER_ABSTIME) {
        errno = EINVAL;
        return -1;
    }
    struct timespec now;
    if (!pnow) {
        int res = clock_gettime(clock_id, &now);
        if (res < 0) return res;
        pnow = &now;
    }
#undef timespecsub
#define timespecsub(tvp, uvp, vvp)                                         \
    do {                                                                   \
        (vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec;                    \
        (vvp)->tv_nsec = (tvp)->tv_nsec - (uvp)->tv_nsec;                 \
        if ((vvp)->tv_nsec < 0) {                                         \
            (vvp)->tv_sec--;                                              \
            (vvp)->tv_nsec += 1000000000;                                 \
        }                                                                 \
    } while (0)
    struct timespec request;
    timespecsub(prequest, pnow, &request);
    return nanosleep(&request, remain);
#endif
}
