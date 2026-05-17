/// \file hal_priv.h
///
/// HAL 私有内部结构与模板声明。
///
/// 定义 HAL 共享内存布局（hal_data_t）、各对象结构体（组件/线程/引脚/参数/函数）、
/// 链表工具、模板化的共享内存指针/偏移量转换工具，以及内部分配/查找函数。
/// 仅供 HAL 内部实现使用，不对用户代码暴露。

#ifndef HAL_PRIV_H
#define HAL_PRIV_H

#include <rtapi/rtapi_mutex.h>
#include <hal/hal.h>
#include <unistd.h>

// -----------------------------------------------------------------------------
// 共享内存常量
// -----------------------------------------------------------------------------

/// HAL 对象名称的最大字符数（含终止符），与 hal.h 保持一致。
#define HAL_NAME_LEN     47
/// HAL 线程的默认栈大小（字节）。
#define HAL_STACKSIZE 16384
/// 用于打开 HAL 共享内存的 System V IPC key（"HAL2" 的 ASCII 码值）。
#define HAL_KEY   0x48414C32
/// HAL 共享内存的版本号，用于版本兼容性校验。
#define HAL_VER   0x00000010
/// HAL 共享内存的总大小（256 页，每页 4096 字节，共 1 MB）。
#define HAL_SIZE  (256 * 4096)

/// 伪组件名称前缀，用于内部占位组件（不以用户可见方式注册）。
#define HAL_PSEUDO_COMP_PREFIX "__"

/// HAL 共享内存在当前进程中的基地址。
extern char *hal_shmem_base;
/// 指向 HAL 共享内存数据区起始位置的指针。
extern struct hal_data_t *hal_data;

// -----------------------------------------------------------------------------
// 共享内存指针/偏移量转换（C++ 模板版本）
//
// C++ 环境下使用模板类封装偏移量，支持类型安全的指针恢复。
// 指针在共享内存中以相对偏移量存储（省去跨进程指针的序列化问题）。
// -----------------------------------------------------------------------------

#ifdef __cplusplus

/// \brief 检查给定指针是否位于 HAL 共享内存范围内。
/// \param t 任意类型指针。
/// \return 指针落在共享内存区间内返回 true，否则返回 false。
template<class T>
bool hal_shmchk(T *t) {
    char *c = (char *)t;
    return c > hal_shmem_base && c < hal_shmem_base + HAL_SIZE;
}

/// \brief 将指针转换为相对 HAL 共享内存基址的字节偏移量。
/// \param t 任意类型指针，nullptr 返回 0。
/// \return 字节偏移量（从 hal_shmem_base 算起）。
template<class T>
int hal_shmoff(T *t) {
    return t ? (char *)t - hal_shmem_base : 0;
}

/// \brief 将字节偏移量转换回原始类型的指针。
/// \param p 字节偏移量，0 返回 nullptr。
/// \return 恢复后的类型指针。
template<class T>
T *hal_shmptr(int p) {
    return p ? (T *)(hal_shmem_base + p) : nullptr;
}

/// \brief 将绝对指针包装为 HAL 共享内存偏移量的类型安全封装。
///
/// hal_shmfield<T> 在内部仅存储一个 rtapi_intptr_t 偏移量，
/// 通过 get()/operator*()/operator->() 透明地恢复为原始指针。
template<class T>
class hal_shmfield {
public:
    /// 默认构造，偏移量初始化为 0。
    hal_shmfield() : off{} {}
    /// 从指针构造，自动转换为偏移量。
    hal_shmfield(T *t) : off{hal_shmoff(t)} {}
    /// 赋值操作符，接受原始指针。
    hal_shmfield &operator=(T *t) {
        off = hal_shmoff(t);
        return *this;
    }
    /// 获取存储的实际指针。
    T *get() { return hal_shmptr<T>(off); }
    const T *get() const { return hal_shmptr<T>(off); }
    /// 解引用操作符。
    T *operator *() { return get(); }
    const T *operator *() const { return get(); }
    /// 成员访问操作符。
    T *operator ->() { return get(); }
    /// 布尔转换，偏移量非零时为 true。
    operator bool() const { return off; }
private:
    rtapi_intptr_t off;   ///< 相对于 hal_shmem_base 的字节偏移量。
};

/// \brief 工厂函数，从原始指针创建 hal_shmfield。
template<class T>
hal_shmfield<T> hal_make_shmfield(T *t) {
    return hal_shmfield<T>(t);
}

/// 编译期断言：确保 hal_shmfield<void> 的大小与 rtapi_intptr_t 相同，
/// 保证偏移量可以无歧义地存储和恢复。
static_assert(sizeof(hal_shmfield<void>) == sizeof(rtapi_intptr_t),
              "hal_shmfield size matches");

/// C++ 环境下 SHMFIELD(type) 等价于 hal_shmfield<type>。
#define SHMFIELD(type) hal_shmfield<type>
/// 从 hal_shmfield 对象获取实际指针。
#define SHMPTR(arg) ((arg).get())
/// 从原始指针创建 hal_shmfield 对象（自动转换偏移量）。
#define SHMOFF(ptr) (hal_make_shmfield(ptr))

// -----------------------------------------------------------------------------
// 共享内存指针/偏移量转换（C 宏版本）
//
// C 环境下偏移量直接存储为整数，通过宏在编译时完成指针/偏移量转换。
// -----------------------------------------------------------------------------

#else
#define SHMFIELD(type) rtapi_intptr_t

/// 将共享内存偏移量转换为实际指针。
#define SHMPTR(offset)  ((void *)(hal_shmem_base + (offset)))

/// 将指针转换为相对于共享内存基址的字节偏移量。
#define SHMOFF(ptr)     (((char *)(ptr)) - hal_shmem_base)

/// 检查指针是否位于 HAL 共享内存范围内。
#define SHMCHK(ptr)  (((char *)(ptr)) > (hal_shmem_base) && \
                     ((char *)(ptr)) < (hal_shmem_base + HAL_SIZE))
#endif

// -----------------------------------------------------------------------------
// 链表节点
// -----------------------------------------------------------------------------

/// \brief 双向链表节点结构，用于串联 HAL 内部各类对象。
typedef struct hal_list_t {
    SHMFIELD(hal_list_t) next;   ///< 链表中下一个节点。
    SHMFIELD(hal_list_t) prev;   ///< 链表中上一个节点。
} hal_list_t;

/// \brief 旧名称别名记录结构，用于在重命名引脚/参数时保留原名。
typedef struct hal_oldname_t {
    SHMFIELD(hal_oldname_t) next_ptr;   ///< 仅在空闲链表（free list）中使用。
    char name[HAL_NAME_LEN + 1];       ///< 被替换的原名称。
} hal_oldname_t;

// -----------------------------------------------------------------------------
// 前向声明
// -----------------------------------------------------------------------------

typedef struct hal_comp_t hal_comp_t;       ///< HAL 组件结构体。
typedef struct hal_thread_t hal_thread_t;  ///< HAL 线程结构体。
typedef struct hal_param_t hal_param_t;     ///< HAL 参数结构体。
typedef struct hal_funct_t hal_funct_t;    ///< HAL 函数结构体。
typedef struct hal_funct_entry_t hal_funct_entry_t;  ///< HAL 函数条目结构体。
typedef struct hal_pin_t hal_pin_t;         ///< HAL 引脚结构体。

// -----------------------------------------------------------------------------
// 组件类型枚举
// -----------------------------------------------------------------------------

/// \brief HAL 组件的类型枚举。
typedef enum {
    COMPONENT_TYPE_UNKNOWN = -1,   ///< 类型未知。
    COMPONENT_TYPE_USER,           ///< 用户空间组件。
    COMPONENT_TYPE_REALTIME,       ///< 实时内核组件。
    COMPONENT_TYPE_OTHER           ///< 其他类型。
} component_type_t;

// -----------------------------------------------------------------------------
// HAL 生命周期状态枚举
// -----------------------------------------------------------------------------

/// \brief HAL 系统整体生命周期状态枚举。
typedef enum {
    HAL_S_UNINIT        = 0,   ///< 共享内存尚未分配。
    HAL_S_INITIALIZING  = 1,   ///< shmget 成功，init_hal_data 正在执行。
    HAL_S_ACTIVE        = 2,   ///< 完全初始化，可创建线程。
    HAL_S_RUNNING       = 3,   ///< 实时线程正在执行。
    HAL_S_SHUTTING_DOWN = 4,   ///< hal_app_exit() 正在释放资源。
    HAL_S_DESTROYED     = 5,   ///< 共享内存已释放，不可再使用。
    HAL_S_ERROR         = -1   ///< 初始化失败，不可恢复。
} hal_state_t;

// -----------------------------------------------------------------------------
// hal_data_t — HAL 共享内存数据区根结构
// -----------------------------------------------------------------------------

/// \brief HAL 共享内存的根数据结构，包含所有 HAL 全局状态和自由链表。
///
/// 此结构体位于 HAL 共享内存的起始处，包含：
/// - 互斥锁（保护所有链表操作）
/// - 共享内存池管理（top/bot 指针）
/// - 所有对象类型的链表头和空闲链表头
/// - 生命周期状态、初始化 PID、最近错误码
typedef struct hal_data_t {
    rtapi_mutex_t mutex;        ///< 保护所有链表等共享数据结构的互斥锁。
    hal_s32_t shmem_avail;     ///< 当前 HAL 共享内存中剩余的可用字节数。

    /// 待调度的组件构造函数指针（用于延迟构造）。
    constructor pending_constructor;
    /// 新实例的名称前缀。
    char constructor_prefix[HAL_NAME_LEN + 1];
    /// 构造参数。
    char constructor_arg[HAL_NAME_LEN + 1];

    /// 空闲共享内存区域的起始偏移量（第一个空闲字节相对于基址的位移）。
    int shmem_bot;
    /// 空闲共享内存区域的结束偏移量（最后一个空闲字节之后的位移）。
    int shmem_top;

    /// 组件链表的头节点（SHMFIELD 偏移量）。
    SHMFIELD(hal_comp_t) comp_list_ptr;
    /// 引脚链表的头节点。
    SHMFIELD(hal_pin_t) pin_list_ptr;
    /// 参数链表的头节点。
    SHMFIELD(hal_param_t) param_list_ptr;
    /// 函数链表的头节点。
    SHMFIELD(hal_funct_t) funct_list_ptr;
    /// 线程链表的头节点。
    SHMFIELD(hal_thread_t) thread_list_ptr;

    long base_period;           ///< 实时任务的定时器周期（纳秒）。
    int threads_running;        ///< 非零表示线程已启动。

    /// 空闲 oldname 结构体链表的头节点。
    SHMFIELD(hal_oldname_t) oldname_free_ptr;
    /// 空闲组件结构体链表的头节点。
    SHMFIELD(hal_comp_t) comp_free_ptr;
    /// 空闲引脚结构体链表的头节点。
    SHMFIELD(hal_pin_t) pin_free_ptr;
    /// 空闲参数结构体链表的头节点。
    SHMFIELD(hal_param_t) param_free_ptr;
    /// 空闲函数结构体链表的头节点。
    SHMFIELD(hal_funct_t) funct_free_ptr;
    hal_list_t funct_entry_free;   ///< 空闲函数条目结构体链表。
    /// 空闲线程结构体链表的头节点。
    SHMFIELD(hal_thread_t) thread_free_ptr;

    /// 若为真，表示 HAL 层认为 rtapi 已精确满足用户请求的周期时间。
    int exact_base_period;
    /// 当前 HAL 锁级别（HAL_LOCK_* 的按位 OR 组合）。
    unsigned char lock;

    /// 当前 HAL 生命周期状态（HAL_S_* 枚举值），
    /// 由 init_hal_data() 初始化，由 hal_start_threads / hal_stop_threads / hal_app_exit 更新。
    int state;
    /// 执行首次 hal_init() 并创建共享内存的进程 PID，
    /// 在首次 hal_init() 完成前为 0。
    pid_t init_pid;
    /// HAL 层最近一次 OS 操作失败时捕获的 errno，正常时为 0。
    int error_code;
} hal_data_t;

// -----------------------------------------------------------------------------
// hal_stats_t — HAL 统计快照
// -----------------------------------------------------------------------------

/// \brief HAL 统计信息的只读快照结构体。
///
/// 调用方负责分配，hal_get_stats() 在持有 hal_data->mutex 的情况下填充各字段。
typedef struct {
    hal_state_t state;         ///< 当前 HAL 生命周期状态。
    int comp_count;            ///< 已注册的组件数量。
    int pin_count;             ///< 已注册的引脚数量。
    int param_count;           ///< 已注册的参数数量。
    int funct_count;           ///< 已导出的函数数量。
    int thread_count;          ///< 已创建的线程数量。
    int threads_running;       ///< 1 表示线程已启动，0 表示未启动。
    long shmem_avail;          ///< HAL 共享内存中剩余的空闲字节数。
    long shmem_total;          ///< HAL 共享内存总大小（等于 HAL_SIZE）。
    pid_t init_pid;            ///< 执行 HAL 层初始化的进程 PID。
    int error_code;            ///< HAL 层最近捕获的错误码。
} hal_stats_t;

// -----------------------------------------------------------------------------
// hal_comp_t — HAL 组件结构体
// -----------------------------------------------------------------------------

/// \brief HAL 组件的描述结构体，每个注册的组件对应一个实例。
struct hal_comp_t {
    SHMFIELD(hal_comp_t) next_ptr;    ///< 链表中下一个组件。
    int comp_id;                      ///< 组件 ID（RTAPI 模块 ID）。
    int mem_id;                       ///< 该组件使用的 RTAPI 共享内存 ID。
    component_type_t type;            ///< 组件类型。
    int ready;                        ///< 非零表示已就绪，零表示未就绪。
    int pid;                          ///< 组件所在进程 PID（仅对用户组件有意义）。
    void *shmem_base;                ///< 该组件专用共享内存基址。
    char name[HAL_NAME_LEN + 1];     ///< 组件名称。
    constructor make;                 ///< 组件构造函数（用于延迟调用）。
    SHMFIELD(char) insmod_args;      ///< 传递给 insmod 的参数字符串（偏移量）。
};

// -----------------------------------------------------------------------------
// hal_thread_t — HAL 线程结构体
// -----------------------------------------------------------------------------

/// \brief HAL 实时线程的描述结构体，对应一个 POSIX 调度线程。
struct hal_thread_t {
    SHMFIELD(hal_thread_t) next_ptr;   ///< 链表中下一个线程。
    int uses_fp;                        ///< 非零表示使用浮点单元。
    long int period;                    ///< 线程调度周期（纳秒）。
    int priority;                      ///< 线程优先级。
    int task_id;                       ///< 运行此线程的 RTAPI 任务 ID。
    hal_s32_t *runtime;                ///< 最近一次执行的耗时（CPU 时钟周期）。
    hal_s32_t maxtime;                 ///< 单次执行的最长耗时（CPU 时钟周期）。
    hal_list_t funct_list;             ///< 此线程每周期要调度的函数链表。
    char name[HAL_NAME_LEN + 1];       ///< 线程名称。
    int comp_id;                       ///< 创建该线程的组件 ID。
};

// -----------------------------------------------------------------------------
// hal_pin_t — HAL 引脚结构体
// -----------------------------------------------------------------------------

/// \brief HAL 引脚的描述结构体，表示组件的输入/输出数据端口。
struct hal_pin_t {
    SHMFIELD(hal_pin_t) next_ptr;     ///< 链表中下一个引脚。
    SHMFIELD(void *) data_ptr_addr;   ///< 指向引脚数据指针本身的地址（用于间接绑定）。
    SHMFIELD(hal_comp_t) owner_ptr;   ///< 拥有此引脚的组件。
    SHMFIELD(hal_sig_t) signal;       ///< 此引脚所连接的信号（偏移量）。
    hal_data_u dummysig;              ///< 未连接信号时的哑数据（data_ptr 指向此处）。
    SHMFIELD(hal_oldname_t) oldname;  ///< 重命名前的旧名称（未重命名时为零）。
    hal_type_t type;                  ///< 数据类型。
    char name[HAL_NAME_LEN + 1];      ///< 引脚名称。
};

// -----------------------------------------------------------------------------
// hal_param_t — HAL 参数结构体
// -----------------------------------------------------------------------------

/// \brief HAL 参数的描述结构体，表示组件可由外部配置的运行时变量。
struct hal_param_t {
    SHMFIELD(hal_param_t) next_ptr;     ///< 链表中下一个参数。
    SHMFIELD(void *) data_ptr;           ///< 参数值在共享内存中的偏移量。
    SHMFIELD(hal_comp_t) owner_ptr;     ///< 拥有此参数的组件。
    SHMFIELD(hal_oldname_t) oldname;     ///< 重命名前的旧名称。
    hal_type_t type;                    ///< 参数类型。
    hal_param_dir_t dir;                ///< 读写方向。
    char name[HAL_NAME_LEN + 1];       ///< 参数名称。
};

// -----------------------------------------------------------------------------
// hal_funct_t — HAL 函数结构体
// -----------------------------------------------------------------------------

/// \brief HAL 导出函数的描述结构体，表示可被线程调度的可调用函数单元。
struct hal_funct_t {
    SHMFIELD(hal_funct_t) next_ptr;     ///< 链表中下一个函数。
    int uses_fp;                        ///< 非零表示使用浮点单元。
    SHMFIELD(hal_comp_t) owner_ptr;    ///< 导出此函数的组件。
    int reentrant;                      ///< 非零表示函数可重入调用。
    int users;                          ///< 当前有多少个线程使用此函数。
    void *arg;                          ///< 传递给函数的用户参数。
    void (*funct)(void *, long);       ///< 函数入口指针。
    hal_s32_t *runtime;                ///< 最近一次调用的耗时（CPU 时钟周期）。
    hal_s32_t maxtime;                 ///< 单次调用的最长耗时。
    hal_bit_t maxtime_increased;       ///< 上次调用 maxtime 是否被更新。
    char name[HAL_NAME_LEN + 1];       ///< 函数名称。
};

// -----------------------------------------------------------------------------
// hal_funct_entry_t — HAL 函数条目结构体
// -----------------------------------------------------------------------------

/// \brief 函数在线程链表中的条目结构体，用于将同一函数链接到不同线程。
struct hal_funct_entry_t {
    hal_list_t links;                       ///< 链表节点（next/prev）。
    void *arg;                              ///< 传递给函数的用户参数。
    void (*funct)(void *, long);           ///< 函数入口指针。
    SHMFIELD(hal_funct_t) funct_ptr;       ///< 指向所关联 hal_funct_t 的偏移量。
};

// -----------------------------------------------------------------------------
// 链表操作
// -----------------------------------------------------------------------------

/// \brief 初始化一个链表节点（将 next 和 prev 均指向自身）。
/// \param entry 要初始化的节点指针。
void list_init_entry(hal_list_t *entry);

/// \brief 获取链表中给定节点的前一个节点。
/// \param entry 当前节点。
/// \return 前一个节点的指针，到达链表尾部时为 nullptr。
hal_list_t *list_prev(hal_list_t *entry);

/// \brief 获取链表中给定节点的下一个节点。
/// \param entry 当前节点。
/// \return 下一个节点的指针，到达链表尾部时为 nullptr。
hal_list_t *list_next(hal_list_t *entry);

/// \brief 在指定节点之后插入新节点。
/// \param entry 要插入的节点。
/// \param prev 插入位置之后的节点。
void list_add_after(hal_list_t *entry, hal_list_t *prev);

/// \brief 在指定节点之前插入新节点。
/// \param entry 要插入的节点。
/// \param next 插入位置之前的节点。
void list_add_before(hal_list_t *entry, hal_list_t *next);

/// \brief 从链表中移除指定节点。
/// \param entry 要移除的节点。
/// \return 被移除的节点指针。
hal_list_t *list_remove_entry(hal_list_t *entry);

// -----------------------------------------------------------------------------
// 内部查找函数
// -----------------------------------------------------------------------------

/// \brief 按名称查找组件。
/// \param name 组件名称。
/// \return 匹配的组件指针，未找到返回 nullptr。
extern hal_comp_t *halpr_find_comp_by_name(const char *name);

/// \brief 按名称查找线程。
/// \param name 线程名称。
/// \return 匹配的线程指针，未找到返回 nullptr。
extern hal_thread_t *halpr_find_thread_by_name(const char *name);

/// \brief 按名称查找导出的函数。
/// \param name 函数名称。
/// \return 匹配的函数指针，未找到返回 nullptr。
extern hal_funct_t *halpr_find_funct_by_name(const char *name);

/// \brief 按 ID 查找组件。
/// \param id 组件 ID。
/// \return 匹配的组件指针，未找到返回 nullptr。
extern hal_comp_t *halpr_find_comp_by_id(int id);

// -----------------------------------------------------------------------------
// 跨编译单元符号（hal_lib.c 拆分后暴露）
// -----------------------------------------------------------------------------

/// 当前 HAL 库的模块 ID。
extern int lib_module_id;

// -----------------------------------------------------------------------------
// 共享内存分配（从共享内存池分配）
// -----------------------------------------------------------------------------

/// \brief 从共享内存池的底部向上分配内存块。
/// \param size 要分配的字节数。
/// \return 分配到的内存指针，共享内存耗尽返回 nullptr。
extern void *shmalloc_up(long int size);

/// \brief 从共享内存池的顶部向下分配内存块（用于大块一次性分配）。
/// \param size 要分配的字节数。
/// \return 分配到的内存指针，共享内存耗尽返回 nullptr。
extern void *shmalloc_dn(long int size);

// -----------------------------------------------------------------------------
// 结构体分配（从空闲链表获取）
// -----------------------------------------------------------------------------

/// \brief 从空闲链表中分配一个组件结构体。
/// \return 分配到的组件指针，空闲链表为空返回 nullptr。
extern hal_comp_t *halpr_alloc_comp_struct(void);

/// \brief 释放组件结构体至空闲链表。
/// \param comp 要释放的组件指针。
extern void free_comp_struct(hal_comp_t *comp);

/// \brief 释放函数结构体至空闲链表。
/// \param funct 要释放的函数指针。
extern void free_funct_struct(hal_funct_t *funct);

/// \brief 释放函数条目结构体至空闲链表。
/// \param funct_entry 要释放的函数条目指针。
extern void free_funct_entry_struct(hal_funct_entry_t *funct_entry);

/// \brief 释放线程结构体至空闲链表。
/// \param thread 要释放的线程指针。
extern void free_thread_struct(hal_thread_t *thread);

/// \brief 释放参数结构体至空闲链表。
/// \param param 要释放的参数指针。
extern void free_param_struct(hal_param_t *param);

/// \brief 释放引脚结构体至空闲链表。
/// \param pin 要释放的引脚指针。
extern void free_pin_struct(hal_pin_t *pin);

/// \brief 释放 oldname 结构体至空闲链表。
/// \param oldname 要释放的 oldname 指针。
extern void free_oldname_struct(hal_oldname_t *oldname);

// -----------------------------------------------------------------------------
// 结构体分配（新建）
// -----------------------------------------------------------------------------

/// \brief 在共享内存中分配并初始化一个新的函数结构体。
/// \return 分配到的函数指针，失败返回 nullptr。
extern hal_funct_t *alloc_funct_struct(void);

/// \brief 在共享内存中分配并初始化一个新的函数条目结构体。
/// \return 分配到的函数条目指针，失败返回 nullptr。
extern hal_funct_entry_t *alloc_funct_entry_struct(void);

/// \brief 在共享内存中分配并初始化一个新的线程结构体。
/// \return 分配到的线程指针，失败返回 nullptr。
extern hal_thread_t *alloc_thread_struct(void);

/// \brief 在共享内存中分配并初始化一个新的参数结构体。
/// \return 分配到的参数指针，失败返回 nullptr。
extern hal_param_t *alloc_param_struct(void);

/// \brief 在共享内存中分配并初始化一个新的引脚结构体。
/// \return 分配到的引脚指针，失败返回 nullptr。
extern hal_pin_t *alloc_pin_struct(void);

// -----------------------------------------------------------------------------
// 共享内存初始化
// -----------------------------------------------------------------------------

/// \brief 初始化 HAL 共享内存数据区（hal_data_t 及所有链表）。
///
/// 由首次 hal_init() 调用，执行内存池布局、空闲链表初始化、互斥锁初始化。
///
/// \return 成功返回 0，失败返回负值 errno。
extern int init_hal_data(void);

#endif //HAL_PRIV_H
