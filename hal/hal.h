/// \file hal.h
///
/// 硬件抽象层（HAL）公开接口声明。
///
/// 定义组件管理（初始化/就绪/销毁）、内存分配、线程管理、
/// 函数导出、引脚/参数注册等 HAL 核心 API。
/// 所有函数均可被 C/C++ 代码调用（已用 extern "C" 包裹）。

#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <rtapi/rtapi_stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// 名称长度与锁级别
// -----------------------------------------------------------------------------

/// HAL 对象（组件、线程、函数、引脚、参数）名称的最大字符数（含终止符）。
#define HAL_NAME_LEN     47

/// 不施加任何锁定；允许执行所有命令。
#define HAL_LOCK_NONE     0
/// 禁止加载实时组件。
#define HAL_LOCK_LOAD     1
/// 禁止 link 和 addf 相关命令。
#define HAL_LOCK_CONFIG   2
/// 禁止设置参数值。
#define HAL_LOCK_PARAMS   4
/// 禁止启动/停止 HAL 线程。
#define HAL_LOCK_RUN      8

// -----------------------------------------------------------------------------
// 组件生命周期
// -----------------------------------------------------------------------------

/// \brief 初始化 HAL 组件并返回组件 ID。
/// \param name 组件名称（不超过 HAL_NAME_LEN 字符）。
/// \return 组件 ID（>= 0），失败返回负值。
extern int hal_init(const char *name);

/// \brief 销毁 HAL 组件，释放所有关联资源。
/// \param comp_id 组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_exit(int comp_id);

/// \brief 在 HAL 共享内存池中分配指定大小的内存。
///
/// 分配的内存可被多个 HAL 组件共享，适用于组件间通过引脚传递数据。
///
/// \param size 要分配的字节数。
/// \return 指向分配内存的指针，失败返回 nullptr。
extern void *hal_malloc(long int size);

/// \brief 将组件标记为未就绪状态（初始化中途失败时使用）。
/// \param comp_id 组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_set_unready(int comp_id);

/// \brief 将组件标记为就绪状态，允许其引脚和参数被其他组件使用。
/// \param comp_id 组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_ready(int comp_id);

/// \brief 将组件从就绪状态取消（可用于组件内部重置流程）。
/// \param comp_id 组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_unready(int comp_id);

/// \brief 根据组件 ID 查询其名称字符串。
/// \param comp_id 组件 ID。
/// \return 组件名称字符串指针；未找到返回 nullptr。
extern char *hal_comp_name(int comp_id);

// -----------------------------------------------------------------------------
// 数据类型枚举
// -----------------------------------------------------------------------------

/// \brief HAL 引脚和参数的数据类型枚举。
typedef enum {
    HAL_TYPE_UNSPECIFIED = -1,   ///< 未指定类型。
    HAL_TYPE_UNINITIALIZED = 0,  ///< 未初始化。
    HAL_BIT = 1,                 ///< 布尔位（volatile bool）。
    HAL_FLOAT = 2,               ///< 单精度浮点。
    HAL_S32 = 3,                 ///< 有符号 32 位整数。
    HAL_U32 = 4,                 ///< 无符号 32 位整数。
    HAL_PORT = 5,                ///< 端口类型。
    HAL_S64 = 6,                 ///< 有符号 64 位整数。
    HAL_U64 = 7,                 ///< 无符号 64 位整数。
    HAL_INT32 = 8,               ///< 32 位整数（别名）。
    HAL_TYPE_MAX,                 ///< 枚举上界（用于边界检查）。
} hal_type_t;

// -----------------------------------------------------------------------------
// 参数读写方向
// -----------------------------------------------------------------------------

/// \brief HAL 参数的读写方向枚举。
typedef enum {
    HAL_RO = 64,                        ///< 只读参数。
    HAL_RW = HAL_RO | 128,              ///< 读写参数（低 7 位为只写标记，保留）。
} hal_param_dir_t;

// -----------------------------------------------------------------------------
// 自定义数据类型
//
// 所有类型均声明为 volatile，以保证在实时多线程环境中的可见性。
// float 和 double 对齐至 8 字节边界以支持高效的 SIMD 操作。
// -----------------------------------------------------------------------------

/// 布尔位类型，等价于 volatile bool。
typedef volatile bool hal_bit_t;
/// 无符号 32 位整数类型。
typedef volatile rtapi_u32 hal_u32_t;
/// 有符号 32 位整数类型。
typedef volatile rtapi_s32 hal_s32_t;
/// 无符号 64 位整数类型。
typedef volatile rtapi_u64 hal_u64_t;
/// 有符号 64 位整数类型。
typedef volatile rtapi_s64 hal_s64_t;
/// 端口类型。
typedef volatile int hal_port_t;

/// 内部浮点类型，8 字节对齐以支持 SSE/AVX。
typedef double real_t __attribute__((aligned(8)));
/// 内部无符号整数类型，8 字节对齐。
typedef rtapi_u64 ireal_t __attribute__((aligned(8)));

/// HAL 浮点类型，等价于 volatile real_t。
#define hal_float_t volatile real_t

/// \brief HAL 数据联合体，封装所有支持类型的存储视图。
///
/// 同一块内存可按不同类型解读；使用时须根据 hal_type_t 确定实际视图。
typedef union {
    hal_bit_t b;      ///< 按布尔位访问。
    hal_s32_t s;      ///< 按有符号 32 位整数访问。
    hal_u32_t u;      ///< 按无符号 32 位整数访问。
    hal_float_t f;    ///< 按浮点数访问。
    hal_port_t p;    ///< 按端口类型访问。
    hal_s64_t ls;     ///< 按有符号 64 位整数访问。
    hal_u64_t lu;     ///< 按无符号 64 位整数访问。
} hal_data_u;

// -----------------------------------------------------------------------------
// 锁级别
// -----------------------------------------------------------------------------

/// \brief 设置 HAL 全局锁级别。
/// \param lock_type 锁级别（HAL_LOCK_* 的按位 OR 组合）。
/// \return 成功返回 0，失败返回负值。
extern int hal_set_lock(unsigned char lock_type);

/// \brief 获取当前 HAL 全局锁级别。
/// \return 当前锁级别。
extern unsigned char hal_get_lock();

// -----------------------------------------------------------------------------
// 函数导出与线程管理
// -----------------------------------------------------------------------------

/// \brief 导出函数至 HAL（支持格式化名称构造）。
///
/// \param funct     函数入口。
/// \param arg       传递给 funct 的通用参数。
/// \param uses_fp   是否使用浮点单元（非零表示使用）。
/// \param reentrant 函数是否可重入调用。
/// \param comp_id   所属组件 ID。
/// \param fmt       printf 风格格式字符串，用于构造 HAL 内部函数名。
/// \param ...       格式参数。
/// \return 成功返回 0，失败返回负值。
extern int hal_export_functf(void (*funct)(void *, long), void *arg,
                             int uses_fp, int reentrant, int comp_id,
                             const char *fmt, ...);

/// \brief 导出函数至 HAL（固定名称版本）。
///
/// \param name      函数在 HAL 中的名称。
/// \param funct     函数入口。
/// \param arg       传递给 funct 的通用参数。
/// \param uses_fp   是否使用浮点单元。
/// \param reentrant 是否可重入。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_export_funct(const char *name,
                             void (*funct)(void *, long), void *arg,
                             int uses_fp, int reentrant, int comp_id);

/// \brief 创建 HAL 实时线程。
///
/// \param name           线程名称。
/// \param period_nsec    调度周期（纳秒）。
/// \param uses_fp         是否使用浮点单元。
/// \param priority        线程优先级。
/// \return 成功返回 0，失败返回负值。
extern int hal_create_thread(const char *name, unsigned long period_nsec,
                             int uses_fp, int priority);

/// \brief 删除指定的 HAL 线程。
/// \param name 线程名称。
/// \return 成功返回 0，失败返回负值。
extern int hal_thread_delete(const char *name);

/// \brief 将函数追加到指定线程的调度队列中。
///
/// \param funct_name  函数名称。
/// \param thread_name 目标线程名称。
/// \param position    在线程调度序列中的位置（越小越靠前执行）。
/// \return 成功返回 0，失败返回负值。
extern int hal_add_funct_to_thread(const char *funct_name,
                                   const char *thread_name, int position);

/// \brief 将函数从指定线程的调度队列中移除。
/// \param funct_name  函数名称。
/// \param thread_name 目标线程名称。
/// \return 成功返回 0，失败返回负值。
extern int hal_del_funct_from_thread(const char *funct_name,
                                    const char *thread_name);

/// \brief 启动所有已注册的 HAL 线程。
/// \return 成功返回 0，失败返回负值。
extern int hal_start_threads(void);

/// \brief 停止所有正在运行的 HAL 线程。
/// \return 成功返回 0，失败返回负值。
extern int hal_stop_threads(void);

/// \brief 获取 HAL 系统的整体运行状态。
/// \return HAL 状态码（具体定义见 hal_priv.h）。
extern int hal_get_state(void);

// -----------------------------------------------------------------------------
// 组件构造函数类型
// -----------------------------------------------------------------------------

/// \brief 组件构造函数函数指针类型。
///
/// \param prefix 组件名称前缀。
/// \param arg    构造参数。
/// \return 成功返回 0，失败返回负值。
typedef int (*constructor)(char *prefix, char *arg);

// -----------------------------------------------------------------------------
// HAL 应用入口
// -----------------------------------------------------------------------------

/// \brief 运行 HAL 应用主循环，初始化所有组件并启动线程。
/// \return 成功返回 0，失败返回负值。
extern int hal_app_main(void);

/// \brief 退出 HAL 应用，停止线程并释放资源。
extern void hal_app_exit(void);

// -----------------------------------------------------------------------------
// 参数与引脚注册
// -----------------------------------------------------------------------------

/// \brief 注册一个布尔型 HAL 参数。
///
/// \param name      参数名称。
/// \param dir       读写方向（HAL_RO / HAL_RW）。
/// \param type      参数类型（必须为 HAL_BIT）。
/// \param data_addr 参数数据的地址。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_param_bit_new(const char *name, hal_param_dir_t dir,
                              hal_type_t type, hal_bit_t *data_addr,
                              int comp_id);

/// \brief 注册一个有符号 32 位整数型 HAL 参数。
///
/// \param name      参数名称。
/// \param dir       读写方向。
/// \param type      参数类型（必须为 HAL_S32）。
/// \param data_addr 参数数据的地址。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_param_s32_new(const char *name, hal_param_dir_t dir,
                              hal_type_t type, hal_s32_t *data_addr,
                              int comp_id);

/// \brief 注册 HAL 引脚，支持格式化名称构造。
///
/// \param type          引脚类型（hal_type_t）。
/// \param data_ptr_addr 输出参数，指向存放数据指针地址的变量。
/// \param comp_id       所属组件 ID。
/// \param fmt           printf 风格格式字符串。
/// \param ...           格式参数。
/// \return 成功返回 0，失败返回负值。
extern int hal_pin_newf(hal_type_t type, hal_s32_t **data_ptr_addr,
                        int comp_id, const char *fmt, ...);

/// \brief 注册通用 HAL 参数（支持所有类型）。
///
/// \param name      参数名称。
/// \param type      参数类型。
/// \param dir       读写方向。
/// \param data_addr 数据地址。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_param_new(const char *name, hal_type_t type,
                          hal_param_dir_t dir, void *data_addr,
                          int comp_id);

/// \brief 注册浮点型 HAL 参数，支持格式化名称构造。
///
/// \param dir       读写方向。
/// \param data_addr 参数数据地址。
/// \param comp_id   所属组件 ID。
/// \param fmt       printf 风格格式字符串。
/// \param ...       格式参数。
/// \return 成功返回 0，失败返回负值。
extern int hal_param_float_newf(hal_param_dir_t dir, hal_float_t *data_addr,
                                 int comp_id, const char *fmt, ...);

/// \brief 注册通用 HAL 引脚（固定名称版本）。
///
/// \param name           引脚名称。
/// \param type           引脚类型。
/// \param data_ptr_addr  输出参数，指向存放数据指针地址的变量。
/// \param comp_id        所属组件 ID。
/// \return 成功返回 0，失败返回负值。
extern int hal_pin_new(const char *name, hal_type_t type,
                        void **data_ptr_addr, int comp_id);

#ifdef __cplusplus
}
#endif

#endif //HAL_H
