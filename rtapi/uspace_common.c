/// \file uspace_common.c
///
/// RTAPI 用户空间层的核心实现。
///
/// 实现 rtapi.h 中声明的共享内存管理、消息打印、UUID 生成等函数。

#include "rtapi/uspace_common.h"
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include "rtapi/rtapi.h"

extern uid_t ruid;
extern sem_t sem_count_usart_tx;

/// 当前全局日志级别，默认为输出所有级别。
static msg_level_t msg_level = RTAPI_MSG_ALL;

// -----------------------------------------------------------------------------
// 消息打印
// -----------------------------------------------------------------------------

/// \brief 按指定级别打印格式化消息（rtapi 版本）。
///
/// \param level 消息级别。
/// \param fmt   printf 风格格式字符串。
/// \param ...   可变参数。
void rtapi_print_msg(msg_level_t level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    default_rtapi_msg_handler(level, fmt, args);
    va_end(args);
}

/// \brief 按指定级别打印格式化消息（内部版本）。
///
/// \param level 消息级别。
/// \param fmt   printf 风格格式字符串。
/// \param ...   可变参数。
void print_msg(msg_level_t level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    default_msg_handler(level, fmt, args);
    va_end(args);
}

// -----------------------------------------------------------------------------
// 安全字符串格式化
// -----------------------------------------------------------------------------

/// \brief 安全字符串格式化，边界检查更严格。
///
/// \param buffer 输出缓冲区。
/// \param size   缓冲区大小（字节）。
/// \param msg    printf 风格格式字符串。
/// \param ...    可变参数。
/// \return 写入的字符数（不含终止符），失败返回负值。
int rt_snprintf(char *buffer, unsigned long int size, const char *msg, ...) {
    va_list args;
    int result;
    va_start(args, msg);
    result = vsnprintf(buffer, size, msg, args);
    va_end(args);
    return result;
}

/// \brief rt_snprintf 的 va_list 版本。
///
/// \param buffer 输出缓冲区。
/// \param size   缓冲区大小（字节）。
/// \param fmt    格式字符串。
/// \param args   已解包的可变参数列表。
/// \return 写入的字符数（不含终止符），失败返回负值。
int rt_vsnprintf(char *buffer, unsigned long int size, const char *fmt,
                 va_list args) {
    return vsnprintf(buffer, size, fmt, args);
}

// -----------------------------------------------------------------------------
// 共享内存管理
// -----------------------------------------------------------------------------

/// \brief 创建或获取一个共享内存段，完整流程分 6 阶段完成。
///
/// \param key       共享内存的标识 key，不能为 0。
/// \param module_id 调用模块的 ID（本实现中未使用）。
/// \param size      内存块大小（字节），必须大于 0。
/// \return 共享内存句柄（数组下标，>= 0），失败返回负值 errno。
int rtapi_shmem_new(int key, int module_id, unsigned long int size) {
    (void)module_id;
    rtapi_shmem_handle *shmem = NULL;
    int i;
    int saved_errno = 0;
    long pagesize;

    // Phase 1：参数校验。key 为 0 等价于 IPC_PRIVATE，会导致每次调用
    // 都创建一个无法被其他进程访问的段。
    if (key == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_shmem_new: key must be non-zero\n");
        return -EINVAL;
    }
    if (size == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_shmem_new: size must be > 0\n");
        return -EINVAL;
    }

    // Phase 2：遍历句柄数组，查找已有映射（key 相同则增加引用计数直接返回）
    // 或寻找第一个空闲槽位。shmem_array 为进程本地的静态存储，无需加锁。
    for (i = 0, shmem = NULL; i < MAX_SHM; i++) {
        if (shmem_array[i].magic == SHMEM_MAGIC) {
            if (shmem_array[i].key == key) {
                shmem_array[i].count++;
                return i;
            }
        } else if (!shmem) {
            shmem = &shmem_array[i];
        }
    }
    if (!shmem) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_shmem_new failed due to MAX_SHM\n");
        return -ENOMEM;
    }

    shmem->owner_pid = getpid();
    shmem->error_code = 0;

    // Phase 3：向内核请求共享内存段。
    // 若因 root 遗留的段导致 EPERM，则重试最多 5 次（uid 切换尚未传播）。
    {
        int shmget_retries = 5;
    shmget_again:
        shmem->id = shmget((key_t)key, (int)size, IPC_CREAT | 0600);
        if (shmem->id == -1) {
            if (shmget_retries-- && errno == EPERM) {
                sched_yield();
                goto shmget_again;
            }
            saved_errno = errno;
            shmem->error_code = saved_errno;
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_shmem_new failed due to shmget(key=0x%08x): %s\n",
                key, strerror(saved_errno));
            return -saved_errno;
        }
    }
    shmem->state = SHMEM_S_CREATED;

    // 若以 root 身份运行，将段的所有者改为真实 uid，使非 root 进程也能访问。
    {
        struct shmid_ds stat;
        int res = shmctl(shmem->id, IPC_STAT, &stat);
        if (res < 0) {
            perror("shmctl IPC_STAT");
        }
        if (geteuid() == 0) {
            stat.shm_perm.uid = ruid;
            res = shmctl(shmem->id, IPC_SET, &stat);
            if (res < 0) {
                perror("shmctl IPC_SET");
            }
        }
    }

    // Phase 4：将段映射到进程虚拟地址空间。
    shmem->mem = shmat(shmem->id, NULL, 0);
    if ((ssize_t)(shmem->mem) == -1) {
        saved_errno = errno;
        shmem->error_code = saved_errno;
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_shmem_new failed due to shmat(): %s\n",
            strerror(saved_errno));
        goto cleanup_created;
    }
    shmem->state = SHMEM_S_ATTACHED;

    // Phase 5：触碰映射的每一页，强制内核立即分配物理页框。
    // 这避免了实时控制循环执行时出现不可预知的缺页中断延迟。
    // volatile 读取防止编译器将循环优化掉。
    pagesize = sysconf(_SC_PAGESIZE);
    for (size_t off = 0; off < size; off += pagesize) {
        volatile char c = ((char *)shmem->mem)[off];
        (void)c;
    }

    // Phase 6：填充元数据并转入 ACTIVE 状态。
    shmem->magic = SHMEM_MAGIC;
    shmem->size  = size;
    shmem->key   = key;
    shmem->count = 1;
    shmem->state = SHMEM_S_ACTIVE;

    return shmem - shmem_array;

cleanup_created:
    {
        struct shmid_ds dummy;
        int rm_ret = shmctl(shmem->id, IPC_RMID, &dummy);
        if (rm_ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_shmem_new rollback: shmctl(IPC_RMID) failed: %s\n",
                strerror(errno));
        }
    }
    shmem->state = SHMEM_S_ERROR;
    shmem->id    = -1;
    shmem->mem   = NULL;
    return -saved_errno;
}

/// \brief 获取共享内存段的用户空间指针。
///
/// \param handle 句柄（rtapi_shmem_new 的返回值）。
/// \param ptr    输出参数，指向映射后指针的指针。
/// \return 成功返回 0；句柄无效或段未在 ACTIVE/ATTACHED 状态时返回负值 errno。
int rtapi_shmem_getptr(int handle, void **ptr) {
    rtapi_shmem_handle *shmem;

    if (handle < 0 || handle >= MAX_SHM)
        return -EINVAL;

    shmem = &shmem_array[handle];

    // 校验幻数，确认该句柄由 rtapi_shmem_new 成功创建。
    if (shmem->magic != SHMEM_MAGIC)
        return -EINVAL;

    // 状态守卫：仅在段已映射时返回指针。
    // UNBORN / DESTROYED 状态无有效映射；ERROR 状态可能部分初始化，不可使用。
    if (shmem->state != SHMEM_S_ACTIVE && shmem->state != SHMEM_S_ATTACHED) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_shmem_getptr: segment in invalid state %d (key=0x%08x)\n",
            shmem->state, shmem->key);
        return -EACCES;
    }

    *ptr = shmem->mem;
    return 0;
}

/// \brief 减少引用计数，最后一个引用退出时断开映射并通知内核销毁段。
///
/// \param handle    句柄。
/// \param module_id 调用模块的 ID（本实现中未使用）。
/// \return 成功返回 0；句柄无效或底层操作失败时返回 -EINVAL。
int rtapi_shmem_delete(int handle, int module_id) {
    struct shmid_ds d;
    int r1 = 0, r2 = 0;
    rtapi_shmem_handle *shmem;
    (void)module_id;

    if (handle < 0 || handle >= MAX_SHM)
        return -EINVAL;

    shmem = &shmem_array[handle];

    if (shmem->magic != SHMEM_MAGIC)
        return -EINVAL;

    // 减少引用计数。若本进程内仍有其他引用，直接返回，不操作映射或 OS 段。
    shmem->count--;
    if (shmem->count > 0)
        return 0;

    // count == 0：当前进程最后一个引用已释放。

    // 若段仍处于映射状态，则解除映射（ACTIVE 和 ATTACHED 均表示有有效映射）。
    if (shmem->state == SHMEM_S_ACTIVE || shmem->state == SHMEM_S_ATTACHED) {
        r1 = shmdt(shmem->mem);
        if (r1 < 0) {
            shmem->error_code = errno;
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_shmem_delete: shmdt(%p) failed: %s\n",
                shmem->mem, strerror(errno));
        }
        shmem->state = SHMEM_S_DETACHED;
        shmem->mem   = NULL;
    }

    // 查询内核：是否还有其他进程仍连接该段。若无，则可以安全销毁。
    r2 = shmctl(shmem->id, IPC_STAT, &d);
    if (r2 != 0) {
        shmem->error_code = errno;
        rtapi_print_msg(RTAPI_MSG_ERR,
            "shmctl(%d, IPC_STAT, ...): %s\n", shmem->id, strerror(errno));
    }

    if (r2 == 0 && d.shm_nattch == 0) {
        r2 = shmctl(shmem->id, IPC_RMID, &d);
        if (r2 != 0) {
            shmem->error_code = errno;
            rtapi_print_msg(RTAPI_MSG_ERR,
                "shmctl(%d, IPC_RMID, ...): %s\n", shmem->id, strerror(errno));
        }
        shmem->state = SHMEM_S_DESTROYED;
    }

    // 将槽位归还空闲池。
    shmem->magic = 0;

    if ((r1 != 0) || (r2 != 0))
        return -EINVAL;
    return 0;
}

/// \brief 查询共享内存段的当前生命周期状态。
///
/// \param handle 句柄。
/// \return 段状态（SHMEM_S_* 枚举值），句柄无效返回 SHMEM_S_ERROR。
int rtapi_shmem_getstate(int handle) {
    if (handle < 0 || handle >= MAX_SHM)
        return SHMEM_S_ERROR;

    rtapi_shmem_handle *shmem = &shmem_array[handle];
    if (shmem->magic != SHMEM_MAGIC)
        return SHMEM_S_ERROR;

    return shmem->state;
}

// -----------------------------------------------------------------------------
// 时钟
// -----------------------------------------------------------------------------

/// \brief 获取自系统启动以来的时钟滴答数（纳秒级时间戳）。
/// \return 累计计数，等价于调用 rdtscll 获取的 rtapi_get_time() 值。
long long rtapi_get_clocks(void) {
    long long int retval;
    rdtscll(retval);
    return retval;
}

// -----------------------------------------------------------------------------
// 模块初始化与 UUID 生成
// -----------------------------------------------------------------------------

/// \brief 初始化基于共享内存的 UUID 生成器并返回唯一的 ID 值。
///
/// 在首次调用时创建共享内存段和互斥锁，后续调用原子地递增并返回 UUID。
///
/// \param modname 模块名称（本实现中未使用）。
/// \return 分配到的 UUID（>= 1），失败返回负值 errno。
int rtapi_init(const char *modname) {
    (void)modname;
    static uuid_data_t *uuid_data       = 0;
    static const int    uuid_id          = 0;
    static char        *uuid_shmem_base = 0;
    int retval, id;
    void *uuid_mem;

    uuid_mem_id = rtapi_shmem_new(UUID_KEY, uuid_id, sizeof(uuid_data_t));
    if (uuid_mem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_init: could not open shared memory for uuid\n");
        rtapi_exit(uuid_id);
        return -EINVAL;
    }

    retval = rtapi_shmem_getptr(uuid_mem_id, &uuid_mem);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_init: could not access shared memory for uuid\n");
        rtapi_exit(uuid_id);
        return -EINVAL;
    }

    if (uuid_shmem_base == 0) {
        uuid_shmem_base = (char *)uuid_mem;
        uuid_data       = (uuid_data_t *)uuid_mem;
    }

    rtapi_mutex_get(&uuid_data->mutex);
    uuid_data->uuid++;
    id = uuid_data->uuid;
    rtapi_mutex_give(&uuid_data->mutex);

    return id;
}

/// \brief 卸载 RTAPI 模块，删除 UUID 共享内存段。
///
/// \param module_id 要卸载的模块 ID。
/// \return 始终返回 0。
int rtapi_exit(int module_id) {
    rtapi_shmem_delete(uuid_mem_id, module_id);
    return 0;
}
