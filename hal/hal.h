//
// Created by Administrator on 2025/8/16.
//

#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <rtapi/rtapi_stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define HAL_NAME_LEN     47
#define HAL_LOCK_NONE     0     /* no locking done, any command is permitted */
#define HAL_LOCK_LOAD     1     /* loading rt components is not permitted */
#define HAL_LOCK_CONFIG   2     /* locking of link and addf related commands */
#define HAL_LOCK_PARAMS   4     /* locking of parameter set commands */
#define HAL_LOCK_RUN      8     /* locking of start/stop of HAL threads */

///函数定义
extern int hal_init(const char *name);
extern int hal_exit(int comp_id);
extern void *hal_malloc(long int size);
extern int hal_set_unready(int comp_id);
extern int hal_ready(int comp_id);
extern int hal_unready(int comp_id);
extern char *hal_comp_name(int comp_id);

typedef enum {
    HAL_TYPE_UNSPECIFIED = -1,
    HAL_TYPE_UNINITIALIZED = 0,
    HAL_BIT = 1,
    HAL_FLOAT = 2,
    HAL_S32 = 3,
    HAL_U32 = 4,
    HAL_PORT = 5,
    HAL_S64 = 6,
    HAL_U64 = 7,
    HAL_INT32 = 8,
    HAL_TYPE_MAX,
} hal_type_t;

typedef enum {
    HAL_RO = 64,
    HAL_RW = HAL_RO | 128 /* HAL_WO */,
} hal_param_dir_t;

///自定义类型结构
typedef volatile bool hal_bit_t;
typedef volatile rtapi_u32 hal_u32_t;
typedef volatile rtapi_s32 hal_s32_t;
typedef volatile rtapi_u64 hal_u64_t;
typedef volatile rtapi_s64 hal_s64_t;
typedef volatile int hal_port_t;
typedef double real_t __attribute__((aligned(8)));
typedef rtapi_u64 ireal_t __attribute__((aligned(8)));
#define hal_float_t volatile real_t

typedef union {
    hal_bit_t b;
    hal_s32_t s;
    hal_u32_t u;
    hal_float_t f;
    hal_port_t p;
    hal_s64_t ls;
    hal_u64_t lu;
} hal_data_u;

extern int hal_set_lock(unsigned char lock_type);
extern unsigned char hal_get_lock();
extern int hal_export_functf(void (*funct) (void *, long),
    void *arg, int uses_fp, int reentrant, int comp_id, const char *fmt, ...);
extern int hal_export_funct(const char *name, void (*funct) (void *, long),
    void *arg, int uses_fp, int reentrant, int comp_id);
extern int hal_create_thread(const char *name, unsigned long period_nsec, int uses_fp, int priority);
extern int hal_thread_delete(const char *name);
extern int hal_add_funct_to_thread(const char *funct_name, const char *thread_name, int position);
extern int hal_del_funct_from_thread(const char *funct_name, const char *thread_name);
extern int hal_start_threads(void);
extern int hal_stop_threads(void);

typedef int(*constructor)(char *prefix, char *arg);

extern int hal_app_main(void);
extern void hal_app_exit(void);

    extern int hal_param_bit_new(const char *name, hal_param_dir_t dir, hal_type_t type, hal_bit_t * data_addr, int comp_id);
extern int hal_param_s32_new(const char *name, hal_param_dir_t dir, hal_type_t type, hal_s32_t * data_addr,int comp_id);
    extern int hal_pin_newf(hal_type_t type, hal_s32_t ** data_ptr_addr, int comp_id, const char *fmt, ...);

extern int hal_param_new(const char *name, hal_type_t type, hal_param_dir_t dir, void *data_addr, int comp_id);

extern int hal_param_float_newf(hal_param_dir_t dir, hal_float_t * data_addr, int comp_id, const char *fmt, ...);
extern int hal_pin_new(const char *name, hal_type_t type, void **data_ptr_addr, int comp_id);
// extern int hal_pin_newfv(hal_type_t type, void ** data_ptr_addr, int comp_id, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif //HAL_H
