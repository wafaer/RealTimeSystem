#include <osal.h>
#include <stdlib.h>
#include <time.h>
#include "task.h"
#include "rtapi/rtapi.h"
#include "rtapi/rtapi_mutex.h"

void taskCommandHandler_locked(void *arg, long period)
{
    (void)arg;

}

void taskCommandHandler(void *arg, long period)
{
    if (rtapi_mutex_try(&taskStruct->command_mutex) != 0) {

        return;  /* 锁被占用，跳过本次处理，等待下一周期重试 */
    }

    /* 成功获取锁：执行实际的命令处理（在锁保护下） */
    taskCommandHandler_locked(arg, period);
    /* 处理完成：释放互斥锁，允许 Task 层继续写入命令 */
    rtapi_mutex_give(&taskStruct->command_mutex);
}
