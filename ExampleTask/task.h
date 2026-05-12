#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <limits.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtapi/rtapi_mutex.h"

	typedef struct
	{
		hal_s32_t *s32_param1;
		hal_s32_t *s32_param2;
		hal_s32_t *s32_param3;

		hal_bit_t *bit_param1;
		hal_bit_t *bit_param2;
		hal_bit_t *bit_param3;
	} task_hal_t;

	//函数定义
	int motion_thread_main(void);
	void updata_axis_param(int numAxes);
	void motion_thread_exit(void);
	static int init_motion_comm_buffers(void);
	void emcmot_config_change(void);
	static int init_motion_threads(void);
	void emcmotSetCycleTime(unsigned long nsec );
	static int setTrajCycleTime(double secs);
	static int setServoCycleTime(double secs);

	extern void taskController(void *arg, long period);
	extern void taskCommandHandler(void *arg, long period);

#ifdef __cplusplus
}
#endif

#endif //MOTION_H
