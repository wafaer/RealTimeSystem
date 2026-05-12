
#include "libnml/emc.h"
#include <csignal>
#include <stdlib.h>
#include "hal/hal.h"

extern "C" {
#include "rtapi/rtapi.h"
}


#include "ExampleTask/task.h"

volatile int done;
static RCS_TIMER *timer = 0;

static int emcTaskNoDelay = 0;
static int emcTaskEager = 0;
int num_axis = EMCMOT_MAX_AXIS;

void emctask_quit(int sig)
{
	// set main's done flag
	done = 1;
	// restore signal handler
	signal(sig, emctask_quit);
}

static int emctask_shutdown(void)
{
	// delete the timer
	if (0 != timer) {
		delete timer;
		timer = 0;
	}

	return 0;
}

static int emctask_startup()
{
    double end;
    int good;
	int retval;

	#define RETRY_TIME 10.0		// seconds to wait for subsystems to come up
	#define RETRY_INTERVAL 1.0	// seconds between wait tries for a subsystem



	// get the timer
	if (!emcTaskNoDelay)
	{
		timer = new RCS_TIMER(DEFAULT_EMC_TASK_CYCLE_TIME, "", "");
	}

    // 初始化运动
	if(motion_thread_main() != 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "can't not init motion_thread_main");
		return -1;
	}

	
	updata_axis_param(num_axis);

	//更新运动状态
    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == emcMotionUpdate())
	{
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done)
	{
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rtapi_print_msg(RTAPI_MSG_ERR,"can't read ExampleTask status\n");
	return -1;
    }
    if (done ) {
	emctask_shutdown();
	exit(1);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    double startTime, endTime, deltaTime;
	double first_start_time;
    int num_latency_warnings = 0;
    int latency_excursion_factor = 10;
    double minTime, maxTime;

    // loop until done
    done = 0;
    // trap ^C
    signal(SIGINT, emctask_quit);
    // and SIGTERM (used by runscript to shut down)
    signal(SIGTERM, emctask_quit);

    // initialize everything
    if (0 != emctask_startup())
    {
		emctask_shutdown();
    	stopprintf();
		exit(1);
    }

    startTime = etime();	// set start time before entering loop;
    first_start_time = startTime;
    endTime = startTime;
    // it will be set at end of loop from now on
    minTime = DBL_MAX;	// set to value that can never be exceeded
    maxTime = 0.0;		// set to value that can never be underset

	//开启线程
	hal_start_threads();

	//主循环
    while (!done)
    {
		emcMotionUpdate();

	    endTime = etime();
	    deltaTime = endTime - startTime;
	    if (deltaTime < minTime)
	        minTime = deltaTime;
	    else if (deltaTime > maxTime)
	        maxTime = deltaTime;
	    startTime = endTime;
	    if (!getenv( (char*)"QUIET_TASK") ) {
	            if (deltaTime > (latency_excursion_factor * DEFAULT_EMC_TASK_CYCLE_TIME))
	            	{
	                if (num_latency_warnings < 10) {
	                    rtapi_print_msg(RTAPI_MSG_INFO, "main: main loop took %.6f seconds\n", deltaTime);
	                }
	                num_latency_warnings ++;
	            }
	        }
		if ((emcTaskNoDelay) || (emcTaskEager)) {
		    emcTaskEager = 0;
		} else {
		    timer->wait();
		}
    }

	//停止线程
	hal_stop_threads();
	stopprintf();

	// 停止运动
	emcMotionAbort();
	emcTrajDisable();

    // clean up everything
    emctask_shutdown();

    // and leave
    exit(0);
}
