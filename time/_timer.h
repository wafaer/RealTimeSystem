/// \file _timer.h
///
/// 实时系统计时接口定义。
///
/// 提供基于系统时钟的休眠、计时及时钟分辨率查询功能。

#ifndef REALTIMESYSTEM__TIMER_H
#define REALTIMESYSTEM__TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/// \brief 进程级休眠。
///
/// 使当前进程挂起指定时长（精度受系统时钟滴答分辨率限制）。
///
/// \param secs 休眠时长（秒）。
extern void esleep(double secs);

/// \brief 获取自任意基准时刻起的秒数。
///
/// \return 自基准时刻起的秒数（双精度浮点）。
extern double etime();

/// \brief 获取系统时钟滴答频率。
///
/// \return 每秒时钟滴答数（通常为 CLOCKS_PER_SEC 或 sysconf(_SC_CLK_TCK)）。
extern double clk_tck();

#ifdef __cplusplus
}
#endif

#endif  // REALTIMESYSTEM__TIMER_H
