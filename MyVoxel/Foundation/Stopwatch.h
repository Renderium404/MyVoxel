#ifndef MYVOXEL_FOUNDATION_STOPWATCH_H
#define MYVOXEL_FOUNDATION_STOPWATCH_H

#include <chrono>

namespace MyVoxel
{
namespace Foundation
{

// 提供基于稳定时钟的轻量性能计时。
class Stopwatch
{
public:
    using Clock = std::chrono::steady_clock;

    Stopwatch()
        : m_start(Clock::now())
    {
    }

    /// 计时控制

    // 重新开始计时。
    void restart()
    {
        m_start = Clock::now();
    }

    /// 计时结果

    // 返回已经经过的秒数。
    double elapsedSeconds() const
    {
        return std::chrono::duration<double>(Clock::now() - m_start).count();
    }

    // 返回已经经过的毫秒数。
    double elapsedMilliseconds() const
    {
        return std::chrono::duration<double, std::milli>(Clock::now() - m_start).count();
    }

    // 返回已经经过的微秒数。
    double elapsedMicroseconds() const
    {
        return std::chrono::duration<double, std::micro>(Clock::now() - m_start).count();
    }

private:
    Clock::time_point m_start; // 当前计时起点。
};

}
}

#endif // MYVOXEL_FOUNDATION_STOPWATCH_H