#ifndef MYVOXEL_FOUNDATION_PARALLELEXECUTIONSTATISTICS_H
#define MYVOXEL_FOUNDATION_PARALLELEXECUTIONSTATISTICS_H

#include <cstddef>
#include <thread>
#include <vector>

namespace MyVoxel
{
namespace Foundation
{

// 保存一个参与线程在一次同步并行执行中的原始调度结果。
struct ParallelWorkerStatistics
{
    ParallelWorkerStatistics()
        : claimedBatchCount(0)
        , completedBatchCount(0)
        , claimedTaskCount(0)
        , completedTaskCount(0)
        , elapsedMilliseconds(0.0)
        , callingThread(false)
    {
    }

    std::thread::id threadId; // 当前参与线程标识。
    std::size_t claimedBatchCount; // 当前线程领取的批次数量。
    std::size_t completedBatchCount; // 当前线程完整完成的批次数量。
    std::size_t claimedTaskCount; // 当前线程领取的任务索引数量。
    std::size_t completedTaskCount; // 当前线程完整完成的任务索引数量。
    double elapsedMilliseconds; // 当前线程进入执行循环后的持续时间。
    bool callingThread; // 当前记录是否属于发起本次同步调用的线程。
};

// 保存一次同步并行执行产生的原始调度事实。
//
// 统计对象只能由一次正在执行的execute调用写入。
// 同一个统计对象不能同时传给多个execute调用。
struct ParallelExecutionStatistics
{
    ParallelExecutionStatistics()
    {
        clear();
    }

    // 清除全部统计结果。
    void clear()
    {
        serialExecution = false;
        nestedSerialExecution = false;
        cancellationRequested = false;
        exceptionOccurred = false;

        taskCount = 0;
        requestedWorkerCount = 0;
        minimumParallelTaskCount = 0;
        participantLimit = 0;
        submittedWorkerTaskCount = 0;
        batchSize = 0;

        elapsedMilliseconds = 0.0;
        workers.clear();
    }

    bool serialExecution; // 当前调用是否使用单参与线程执行。
    bool nestedSerialExecution; // 当前调用是否因位于同一线程池工作线程中而使用单参与线程执行。
    bool cancellationRequested; // 当前调用是否实际观察到外部取消请求。
    bool exceptionOccurred; // 当前调用是否捕获到任务异常或任务提交异常。

    std::size_t taskCount; // 当前请求处理的任务索引数量。
    std::size_t requestedWorkerCount; // 调用配置指定的最大参与线程数量，0表示未单独限制。
    std::size_t minimumParallelTaskCount; // 当前调用使用的最小并行任务阈值。
    std::size_t participantLimit; // 当前调用解析得到的最大参与线程数量。
    std::size_t submittedWorkerTaskCount; // 当前调用实际提交到线程池的参与任务数量。
    std::size_t batchSize; // 当前调度实际使用的最大批次大小。

    double elapsedMilliseconds; // 当前同步execute调用的完整持续时间。
    std::vector<ParallelWorkerStatistics> workers; // 实际领取过至少一个批次的参与线程记录。
};

}
}

#endif // MYVOXEL_FOUNDATION_PARALLELEXECUTIONSTATISTICS_H