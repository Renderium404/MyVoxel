#ifndef MYVOXEL_FOUNDATION_PARALLELOPTIONS_H
#define MYVOXEL_FOUNDATION_PARALLELOPTIONS_H

#include <cstddef>

#include "Cancellation.h"

namespace MyVoxel
{
namespace Foundation
{

struct ParallelExecutionStatistics;

// 配置一次ParallelExecutor::execute调用。
struct ParallelOptions
{
    ParallelOptions()
        : workerCount(0)
        , minimumParallelTaskCount(2)
        , batchSize(0)
        , statistics(nullptr)
    {
    }

    std::size_t workerCount; // 本次调用允许使用的最大参与线程数量，0表示使用执行器允许的全部线程。
    std::size_t minimumParallelTaskCount; // 任务数量低于该值时使用单参与线程执行。
    std::size_t batchSize; // 每次动态领取的连续任务数量，0表示根据任务数量和参与线程数量自动确定。
    ParallelExecutionStatistics* statistics; // 可选原始调度统计输出，不需要统计时保持为空。
    CancellationToken cancellationToken; // 可选外部协作取消令牌，空令牌表示不支持外部取消。
};

}
}

#endif // MYVOXEL_FOUNDATION_PARALLELOPTIONS_H