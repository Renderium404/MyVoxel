#include "ParallelExecutor.h"

#include <algorithm>
#include <limits>

namespace MyVoxel
{
namespace Foundation
{

ParallelExecutor::ParallelExecutor(std::size_t maximumWorkerCountValue)
    : m_threadPool(&ThreadPool::global())
    , m_maximumWorkerCount(1)
{
    setMaximumWorkerCount(maximumWorkerCountValue);
}

ParallelExecutor::ParallelExecutor(ThreadPool& threadPoolValue, std::size_t maximumWorkerCountValue)
    : m_threadPool(&threadPoolValue)
    , m_maximumWorkerCount(1)
{
    setMaximumWorkerCount(maximumWorkerCountValue);
}

/// 执行器配置

void ParallelExecutor::setMaximumWorkerCount(std::size_t maximumWorkerCountValue)
{
    const std::size_t poolWorkerCount = m_threadPool->workerCount();
    const std::size_t resolvedWorkerCount =
        maximumWorkerCountValue > 0
            ? (std::min)(maximumWorkerCountValue, poolWorkerCount)
            : poolWorkerCount;

    m_maximumWorkerCount.store(
        (std::max)(static_cast<std::size_t>(1), resolvedWorkerCount),
        std::memory_order_release);
}

std::size_t ParallelExecutor::maximumWorkerCount() const
{
    return m_maximumWorkerCount.load(std::memory_order_acquire);
}

ThreadPool& ParallelExecutor::threadPool()
{
    return *m_threadPool;
}

const ThreadPool& ParallelExecutor::threadPool() const
{
    return *m_threadPool;
}

ParallelExecutor& ParallelExecutor::global()
{
    // 全局执行器持续到进程结束，其线程池同样采用进程生命周期。
    static ParallelExecutor* executor = new ParallelExecutor(ThreadPool::global());
    return *executor;
}

/// 执行策略

std::size_t ParallelExecutor::resolveParticipantCount(std::size_t taskCount, const ParallelOptions& options) const
{
    if (taskCount == 0)
    {
        return 0;
    }

    if (taskCount < options.minimumParallelTaskCount || m_threadPool->isWorkerThread())
    {
        return 1;
    }

    const std::size_t executorLimit = maximumWorkerCount();
    const std::size_t requestedLimit =
        options.workerCount > 0
            ? (std::min)(options.workerCount, executorLimit)
            : executorLimit;

    return (std::max)(
        static_cast<std::size_t>(1),
        (std::min)(requestedLimit, taskCount));
}

std::size_t ParallelExecutor::resolveBatchSize(std::size_t taskCount,
                                               std::size_t participantCount,
                                               std::size_t requestedBatchSize)
{
    if (taskCount == 0)
    {
        return 0;
    }

    if (requestedBatchSize > 0)
    {
        return (std::min)(requestedBatchSize, taskCount);
    }

    const std::size_t batchesPerParticipant = 4; // 自动调度为每个参与线程准备约四个批次，以平衡领取开销和负载差异。
    const std::size_t maximumBatchCount =
        participantCount > (std::numeric_limits<std::size_t>::max)() / batchesPerParticipant
            ? (std::numeric_limits<std::size_t>::max)()
            : participantCount * batchesPerParticipant;

    const std::size_t targetBatchCount =
        (std::max)(
            static_cast<std::size_t>(1),
            (std::min)(taskCount, maximumBatchCount));

    const std::size_t quotient = taskCount / targetBatchCount;
    const std::size_t remainder = taskCount % targetBatchCount;

    return quotient + (remainder != 0 ? 1 : 0);
}

}
}