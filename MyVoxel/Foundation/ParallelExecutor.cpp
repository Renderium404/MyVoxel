#include "ParallelExecutor.h"

namespace MyVoxel
{
namespace Foundation
{

namespace
{

// 将用户配置的线程数量解析为至少一个工作线程。
std::size_t resolveWorkerCount(std::size_t workerCount)
{
    if (workerCount > 0)
    {
        return workerCount;
    }

    return ParallelExecutor::hardwareWorkerCount();
}

}

ParallelExecutor::ParallelExecutor(std::size_t maximumWorkerCount)
    : m_maximumWorkerCount(resolveWorkerCount(maximumWorkerCount))
{
}

void ParallelExecutor::setMaximumWorkerCount(std::size_t maximumWorkerCount)
{
    m_maximumWorkerCount.store(resolveWorkerCount(maximumWorkerCount));
}

std::size_t ParallelExecutor::maximumWorkerCount() const
{
    return m_maximumWorkerCount.load();
}

std::size_t ParallelExecutor::hardwareWorkerCount()
{
    const unsigned int workerCount = std::thread::hardware_concurrency();
    return workerCount > 0 ? static_cast<std::size_t>(workerCount) : static_cast<std::size_t>(1);
}

ParallelExecutor& ParallelExecutor::global()
{
    static ParallelExecutor executor;
    return executor;
}

std::size_t ParallelExecutor::effectiveWorkerCount(std::size_t taskCount, const ParallelForOptions& options) const
{
    if (taskCount == 0)
    {
        return 0;
    }

    MYVOXEL_ASSERT(options.minimumParallelTaskCount > 0);

    if (taskCount < options.minimumParallelTaskCount)
    {
        return 1;
    }

    const std::size_t maximumCount = maximumWorkerCount();
    const std::size_t requestedCount = options.workerCount > 0 ? std::min(options.workerCount, maximumCount) : maximumCount;

    return std::max<std::size_t>(1, std::min(requestedCount, taskCount));
}

}
}