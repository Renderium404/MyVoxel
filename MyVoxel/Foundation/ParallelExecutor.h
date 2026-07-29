#ifndef MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H
#define MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

#include "Diagnostic.h"
#include "ParallelOptions.h"

namespace MyVoxel
{
namespace Foundation
{

// 提供统一的同步并行任务执行入口。
class ParallelExecutor
{
public:
    // 创建并行执行器，最大线程数量为0时根据硬件并发数量自动确定。
    explicit ParallelExecutor(std::size_t maximumWorkerCount = 0);

    ParallelExecutor(const ParallelExecutor&) = delete;
    ParallelExecutor& operator=(const ParallelExecutor&) = delete;

    /// 执行器配置

    // 设置最大工作线程数量，0表示重新根据硬件并发数量确定。
    void setMaximumWorkerCount(std::size_t maximumWorkerCount);

    // 返回最大工作线程数量。
    std::size_t maximumWorkerCount() const;

    // 返回系统报告的硬件并发线程数量，无法获取时返回1。
    static std::size_t hardwareWorkerCount();

    // 返回全局默认并行执行器。
    static ParallelExecutor& global();

    /// 并行执行

    // 返回当前任务实际使用的工作线程数量。
    std::size_t effectiveWorkerCount(std::size_t taskCount, const ParallelForOptions& options = ParallelForOptions()) const;

    // 对[0, taskCount)中的每个任务索引执行一次函数，并等待全部任务完成。
    template<typename Function>
    void forEach(std::size_t taskCount, Function function, const ParallelForOptions& options = ParallelForOptions()) const
    {
        MYVOXEL_ASSERT(options.minimumParallelTaskCount > 0);

        if (taskCount == 0)
        {
            return;
        }

        const std::size_t workerCount = effectiveWorkerCount(taskCount, options);

        if (workerCount <= 1)
        {
            for (std::size_t taskIndex = 0; taskIndex < taskCount; ++taskIndex)
            {
                function(taskIndex);
            }

            return;
        }

        std::atomic<std::size_t> nextTaskIndex(0);
        std::atomic<bool> cancelled(false);

        std::mutex exceptionMutex;
        std::exception_ptr firstException;

        const auto worker = [&]()
        {
            while (!cancelled.load())
            {
                const std::size_t taskIndex = nextTaskIndex.fetch_add(1);

                if (taskIndex >= taskCount)
                {
                    return;
                }

                try
                {
                    function(taskIndex);
                }
                catch (...)
                {
                    bool expected = false;

                    if (cancelled.compare_exchange_strong(expected, true))
                    {
                        std::lock_guard<std::mutex> lock(exceptionMutex);
                        firstException = std::current_exception();
                    }

                    return;
                }
            }
        };

        std::vector<std::thread> workers;
        workers.reserve(workerCount - 1);

        try
        {
            for (std::size_t workerIndex = 1; workerIndex < workerCount; ++workerIndex)
            {
                workers.push_back(std::thread(worker));
            }
        }
        catch (...)
        {
            cancelled.store(true);

            for (std::size_t workerIndex = 0; workerIndex < workers.size(); ++workerIndex)
            {
                if (workers[workerIndex].joinable())
                {
                    workers[workerIndex].join();
                }
            }

            throw;
        }

        worker();

        for (std::size_t workerIndex = 0; workerIndex < workers.size(); ++workerIndex)
        {
            if (workers[workerIndex].joinable())
            {
                workers[workerIndex].join();
            }
        }

        if (firstException)
        {
            std::rethrow_exception(firstException);
        }
    }

private:
    std::atomic<std::size_t> m_maximumWorkerCount; // 当前执行器允许使用的最大工作线程数量。
};

}
}

#endif // MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H