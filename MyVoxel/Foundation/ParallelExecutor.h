#ifndef MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H
#define MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "Diagnostic.h"
#include "ParallelExecutionStatistics.h"
#include "ParallelOptions.h"
#include "ThreadPool.h"

namespace MyVoxel
{
namespace Foundation
{

// 提供基于ThreadPool的同步动态批次执行原语。
class ParallelExecutor
{
public:
    // 使用全局线程池创建执行器，maximumWorkerCount为0时允许使用线程池全部工作线程。
    explicit ParallelExecutor(std::size_t maximumWorkerCount = 0);

    // 使用指定线程池创建执行器，maximumWorkerCount为0时允许使用该线程池全部工作线程。
    ParallelExecutor(ThreadPool& threadPool, std::size_t maximumWorkerCount = 0);

    ParallelExecutor(const ParallelExecutor&) = delete;
    ParallelExecutor& operator=(const ParallelExecutor&) = delete;
    ParallelExecutor(ParallelExecutor&&) = delete;
    ParallelExecutor& operator=(ParallelExecutor&&) = delete;

    /// 执行器配置

    // 设置执行器允许使用的最大参与线程数量，0表示使用线程池全部工作线程。
    void setMaximumWorkerCount(std::size_t maximumWorkerCount);

    // 返回执行器允许使用的最大参与线程数量。
    std::size_t maximumWorkerCount() const;

    // 返回当前执行器使用的线程池。
    ThreadPool& threadPool();

    // 返回当前执行器使用的只读线程池。
    const ThreadPool& threadPool() const;

    // 返回使用全局线程池的默认并行执行器。
    static ParallelExecutor& global();

    /// 同步执行

    // 动态领取并完整处理[begin,end)中的连续批次，等待全部参与线程结束后返回。
    template<typename Function>
    void execute(std::size_t begin, std::size_t end, Function function, const ParallelOptions& options = ParallelOptions()) const;

private:
    // 返回当前调用允许使用的参与线程数量。
    std::size_t resolveParticipantCount(std::size_t taskCount, const ParallelOptions& options) const;

    // 返回当前调用实际使用的批次大小。
    static std::size_t resolveBatchSize(std::size_t taskCount, std::size_t participantCount, std::size_t requestedBatchSize);

private:
    ThreadPool* m_threadPool; // 当前执行器使用的固定线程池。
    std::atomic<std::size_t> m_maximumWorkerCount; // 当前执行器允许使用的最大参与线程数量。
};

template<typename Function>
void ParallelExecutor::execute(std::size_t begin, std::size_t end, Function function, const ParallelOptions& options) const
{
    typedef std::chrono::steady_clock Clock;

    MYVOXEL_REQUIRE_MESSAGE(options.minimumParallelTaskCount > 0, "Parallel minimum task count must be greater than zero.");

    ParallelExecutionStatistics* statistics = options.statistics;
    const Clock::time_point executionStart = Clock::now();

    if (statistics)
    {
        statistics->clear();
        statistics->requestedWorkerCount = options.workerCount;
        statistics->minimumParallelTaskCount = options.minimumParallelTaskCount;
    }

    if (begin >= end)
    {
        if (statistics)
        {
            statistics->serialExecution = true;
            statistics->elapsedMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - executionStart).count();
        }

        return;
    }

    const std::size_t taskCount = end - begin;
    const bool nestedExecution = m_threadPool->isWorkerThread();
    const std::size_t participantCount = resolveParticipantCount(taskCount, options);
    const std::size_t batchSize = resolveBatchSize(taskCount, participantCount, options.batchSize);

    if (statistics)
    {
        statistics->taskCount = taskCount;
        statistics->participantLimit = participantCount;
        statistics->batchSize = batchSize;
        statistics->serialExecution = participantCount == 1;
        statistics->nestedSerialExecution = nestedExecution && participantCount == 1;
    }

    if (options.cancellationToken.isCancellationRequested())
    {
        if (statistics)
        {
            statistics->cancellationRequested = true;
            statistics->elapsedMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - executionStart).count();
        }

        return;
    }

    struct ExecutionState
    {
        ExecutionState(std::size_t beginValue,
                       std::size_t taskCountValue,
                       std::size_t batchSizeValue,
                       std::size_t participantCountValue,
                       const std::shared_ptr<Function>& functionValue,
                       const CancellationToken& cancellationTokenValue)
            : beginIndex(beginValue)
            , taskCount(taskCountValue)
            , batchSize(batchSizeValue)
            , nextOffset(0)
            , function(functionValue)
            , cancellationToken(cancellationTokenValue)
            , stopRequested(false)
            , cancellationObserved(false)
            , workers(participantCountValue)
        {
        }

        std::size_t beginIndex;
        std::size_t taskCount;
        std::size_t batchSize;
        std::atomic<std::size_t> nextOffset;
        std::shared_ptr<Function> function;
        CancellationToken cancellationToken;
        std::atomic<bool> stopRequested;
        std::atomic<bool> cancellationObserved;
        std::mutex exceptionMutex;
        std::exception_ptr firstException;
        std::vector<ParallelWorkerStatistics> workers;
    };

    const std::shared_ptr<Function> sharedFunction =
        std::make_shared<Function>(
            std::move(function));

    const std::shared_ptr<ExecutionState> state =
        std::make_shared<ExecutionState>(
            begin,
            taskCount,
            batchSize,
            participantCount,
            sharedFunction,
            options.cancellationToken);

    const std::function<void(std::size_t)> worker =
        [state](std::size_t participantIndex)
        {
            ParallelWorkerStatistics& workerStatistics = state->workers[participantIndex];

            workerStatistics.threadId = std::this_thread::get_id();
            workerStatistics.callingThread = participantIndex == 0;

            const Clock::time_point workerStart = Clock::now();

            while (!state->stopRequested.load(std::memory_order_acquire))
            {
                if (state->cancellationToken.isCancellationRequested())
                {
                    state->cancellationObserved.store(true, std::memory_order_release);
                    state->stopRequested.store(true, std::memory_order_release);
                    break;
                }

                const std::size_t blockOffset =
                    state->nextOffset.fetch_add(
                        state->batchSize,
                        std::memory_order_relaxed);

                if (blockOffset >= state->taskCount)
                {
                    break;
                }

                const std::size_t remainingTaskCount = state->taskCount - blockOffset;
                const std::size_t currentBatchSize = (std::min)(state->batchSize, remainingTaskCount);
                const std::size_t blockBegin = state->beginIndex + blockOffset;
                const std::size_t blockEnd = blockBegin + currentBatchSize;

                ++workerStatistics.claimedBatchCount;
                workerStatistics.claimedTaskCount += currentBatchSize;

                if (state->cancellationToken.isCancellationRequested())
                {
                    state->cancellationObserved.store(true, std::memory_order_release);
                    state->stopRequested.store(true, std::memory_order_release);
                    break;
                }

                try
                {
                    (*state->function)(blockBegin, blockEnd);

                    ++workerStatistics.completedBatchCount;
                    workerStatistics.completedTaskCount += currentBatchSize;
                }
                catch (...)
                {
                    state->stopRequested.store(true, std::memory_order_release);

                    std::lock_guard<std::mutex> lock(state->exceptionMutex);

                    if (!state->firstException)
                    {
                        state->firstException = std::current_exception();
                    }

                    break;
                }
            }

            workerStatistics.elapsedMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - workerStart).count();
        };

    std::vector<std::future<void>> futures;
    futures.reserve(participantCount - 1);

    try
    {
        for (std::size_t participantIndex = 1; participantIndex < participantCount; ++participantIndex)
        {
            futures.push_back(m_threadPool->submit(worker, participantIndex));
        }
    }
    catch (...)
    {
        const std::exception_ptr submissionException = std::current_exception();

        state->stopRequested.store(true, std::memory_order_release);

        {
            std::lock_guard<std::mutex> lock(state->exceptionMutex);

            if (!state->firstException)
            {
                state->firstException = submissionException;
            }
        }

        for (std::size_t futureIndex = 0; futureIndex < futures.size(); ++futureIndex)
        {
            futures[futureIndex].wait();
        }

        if (statistics)
        {
            statistics->exceptionOccurred = true;
            statistics->submittedWorkerTaskCount = futures.size();
            statistics->elapsedMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - executionStart).count();

            for (std::size_t workerIndex = 0; workerIndex < state->workers.size(); ++workerIndex)
            {
                if (state->workers[workerIndex].claimedBatchCount > 0)
                {
                    statistics->workers.push_back(state->workers[workerIndex]);
                }
            }
        }

        std::rethrow_exception(submissionException);
    }

    worker(0);

    for (std::size_t futureIndex = 0; futureIndex < futures.size(); ++futureIndex)
    {
        try
        {
            futures[futureIndex].get();
        }
        catch (...)
        {
            state->stopRequested.store(true, std::memory_order_release);

            std::lock_guard<std::mutex> lock(state->exceptionMutex);

            if (!state->firstException)
            {
                state->firstException = std::current_exception();
            }
        }
    }

    std::exception_ptr firstException;

    {
        std::lock_guard<std::mutex> lock(state->exceptionMutex);
        firstException = state->firstException;
    }

    if (statistics)
    {
        statistics->cancellationRequested =
            state->cancellationObserved.load(std::memory_order_acquire);

        statistics->exceptionOccurred = static_cast<bool>(firstException);
        statistics->submittedWorkerTaskCount = futures.size();
        statistics->elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(Clock::now() - executionStart).count();

        for (std::size_t workerIndex = 0; workerIndex < state->workers.size(); ++workerIndex)
        {
            if (state->workers[workerIndex].claimedBatchCount > 0)
            {
                statistics->workers.push_back(state->workers[workerIndex]);
            }
        }
    }

    if (firstException)
    {
        std::rethrow_exception(firstException);
    }
}

}
}

#endif // MYVOXEL_FOUNDATION_PARALLELEXECUTOR_H