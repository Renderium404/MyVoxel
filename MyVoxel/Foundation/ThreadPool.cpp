#include "ThreadPool.h"

namespace MyVoxel
{
namespace Foundation
{

ThreadPool::ThreadPool(std::size_t workerCountValue)
    : m_activeTaskCount(0)
    , m_stopping(false)
{
    const std::size_t resolvedWorkerCount = resolveWorkerCount(workerCountValue);

    m_workers.reserve(resolvedWorkerCount);

    try
    {
        for (std::size_t workerIndex = 0; workerIndex < resolvedWorkerCount; ++workerIndex)
        {
            m_workers.push_back(std::thread(&ThreadPool::workerLoop, this));
        }
    }
    catch (...)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stopping = true;
        }

        m_taskCondition.notify_all();

        for (std::size_t workerIndex = 0; workerIndex < m_workers.size(); ++workerIndex)
        {
            if (m_workers[workerIndex].joinable())
            {
                m_workers[workerIndex].join();
            }
        }

        throw;
    }
}

ThreadPool::~ThreadPool()
{
    MYVOXEL_REQUIRE_MESSAGE(!isWorkerThread(), "ThreadPool cannot be destroyed from one of its worker threads.");

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
    }

    m_taskCondition.notify_all();

    for (std::size_t workerIndex = 0; workerIndex < m_workers.size(); ++workerIndex)
    {
        if (m_workers[workerIndex].joinable())
        {
            m_workers[workerIndex].join();
        }
    }
}

/// 线程池状态

std::size_t ThreadPool::workerCount() const
{
    return m_workers.size();
}

std::size_t ThreadPool::hardwareWorkerCount()
{
    const unsigned int workerCountValue = std::thread::hardware_concurrency();
    return workerCountValue > 0 ? static_cast<std::size_t>(workerCountValue) : static_cast<std::size_t>(1);
}

ThreadPool& ThreadPool::global()
{
    // 全局线程池持续到进程结束，避免静态析构阶段等待工作线程产生退出死锁。
    static ThreadPool* pool = new ThreadPool();
    return *pool;
}

bool ThreadPool::isWorkerThread() const
{
    const std::thread::id currentThreadId = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_workerThreadIds.find(currentThreadId) != m_workerThreadIds.end();
}

std::size_t ThreadPool::queuedTaskCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tasks.size();
}

std::size_t ThreadPool::activeTaskCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_activeTaskCount;
}

bool ThreadPool::isIdle() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tasks.empty() && m_activeTaskCount == 0;
}

bool ThreadPool::isStopping() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stopping;
}

/// 任务执行

void ThreadPool::waitIdle()
{
    MYVOXEL_REQUIRE_MESSAGE(!isWorkerThread(), "ThreadPool::waitIdle cannot be called from one of its worker threads.");

    std::unique_lock<std::mutex> lock(m_mutex);

    m_idleCondition.wait(
        lock,
        [this]()
        {
            return m_tasks.empty() && m_activeTaskCount == 0;
        });
}

void ThreadPool::workerLoop()
{
    const std::thread::id currentThreadId = std::this_thread::get_id();

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        const std::pair<std::set<std::thread::id>::iterator, bool> inserted =
            m_workerThreadIds.insert(currentThreadId);

        MYVOXEL_ASSERT_MESSAGE(inserted.second, "ThreadPool worker thread was registered more than once.");
    }

    while (true)
    {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(m_mutex);

            m_taskCondition.wait(
                lock,
                [this]()
                {
                    return m_stopping || !m_tasks.empty();
                });

            if (m_stopping && m_tasks.empty())
            {
                break;
            }

            task = std::move(m_tasks.front());
            m_tasks.pop();
            ++m_activeTaskCount;
        }

        // submit使用packaged_task包装任务，用户异常由对应future保存，不会终止工作线程。
        task();

        {
            std::lock_guard<std::mutex> lock(m_mutex);

            MYVOXEL_ASSERT_MESSAGE(m_activeTaskCount > 0, "ThreadPool active task count underflow.");

            --m_activeTaskCount;

            if (m_tasks.empty() && m_activeTaskCount == 0)
            {
                m_idleCondition.notify_all();
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        const std::size_t removedCount = m_workerThreadIds.erase(currentThreadId);

        MYVOXEL_ASSERT_MESSAGE(removedCount == 1, "ThreadPool worker thread was not registered.");
    }
}

std::size_t ThreadPool::resolveWorkerCount(std::size_t requestedWorkerCount)
{
    return requestedWorkerCount > 0 ? requestedWorkerCount : hardwareWorkerCount();
}

}
}