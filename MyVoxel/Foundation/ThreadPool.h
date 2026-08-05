#ifndef MYVOXEL_FOUNDATION_THREADPOOL_H
#define MYVOXEL_FOUNDATION_THREADPOOL_H

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "Diagnostic.h"

namespace MyVoxel
{
namespace Foundation
{

// 提供固定工作线程复用和异步任务提交。
class ThreadPool
{
public:
    // 创建固定大小的线程池，workerCount为0时使用硬件并发数，无法获取时使用一个工作线程。
    explicit ThreadPool(std::size_t workerCount = 0);

    // 完成已经提交的任务并停止全部工作线程。
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    /// 线程池状态

    // 返回线程池持有的固定工作线程数量。
    std::size_t workerCount() const;

    // 返回系统报告的硬件并发线程数量，无法获取时返回1。
    static std::size_t hardwareWorkerCount();

    // 返回进程内共享的默认线程池。
    static ThreadPool& global();

    // 判断当前调用线程是否属于该线程池。
    bool isWorkerThread() const;

    // 返回当前等待执行的任务数量。
    std::size_t queuedTaskCount() const;

    // 返回当前正在执行的任务数量。
    std::size_t activeTaskCount() const;

    // 判断当前任务队列和活动任务是否都为空。
    bool isIdle() const;

    // 判断线程池是否已经开始停止。
    bool isStopping() const;

    /// 任务执行

    // 提交一个异步任务并返回其结果，任务异常通过future传播。
    template<typename Function, typename... Arguments>
    auto submit(Function&& function, Arguments&&... arguments) -> std::future<typename std::result_of<Function(Arguments...)>::type>;

    // 等待任务队列和活动任务全部清空，不允许在线程池自身的工作线程中调用。
    void waitIdle();

private:
    // 运行一个固定工作线程的任务循环。
    void workerLoop();

    // 解析线程池实际创建的工作线程数量。
    static std::size_t resolveWorkerCount(std::size_t requestedWorkerCount);

private:
    std::vector<std::thread> m_workers; // 固定工作线程。
    std::queue<std::function<void()>> m_tasks; // 等待执行的任务队列。
    std::set<std::thread::id> m_workerThreadIds; // 当前线程池已经注册的工作线程标识。
    mutable std::mutex m_mutex; // 保护任务队列、线程标识和线程池状态。
    std::condition_variable m_taskCondition; // 通知工作线程存在新任务或线程池开始停止。
    std::condition_variable m_idleCondition; // 通知等待线程当前线程池已经空闲。
    std::size_t m_activeTaskCount; // 当前正在执行的任务数量。
    bool m_stopping; // 当前线程池是否已经开始停止。
};

template<typename Function, typename... Arguments>
auto ThreadPool::submit(Function&& function, Arguments&&... arguments) -> std::future<typename std::result_of<Function(Arguments...)>::type>
{
    typedef typename std::result_of<Function(Arguments...)>::type Result;

    std::shared_ptr<std::packaged_task<Result()>> task =
        std::make_shared<std::packaged_task<Result()>>(
            std::bind(
                std::forward<Function>(function),
                std::forward<Arguments>(arguments)...));

    std::future<Result> future = task->get_future();

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        MYVOXEL_REQUIRE_MESSAGE(!m_stopping, "Cannot submit a task to a stopping ThreadPool.");

        m_tasks.push([task]()
        {
            (*task)();
        });
    }

    m_taskCondition.notify_one();
    return future;
}

}
}

#endif // MYVOXEL_FOUNDATION_THREADPOOL_H