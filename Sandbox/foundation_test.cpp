#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "MyVoxel/Foundation/Logger.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/Stopwatch.h"

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 测试日志等级过滤和自定义输出目标。
bool testLogger()
{
    std::atomic<int> receivedCount(0);
    std::string receivedMessage;

    MyVoxel::Foundation::Logger::setMinimumLevel(MyVoxel::Foundation::LogLevel::Warning);
    MyVoxel::Foundation::Logger::setSink([&](const MyVoxel::Foundation::LogRecord& record)
    {
        ++receivedCount;
        receivedMessage = record.message;
    });

    MYVOXEL_LOG_INFO("Ignored message");
    MYVOXEL_LOG_ERROR("Accepted message");

    const bool passed = receivedCount.load() == 1 && receivedMessage == "Accepted message";

    MyVoxel::Foundation::Logger::resetSink();
    MyVoxel::Foundation::Logger::setMinimumLevel(MyVoxel::Foundation::LogLevel::Info);

    return check(passed, "Logger level and sink");
}

// 测试稳定时钟计时。
bool testStopwatch()
{
    MyVoxel::Foundation::Stopwatch stopwatch;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const double elapsed = stopwatch.elapsedMilliseconds();
    return check(elapsed >= 1.0, "Stopwatch elapsed time");
}

// 测试并行循环是否完整执行全部任务。
bool testParallelFor()
{
    const std::size_t taskCount = 1000;
    const long long expectedSum = 499500;

    std::atomic<long long> sum(0);

    MyVoxel::Foundation::ParallelExecutor executor(4);
    MyVoxel::Foundation::ParallelForOptions options;
    options.minimumParallelTaskCount = 2;

    executor.forEach(taskCount, [&](std::size_t taskIndex)
    {
        sum.fetch_add(static_cast<long long>(taskIndex));
    }, options);

    return check(sum.load() == expectedSum, "ParallelExecutor task coverage");
}

// 测试单任务时是否自动回退到单线程。
bool testSerialFallback()
{
    MyVoxel::Foundation::ParallelExecutor executor(8);
    MyVoxel::Foundation::ParallelForOptions options;
    options.minimumParallelTaskCount = 4;

    const std::size_t workerCount = executor.effectiveWorkerCount(3, options);
    return check(workerCount == 1, "ParallelExecutor serial fallback");
}

// 测试工作线程异常是否返回调用线程。
bool testParallelException()
{
    MyVoxel::Foundation::ParallelExecutor executor(4);

    bool exceptionReceived = false;

    try
    {
        executor.forEach(32, [](std::size_t taskIndex)
        {
            if (taskIndex == 7)
            {
                throw std::runtime_error("parallel failure");
            }
        });
    }
    catch (const std::runtime_error&)
    {
        exceptionReceived = true;
    }

    return check(exceptionReceived, "ParallelExecutor exception propagation");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testLogger(),
        testStopwatch(),
        testParallelFor(),
        testSerialFallback(),
        testParallelException()
    };

    const std::size_t testCount = sizeof(results) / sizeof(results[0]);

    for (std::size_t testIndex = 0; testIndex < testCount; ++testIndex)
    {
        if (results[testIndex])
        {
            ++passedCount;
        }
        else
        {
            ++failedCount;
        }
    }

    std::cout << std::endl;
    std::cout << "Foundation tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}