#include "Logger.h"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace MyVoxel
{
namespace Foundation
{

namespace
{

// 返回日志配置锁。
std::mutex& loggerStateMutex()
{
    static std::mutex mutex;
    return mutex;
}

// 返回默认输出锁，避免多线程日志相互交错。
std::mutex& loggerOutputMutex()
{
    static std::mutex mutex;
    return mutex;
}

// 返回最低日志等级存储。
LogLevel& minimumLevelStorage()
{
    static LogLevel level = LogLevel::Info;
    return level;
}

// 返回源文件路径中的文件名部分。
const char* sourceFileName(const char* file)
{
    if (!file)
    {
        return nullptr;
    }

    const char* result = file;

    for (const char* current = file; *current != '\0'; ++current)
    {
        if (*current == '/' || *current == '\\')
        {
            result = current + 1;
        }
    }

    return result;
}

// 将系统时间转换为本地时间。
std::tm localTime(std::time_t value)
{
    std::tm result = {};

#if defined(_MSC_VER)
    localtime_s(&result, &value);
#else
    localtime_r(&value, &result);
#endif

    return result;
}

// 使用标准错误流输出一条日志。
void defaultLogSink(const LogRecord& record)
{
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(record.timestamp);
    const std::tm timeParts = localTime(timeValue);
    const long long totalMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(record.timestamp.time_since_epoch()).count();
    const int milliseconds = static_cast<int>(totalMilliseconds % 1000);

    std::lock_guard<std::mutex> lock(loggerOutputMutex());

    std::cerr << '['
              << std::setfill('0')
              << std::setw(2) << timeParts.tm_hour << ':'
              << std::setw(2) << timeParts.tm_min << ':'
              << std::setw(2) << timeParts.tm_sec << '.'
              << std::setw(3) << milliseconds
              << "] [" << Logger::levelName(record.level) << ']'
              << " [Thread " << record.threadId << "] "
              << record.message;

    if (record.file)
    {
        std::cerr << " (" << sourceFileName(record.file) << ':' << record.line << ')';
    }

    std::cerr << std::endl;
}

// 返回日志输出目标存储。
LogSink& logSinkStorage()
{
    static LogSink sink = defaultLogSink;
    return sink;
}

}

void Logger::setMinimumLevel(LogLevel level)
{
    std::lock_guard<std::mutex> lock(loggerStateMutex());
    minimumLevelStorage() = level;
}

LogLevel Logger::minimumLevel()
{
    std::lock_guard<std::mutex> lock(loggerStateMutex());
    return minimumLevelStorage();
}

bool Logger::isEnabled(LogLevel level)
{
    std::lock_guard<std::mutex> lock(loggerStateMutex());
    return static_cast<int>(level) >= static_cast<int>(minimumLevelStorage());
}

void Logger::setSink(const LogSink& sink)
{
    std::lock_guard<std::mutex> lock(loggerStateMutex());
    logSinkStorage() = sink ? sink : LogSink(defaultLogSink);
}

void Logger::resetSink()
{
    std::lock_guard<std::mutex> lock(loggerStateMutex());
    logSinkStorage() = defaultLogSink;
}

void Logger::write(LogLevel level, const std::string& message, const char* file, int line)
{
    LogSink sink;

    {
        std::lock_guard<std::mutex> lock(loggerStateMutex());

        if (static_cast<int>(level) < static_cast<int>(minimumLevelStorage()))
        {
            return;
        }

        sink = logSinkStorage();
    }

    LogRecord record;
    record.level = level;
    record.message = message;
    record.file = file;
    record.line = line;
    record.timestamp = std::chrono::system_clock::now();
    record.threadId = std::this_thread::get_id();

    try
    {
        sink(record);
    }
    catch (...)
    {
        defaultLogSink(record);
    }
}

const char* Logger::levelName(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug:
        return "Debug";

    case LogLevel::Info:
        return "Info";

    case LogLevel::Warning:
        return "Warning";

    case LogLevel::Error:
        return "Error";

    case LogLevel::Fatal:
        return "Fatal";
    }

    return "Unknown";
}

}
}