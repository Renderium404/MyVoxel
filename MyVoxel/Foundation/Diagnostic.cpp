#include "Diagnostic.h"

#include <cstdio>
#include <cstdlib>

namespace MyVoxel
{
namespace Foundation
{

namespace
{

// 输出统一格式的诊断失败信息。
void writeFailure(const char* failureType, const char* condition, const char* message, const char* file, int line, const char* functionName)
{
    std::fprintf(stderr, "\n[%s]\n", failureType ? failureType : "Failure");
    std::fprintf(stderr, "Condition: %s\n", condition ? condition : "<unknown>");
    std::fprintf(stderr, "File: %s\n", file ? file : "<unknown>");
    std::fprintf(stderr, "Line: %d\n", line);
    std::fprintf(stderr, "Function: %s\n", functionName ? functionName : "<unknown>");

    if (message && message[0] != '\0')
    {
        std::fprintf(stderr, "Message: %s\n", message);
    }

    std::fflush(stderr);
}

}

MYVOXEL_NORETURN void reportAssertionFailure(const char* condition, const char* message, const char* file, int line, const char* functionName)
{
    writeFailure("Assertion failed", condition, message, file, line, functionName);
    std::abort();
}

MYVOXEL_NORETURN void reportRequirementFailure(const char* condition, const char* message, const char* file, int line, const char* functionName)
{
    writeFailure("Requirement failed", condition, message, file, line, functionName);
    std::abort();
}

}
}