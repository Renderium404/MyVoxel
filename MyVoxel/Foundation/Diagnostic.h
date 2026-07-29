#ifndef MYVOXEL_FOUNDATION_DIAGNOSTIC_H
#define MYVOXEL_FOUNDATION_DIAGNOSTIC_H

#if defined(_MSC_VER)
#define MYVOXEL_NORETURN __declspec(noreturn)
#define MYVOXEL_FUNCTION_NAME __FUNCTION__
#else
#define MYVOXEL_NORETURN [[noreturn]]
#define MYVOXEL_FUNCTION_NAME __func__
#endif

namespace MyVoxel
{
namespace Foundation
{

/// 诊断失败处理

// 输出断言失败信息并终止程序。
MYVOXEL_NORETURN void reportAssertionFailure(const char* condition, const char* message, const char* file, int line, const char* functionName);

// 输出必要条件失败信息并终止程序。
MYVOXEL_NORETURN void reportRequirementFailure(const char* condition, const char* message, const char* file, int line, const char* functionName);

}
}

#if defined(NDEBUG)

#define MYVOXEL_ASSERT(condition) do { (void)sizeof(condition); } while (false)
#define MYVOXEL_ASSERT_MESSAGE(condition, message) do { (void)sizeof(condition); } while (false)

#else

#define MYVOXEL_ASSERT(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            MyVoxel::Foundation::reportAssertionFailure(#condition, nullptr, __FILE__, __LINE__, MYVOXEL_FUNCTION_NAME); \
        } \
    } while (false)

#define MYVOXEL_ASSERT_MESSAGE(condition, message) \
    do \
    { \
        if (!(condition)) \
        { \
            MyVoxel::Foundation::reportAssertionFailure(#condition, message, __FILE__, __LINE__, MYVOXEL_FUNCTION_NAME); \
        } \
    } while (false)

#endif

#define MYVOXEL_REQUIRE(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            MyVoxel::Foundation::reportRequirementFailure(#condition, nullptr, __FILE__, __LINE__, MYVOXEL_FUNCTION_NAME); \
        } \
    } while (false)

#define MYVOXEL_REQUIRE_MESSAGE(condition, message) \
    do \
    { \
        if (!(condition)) \
        { \
            MyVoxel::Foundation::reportRequirementFailure(#condition, message, __FILE__, __LINE__, MYVOXEL_FUNCTION_NAME); \
        } \
    } while (false)

#define MYVOXEL_UNUSED(value) ((void)(value))

#endif // MYVOXEL_FOUNDATION_DIAGNOSTIC_H