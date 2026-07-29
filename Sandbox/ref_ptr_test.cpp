#include <atomic>
#include <cstddef>
#include <iostream>
#include <utility>

#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace
{

// 提供引用指针测试使用的引用计数对象。
class TestObject : public MyVoxel::Foundation::ReferenceCounted
{
public:
    // 创建指定数值的测试对象。
    explicit TestObject(int value = 0)
        : m_value(value)
    {
        ++s_aliveCount;
    }

    // 返回测试对象数值。
    int value() const
    {
        return m_value;
    }

    // 设置测试对象数值。
    void setValue(int value)
    {
        m_value = value;
    }

    // 清除对象生命周期统计。
    static void resetStatistics()
    {
        s_aliveCount.store(0);
        s_destroyedCount.store(0);
    }

    // 返回当前存活对象数量。
    static int aliveCount()
    {
        return s_aliveCount.load();
    }

    // 返回已经销毁的对象数量。
    static int destroyedCount()
    {
        return s_destroyedCount.load();
    }

protected:
    // 记录测试对象销毁。
    ~TestObject() override
    {
        --s_aliveCount;
        ++s_destroyedCount;
    }

private:
    int m_value; // 当前测试对象数值。

    static std::atomic<int> s_aliveCount; // 当前存活对象数量。
    static std::atomic<int> s_destroyedCount; // 已经销毁的对象数量。
};

std::atomic<int> TestObject::s_aliveCount(0);
std::atomic<int> TestObject::s_destroyedCount(0);

// 提供类型转换测试使用的派生对象。
class DerivedTestObject : public TestObject
{
public:
    // 创建指定数值和附加数值的派生测试对象。
    DerivedTestObject(int value, int extraValue)
        : TestObject(value)
        , m_extraValue(extraValue)
    {
    }

    // 返回派生对象附加数值。
    int extraValue() const
    {
        return m_extraValue;
    }

protected:
    ~DerivedTestObject() override = default;

private:
    int m_extraValue; // 当前派生对象附加数值。
};

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 测试默认引用指针状态。
bool testDefaultState()
{
    const MyVoxel::Foundation::RefPtr<TestObject> first;
    const MyVoxel::Foundation::RefPtr<TestObject> second(nullptr);

    const bool passed =
        first.isNull() &&
        second.isNull() &&
        !first &&
        !second &&
        first == nullptr &&
        nullptr == second &&
        first == second;

    return check(passed, "RefPtr default state");
}

// 测试使用普通指针创建引用。
bool testRawPointerConstruction()
{
    TestObject::resetStatistics();

    bool passed = false;

    {
        MyVoxel::Foundation::RefPtr<TestObject> pointer(new TestObject(12));

        passed =
            pointer &&
            pointer.get() != nullptr &&
            pointer->value() == 12 &&
            (*pointer).value() == 12 &&
            pointer->referenceCount() == 1 &&
            TestObject::aliveCount() == 1 &&
            TestObject::destroyedCount() == 0;
    }

    passed =
        passed &&
        TestObject::aliveCount() == 0 &&
        TestObject::destroyedCount() == 1;

    return check(passed, "RefPtr raw pointer construction");
}

// 测试引用指针复制。
bool testCopyConstruction()
{
    TestObject::resetStatistics();

    bool passed = false;

    {
        MyVoxel::Foundation::RefPtr<TestObject> first = MyVoxel::Foundation::makeRef<TestObject>(21);

        {
            MyVoxel::Foundation::RefPtr<TestObject> second(first);
            MyVoxel::Foundation::RefPtr<TestObject> third = second;

            passed =
                first == second &&
                second == third &&
                first->referenceCount() == 3 &&
                TestObject::aliveCount() == 1;
        }

        passed =
            passed &&
            first->referenceCount() == 1 &&
            TestObject::aliveCount() == 1;
    }

    passed =
        passed &&
        TestObject::aliveCount() == 0 &&
        TestObject::destroyedCount() == 1;

    return check(passed, "RefPtr copy construction");
}

// 测试引用指针移动。
bool testMoveConstruction()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> first = MyVoxel::Foundation::makeRef<TestObject>(32);
    TestObject* rawPointer = first.get();

    MyVoxel::Foundation::RefPtr<TestObject> second(std::move(first));

    const bool passed =
        !first &&
        second &&
        second.get() == rawPointer &&
        second->referenceCount() == 1 &&
        TestObject::aliveCount() == 1;

    second.reset();

    return check(passed && TestObject::destroyedCount() == 1, "RefPtr move construction");
}

// 测试复制赋值和旧对象释放。
bool testCopyAssignment()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> first = MyVoxel::Foundation::makeRef<TestObject>(10);
    MyVoxel::Foundation::RefPtr<TestObject> second = MyVoxel::Foundation::makeRef<TestObject>(20);

    second = first;

    const bool passed =
        first == second &&
        first->referenceCount() == 2 &&
        TestObject::aliveCount() == 1 &&
        TestObject::destroyedCount() == 1;

    first.reset();
    second.reset();

    return check(passed && TestObject::destroyedCount() == 2, "RefPtr copy assignment");
}

// 测试移动赋值和旧对象释放。
bool testMoveAssignment()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> first = MyVoxel::Foundation::makeRef<TestObject>(10);
    MyVoxel::Foundation::RefPtr<TestObject> second = MyVoxel::Foundation::makeRef<TestObject>(20);
    TestObject* firstPointer = first.get();

    second = std::move(first);

    const bool passed =
        !first &&
        second.get() == firstPointer &&
        second->referenceCount() == 1 &&
        TestObject::aliveCount() == 1 &&
        TestObject::destroyedCount() == 1;

    second.reset();

    return check(passed && TestObject::destroyedCount() == 2, "RefPtr move assignment");
}

// 测试自赋值和自移动赋值。
bool testSelfAssignment()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> pointer = MyVoxel::Foundation::makeRef<TestObject>(48);
    TestObject* rawPointer = pointer.get();

    pointer = pointer;

    const bool copyPassed =
        pointer.get() == rawPointer &&
        pointer->referenceCount() == 1;

    pointer = std::move(pointer);

    const bool movePassed =
        pointer.get() == rawPointer &&
        pointer->referenceCount() == 1;

    pointer.reset();

    return check(copyPassed && movePassed && TestObject::destroyedCount() == 1, "RefPtr self assignment");
}

// 测试reset替换和清除对象引用。
bool testReset()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> pointer = MyVoxel::Foundation::makeRef<TestObject>(1);
    TestObject* samePointer = pointer.get();

    pointer.reset(samePointer);

    const bool samePassed =
        pointer.get() == samePointer &&
        pointer->referenceCount() == 1 &&
        TestObject::destroyedCount() == 0;

    pointer.reset(new TestObject(2));

    const bool replacePassed =
        pointer->value() == 2 &&
        pointer->referenceCount() == 1 &&
        TestObject::aliveCount() == 1 &&
        TestObject::destroyedCount() == 1;

    pointer = nullptr;

    const bool clearPassed =
        !pointer &&
        TestObject::aliveCount() == 0 &&
        TestObject::destroyedCount() == 2;

    return check(samePassed && replacePassed && clearPassed, "RefPtr reset");
}

// 测试派生类型到基类和常量类型的转换。
bool testTypeConversion()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<DerivedTestObject> derived = MyVoxel::Foundation::makeRef<DerivedTestObject>(64, 128);
    MyVoxel::Foundation::RefPtr<TestObject> base = derived;
    MyVoxel::Foundation::RefPtr<const TestObject> constant = base;

    const bool passed =
        derived &&
        base &&
        constant &&
        derived == base &&
        base == constant &&
        derived->value() == 64 &&
        derived->extraValue() == 128 &&
        constant->value() == 64 &&
        derived->referenceCount() == 3;

    derived.reset();
    base.reset();

    const bool constantPassed =
        constant &&
        constant->referenceCount() == 1 &&
        TestObject::aliveCount() == 1;

    constant.reset();

    return check(passed && constantPassed && TestObject::destroyedCount() == 1, "RefPtr type conversion");
}

// 测试可转换类型的复制和移动赋值。
bool testConvertingAssignment()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<DerivedTestObject> derived = MyVoxel::Foundation::makeRef<DerivedTestObject>(7, 9);
    MyVoxel::Foundation::RefPtr<TestObject> base;
    MyVoxel::Foundation::RefPtr<const TestObject> constant;

    base = derived;
    constant = std::move(base);

    const bool passed =
        derived &&
        !base &&
        constant &&
        derived == constant &&
        derived->referenceCount() == 2;

    derived.reset();
    constant.reset();

    return check(passed && TestObject::destroyedCount() == 1, "RefPtr converting assignment");
}

// 测试引用指针交换。
bool testSwap()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> first = MyVoxel::Foundation::makeRef<TestObject>(1);
    MyVoxel::Foundation::RefPtr<TestObject> second = MyVoxel::Foundation::makeRef<TestObject>(2);

    TestObject* firstPointer = first.get();
    TestObject* secondPointer = second.get();

    swap(first, second);

    const bool passed =
        first.get() == secondPointer &&
        second.get() == firstPointer &&
        first->value() == 2 &&
        second->value() == 1 &&
        first->referenceCount() == 1 &&
        second->referenceCount() == 1;

    first.reset();
    second.reset();

    return check(passed && TestObject::destroyedCount() == 2, "RefPtr swap");
}

// 测试多线程复制和释放独立引用指针。
bool testConcurrentReferences()
{
    TestObject::resetStatistics();

    MyVoxel::Foundation::RefPtr<TestObject> root = MyVoxel::Foundation::makeRef<TestObject>(256);
    MyVoxel::Foundation::ParallelExecutor executor(8);

    std::atomic<std::size_t> successfulQueries(0);

    executor.forEach(2000, [&](std::size_t)
    {
        MyVoxel::Foundation::RefPtr<TestObject> first = root;
        MyVoxel::Foundation::RefPtr<const TestObject> second = first;

        if (first->value() == 256 && second->value() == 256)
        {
            successfulQueries.fetch_add(1);
        }
    });

    const bool passed =
        successfulQueries.load() == 2000 &&
        root->referenceCount() == 1 &&
        TestObject::aliveCount() == 1 &&
        TestObject::destroyedCount() == 0;

    root.reset();

    return check(passed && TestObject::aliveCount() == 0 && TestObject::destroyedCount() == 1, "RefPtr concurrent references");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testRawPointerConstruction(),
        testCopyConstruction(),
        testMoveConstruction(),
        testCopyAssignment(),
        testMoveAssignment(),
        testSelfAssignment(),
        testReset(),
        testTypeConversion(),
        testConvertingAssignment(),
        testSwap(),
        testConcurrentReferences()
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
    std::cout << "RefPtr tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}