#ifndef MYVOXEL_FOUNDATION_REFPTR_H
#define MYVOXEL_FOUNDATION_REFPTR_H

#include <cstddef>
#include <type_traits>
#include <utility>

#include "Diagnostic.h"

namespace MyVoxel
{
namespace Foundation
{

// 管理继承自ReferenceCounted对象的侵入式共享引用。
template<typename T>
class RefPtr
{
    template<typename U>
    friend class RefPtr;

public:
    /// 构造与析构
    // 构造空引用指针。
    RefPtr()
        : m_pointer(nullptr)
    {
    }
    // 构造空引用指针。
    RefPtr(std::nullptr_t)
        : m_pointer(nullptr)
    {
    }
    // 接管一个对象引用并增加其引用计数，pointer可以为空。
    explicit RefPtr(T* pointer)
        : m_pointer(pointer)
    {
        addReference();
    }
    // 复制同类型引用指针并增加引用计数。
    RefPtr(const RefPtr& other)
        : m_pointer(other.m_pointer)
    {
        addReference();
    }

    // 复制可转换类型的引用指针并增加引用计数。
    template<typename U>
    RefPtr(const RefPtr<U>& other, typename std::enable_if<std::is_convertible<U*, T*>::value>::type* = nullptr)
        : m_pointer(other.m_pointer)
    {
        addReference();
    }

    // 移动同类型引用指针而不改变引用计数。
    RefPtr(RefPtr&& other) 
        : m_pointer(other.m_pointer)
    {
        other.m_pointer = nullptr;
    }

    // 移动可转换类型的引用指针而不改变引用计数。
    template<typename U>
    RefPtr(RefPtr<U>&& other, typename std::enable_if<std::is_convertible<U*, T*>::value>::type* = nullptr) 
        : m_pointer(other.m_pointer)
    {
        other.m_pointer = nullptr;
    }

    // 释放当前对象引用。
    ~RefPtr()
    {
        releaseReference();
    }

    /// 赋值
    // 通过复制或移动临时对象完成同类型赋值。
    RefPtr& operator=(RefPtr other)
    {
        swap(other);
        return *this;
    }
    // 复制可转换类型的引用指针。
    template<typename U>
    typename std::enable_if<std::is_convertible<U*, T*>::value, RefPtr&>::type operator=(const RefPtr<U>& other)
    {
        RefPtr temporary(other);
        swap(temporary);
        return *this;
    }

    // 移动可转换类型的引用指针。
    template<typename U>
    typename std::enable_if<std::is_convertible<U*, T*>::value, RefPtr&>::type operator=(RefPtr<U>&& other)
    {
        RefPtr temporary(std::move(other));
        swap(temporary);
        return *this;
    }

    // 清除当前对象引用。
    RefPtr& operator=(std::nullptr_t)
    {
        reset();
        return *this;
    }

    /// 状态判断

    // 判断当前引用指针是否为空。
    bool isNull() const
    {
        return m_pointer == nullptr;
    }

    // 判断当前引用指针是否持有对象。
    explicit operator bool() const
    {
        return m_pointer != nullptr;
    }

    // 判断当前引用指针是否为空。
    bool operator!() const
    {
        return m_pointer == nullptr;
    }

    /// 对象访问

    // 返回当前对象的普通指针。
    T* get() const
    {
        return m_pointer;
    }

    // 返回当前对象引用，空指针调用属于非法操作。
    T& operator*() const
    {
        MYVOXEL_REQUIRE_MESSAGE(m_pointer, "Cannot dereference a null RefPtr.");
        return *m_pointer;
    }

    // 返回当前对象指针，空指针调用属于非法操作。
    T* operator->() const
    {
        MYVOXEL_REQUIRE_MESSAGE(m_pointer, "Cannot access a null RefPtr.");
        return m_pointer;
    }

    /// 引用修改
    // 将当前引用替换为指定对象，pointer可以为空。
    void reset(T* pointer = nullptr)
    {
        if (m_pointer == pointer)
        {
            return;
        }

        if (pointer)
        {
            pointer->addReference();
        }

        T* previousPointer = m_pointer;
        m_pointer = pointer;

        if (previousPointer)
        {
            previousPointer->releaseReference();
        }
    }

    // 与另一个引用指针交换对象引用。
    void swap(RefPtr& other) 
    {
        T* pointer = m_pointer;
        m_pointer = other.m_pointer;
        other.m_pointer = pointer;
    }

private:
    // 增加当前对象的引用计数。
    void addReference()
    {
        if (m_pointer)
        {
            m_pointer->addReference();
        }
    }

    // 释放当前对象引用并清空指针。
    void releaseReference()
    {
        T* pointer = m_pointer;
        m_pointer = nullptr;

        if (pointer)
        {
            pointer->releaseReference();
        }
    }

private:
    T* m_pointer; // 当前持有引用的对象指针。
};

/// 引用指针创建

// 创建一个侵入式引用计数对象。
template<typename T, typename... Arguments>
RefPtr<T> makeRef(Arguments&&... arguments)
{
    return RefPtr<T>(new T(std::forward<Arguments>(arguments)...));
}

/// 引用指针比较
// 判断两个引用指针是否指向同一个对象。
template<typename T, typename U>
bool operator==(const RefPtr<T>& first, const RefPtr<U>& second)
{
    return static_cast<const void*>(first.get()) == static_cast<const void*>(second.get());
}

// 判断两个引用指针是否指向不同对象。
template<typename T, typename U>
bool operator!=(const RefPtr<T>& first, const RefPtr<U>& second)
{
    return !(first == second);
}

// 判断引用指针是否为空。
template<typename T>
bool operator==(const RefPtr<T>& pointer, std::nullptr_t)
{
    return pointer.get() == nullptr;
}

// 判断引用指针是否为空。
template<typename T>
bool operator==(std::nullptr_t, const RefPtr<T>& pointer)
{
    return pointer.get() == nullptr;
}

// 判断引用指针是否非空。
template<typename T>
bool operator!=(const RefPtr<T>& pointer, std::nullptr_t)
{
    return pointer.get() != nullptr;
}

// 判断引用指针是否非空。
template<typename T>
bool operator!=(std::nullptr_t, const RefPtr<T>& pointer)
{
    return pointer.get() != nullptr;
}

/// 引用指针交换

// 交换两个引用指针持有的对象引用。
template<typename T>
void swap(RefPtr<T>& first, RefPtr<T>& second) 
{
    first.swap(second);
}

}
}

#endif // MYVOXEL_FOUNDATION_REFPTR_H