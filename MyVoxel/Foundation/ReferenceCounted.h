#ifndef MYVOXEL_FOUNDATION_REFERENCECOUNTED_H
#define MYVOXEL_FOUNDATION_REFERENCECOUNTED_H

#include <atomic>
#include <cstddef>

namespace MyVoxel
{
namespace Foundation
{

// 为需要共享生命周期的对象提供线程安全的侵入式引用计数。
class ReferenceCounted
{
public:
    ReferenceCounted(const ReferenceCounted&) = delete;
    ReferenceCounted& operator=(const ReferenceCounted&) = delete;
    ReferenceCounted(ReferenceCounted&&) = delete;
    ReferenceCounted& operator=(ReferenceCounted&&) = delete;

    /// 引用管理
    // 增加一个对象引用。
    void addReference() const;

    // 释放一个对象引用，最后一个引用释放后销毁对象。
    void releaseReference() const;

    // 返回当前引用数量，该结果在并发环境中仅表示调用时刻的瞬时状态。
    std::size_t referenceCount() const;

protected:
    // 构造引用计数为零的对象。
    ReferenceCounted();

    // 通过最后一个引用释放对象。
    virtual ~ReferenceCounted();

private:
    mutable std::atomic<std::size_t> m_referenceCount; // 当前对象的引用数量。
};

}
}

#endif // MYVOXEL_FOUNDATION_REFERENCECOUNTED_H