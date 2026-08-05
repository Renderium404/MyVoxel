#include "ReferenceCounted.h"

#include <limits>

#include "Diagnostic.h"

namespace MyVoxel
{
namespace Foundation
{

ReferenceCounted::ReferenceCounted()
    : m_referenceCount(0)
{
}

ReferenceCounted::~ReferenceCounted()
{
    MYVOXEL_ASSERT_MESSAGE(m_referenceCount.load(std::memory_order_relaxed) == 0, "ReferenceCounted object must be destroyed with zero references.");
}

/// 引用管理

void ReferenceCounted::addReference() const
{
    std::size_t currentCount = m_referenceCount.load(std::memory_order_relaxed);

    while (true)
    {
        MYVOXEL_REQUIRE_MESSAGE(currentCount < std::numeric_limits<std::size_t>::max(), "ReferenceCounted reference count overflow.");

        if (m_referenceCount.compare_exchange_weak(currentCount, currentCount + 1, std::memory_order_relaxed, std::memory_order_relaxed))
        {
            return;
        }
    }
}

void ReferenceCounted::releaseReference() const
{
    std::size_t currentCount = m_referenceCount.load(std::memory_order_acquire);

    while (true)
    {
        MYVOXEL_REQUIRE_MESSAGE(currentCount > 0, "ReferenceCounted reference count underflow.");

        if (m_referenceCount.compare_exchange_weak(currentCount, currentCount - 1, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            break;
        }
    }

    if (currentCount == 1)
    {
        delete this;
    }
}

std::size_t ReferenceCounted::referenceCount() const
{
    return m_referenceCount.load(std::memory_order_acquire);
}

}
}