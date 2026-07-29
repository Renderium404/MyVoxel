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
    const std::size_t previousCount = m_referenceCount.fetch_add(1, std::memory_order_relaxed);
    MYVOXEL_REQUIRE_MESSAGE(previousCount < std::numeric_limits<std::size_t>::max(), "ReferenceCounted reference count overflow.");
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