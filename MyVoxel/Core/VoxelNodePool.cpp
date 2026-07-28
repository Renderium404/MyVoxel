#include "VoxelNodePool.h"

#include <cassert>
#include <limits>

namespace MyVoxel
{

VoxelNodeGroupIndex VoxelNodePool::allocateNodeGroup(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    if (!m_freeIndexes.empty())
    {
        const VoxelNodeGroupIndex index = m_freeIndexes.back();
        m_freeIndexes.pop_back();

        NodeGroupSlot& slot = m_slots[static_cast<std::size_t>(index)];
        assert(!slot.allocated);

        slot.group.reset(state);
        slot.allocated = true;
        ++m_allocatedGroupCount;

        return index;
    }

    assert(m_slots.size() < static_cast<std::size_t>(InvalidVoxelNodeGroupIndex));

    m_slots.push_back(NodeGroupSlot());

    const VoxelNodeGroupIndex index = static_cast<VoxelNodeGroupIndex>(m_slots.size() - 1);
    NodeGroupSlot& slot = m_slots.back();

    slot.group.reset(state);
    slot.allocated = true;
    ++m_allocatedGroupCount;

    return index;
}

void VoxelNodePool::releaseNodeGroup(VoxelNodeGroupIndex index)
{
    assert(contains(index));
    releaseNodeGroupRecursive(index);
}

VoxelNodeGroup& VoxelNodePool::nodeGroup(VoxelNodeGroupIndex index)
{
    assert(contains(index));
    return m_slots[static_cast<std::size_t>(index)].group;
}

const VoxelNodeGroup& VoxelNodePool::nodeGroup(VoxelNodeGroupIndex index) const
{
    assert(contains(index));
    return m_slots[static_cast<std::size_t>(index)].group;
}

bool VoxelNodePool::contains(VoxelNodeGroupIndex index) const
{
    if (index == InvalidVoxelNodeGroupIndex)
    {
        return false;
    }

    const std::size_t slotIndex = static_cast<std::size_t>(index);

    if (slotIndex >= m_slots.size())
    {
        return false;
    }

    return m_slots[slotIndex].allocated;
}

std::size_t VoxelNodePool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t VoxelNodePool::storageGroupCount() const
{
    return m_slots.size();
}

void VoxelNodePool::clear()
{
    m_slots.clear();
    m_freeIndexes.clear();
    m_allocatedGroupCount = 0;
}

void VoxelNodePool::releaseNodeGroupRecursive(VoxelNodeGroupIndex index)
{
    assert(contains(index));

    NodeGroupSlot& slot = m_slots[static_cast<std::size_t>(index)];

    for (int i = 0; i < VoxelCornerCount; ++i)
    {
        VoxelNode& child = slot.group.child(static_cast<VoxelCorner>(i));

        if (child.state() == VoxelState::Subdivided)
        {
            const VoxelNodeGroupIndex childGroupIndex = child.childGroupIndex();
            assert(contains(childGroupIndex));

            releaseNodeGroupRecursive(childGroupIndex);
            child.setState(VoxelState::Empty);
        }
    }

    slot.group.reset(VoxelState::Empty);
    slot.allocated = false;

    m_freeIndexes.push_back(index);

    assert(m_allocatedGroupCount > 0);
    --m_allocatedGroupCount;
}

}