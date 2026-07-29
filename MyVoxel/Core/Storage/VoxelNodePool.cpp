#include "VoxelNodePool.h"

#include <cstddef>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelNodePool::NodeGroupSlot::NodeGroupSlot()
    : allocated(false)
{
}

VoxelNodePool::VoxelNodePool()
    : m_allocatedGroupCount(0)
{
}

/// 节点组管理

VoxelNodeGroupIndex VoxelNodePool::allocateNodeGroup(VoxelState state)
{
    MYVOXEL_ASSERT_MESSAGE(state == VoxelState::Empty || state == VoxelState::Material, "VoxelNodePool allocation state must be Empty or Material.");

    if (!m_freeIndexes.empty())
    {
        const VoxelNodeGroupIndex index = m_freeIndexes.back();
        m_freeIndexes.pop_back();

        NodeGroupSlot& slot = m_slots[static_cast<std::size_t>(index)];

        MYVOXEL_ASSERT_MESSAGE(!slot.allocated, "A reusable VoxelNodePool slot must not already be allocated.");

        slot.group.reset(state);
        slot.allocated = true;
        ++m_allocatedGroupCount;
        return index;
    }

    MYVOXEL_ASSERT_MESSAGE(m_slots.size() < static_cast<std::size_t>(InvalidVoxelNodeGroupIndex), "VoxelNodePool has exhausted the VoxelNodeGroupIndex range.");

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
    MYVOXEL_ASSERT_MESSAGE(contains(index), "VoxelNodePool can only release an allocated node group.");
    releaseNodeGroupRecursive(index);
}

void VoxelNodePool::clear()
{
    m_slots.clear();
    m_freeIndexes.clear();
    m_allocatedGroupCount = 0;
}

/// 节点组访问

VoxelNodeGroup& VoxelNodePool::nodeGroup(VoxelNodeGroupIndex index)
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "VoxelNodePool index must reference an allocated node group.");
    return m_slots[static_cast<std::size_t>(index)].group;
}

const VoxelNodeGroup& VoxelNodePool::nodeGroup(VoxelNodeGroupIndex index) const
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "VoxelNodePool index must reference an allocated node group.");
    return m_slots[static_cast<std::size_t>(index)].group;
}

/// 状态与统计

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

bool VoxelNodePool::isEmpty() const
{
    return m_allocatedGroupCount == 0;
}

std::size_t VoxelNodePool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t VoxelNodePool::storageGroupCount() const
{
    return m_slots.size();
}

std::size_t VoxelNodePool::freeGroupCount() const
{
    return m_freeIndexes.size();
}

void VoxelNodePool::releaseNodeGroupRecursive(VoxelNodeGroupIndex index)
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "VoxelNodePool recursive release requires an allocated node group.");

    NodeGroupSlot& slot = m_slots[static_cast<std::size_t>(index)];

    for (std::size_t childIndex = 0; childIndex < VoxelCornerCount; ++childIndex)
    {
        VoxelNode& child = slot.group.child(static_cast<VoxelCorner>(childIndex));

        MYVOXEL_ASSERT_MESSAGE(child.isValid(), "VoxelNodePool cannot release a node group containing an invalid child node.");

        if (!child.isSubdivided())
        {
            continue;
        }

        const VoxelNodeGroupIndex childGroupIndex = child.childGroupIndex();

        MYVOXEL_ASSERT_MESSAGE(contains(childGroupIndex), "A subdivided VoxelNode must reference an allocated child node group.");

        releaseNodeGroupRecursive(childGroupIndex);
        child.setState(VoxelState::Empty);
    }

    slot.group.reset(VoxelState::Empty);
    slot.allocated = false;
    m_freeIndexes.push_back(index);

    MYVOXEL_ASSERT_MESSAGE(m_allocatedGroupCount > 0, "VoxelNodePool allocated group count cannot underflow.");
    --m_allocatedGroupCount;
}

}