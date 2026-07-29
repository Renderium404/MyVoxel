#include "VoxelNodeGroup.h"

#include <cstddef>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 将体素角点转换为节点组数组索引。
std::size_t cornerIndex(MyVoxel::VoxelCorner corner)
{
    const std::size_t index = static_cast<std::size_t>(corner);

    MYVOXEL_ASSERT_MESSAGE(index < MyVoxel::VoxelCornerCount, "VoxelCorner value must be in range [0, 7].");
    return index;
}

}

namespace MyVoxel
{

VoxelNodeGroup::VoxelNodeGroup()
{
}

VoxelNodeGroup::VoxelNodeGroup(VoxelState state)
{
    reset(state);
}

/// 子节点访问

VoxelNode& VoxelNodeGroup::child(VoxelCorner corner)
{
    return m_children[cornerIndex(corner)];
}

const VoxelNode& VoxelNodeGroup::child(VoxelCorner corner) const
{
    return m_children[cornerIndex(corner)];
}

/// 节点组状态

bool VoxelNodeGroup::isValid() const
{
    for (std::size_t index = 0; index < m_children.size(); ++index)
    {
        if (!m_children[index].isValid())
        {
            return false;
        }
    }

    return true;
}

bool VoxelNodeGroup::canMerge() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot inspect an invalid VoxelNodeGroup.");

    const VoxelState state = m_children[0].state();

    if (state == VoxelState::Subdivided)
    {
        return false;
    }

    for (std::size_t index = 1; index < m_children.size(); ++index)
    {
        if (m_children[index].state() != state)
        {
            return false;
        }
    }

    return true;
}

VoxelState VoxelNodeGroup::mergedState() const
{
    MYVOXEL_ASSERT_MESSAGE(canMerge(), "Only a uniform leaf VoxelNodeGroup can be merged.");
    return m_children[0].state();
}

/// 状态修改

void VoxelNodeGroup::reset(VoxelState state)
{
    MYVOXEL_ASSERT_MESSAGE(state == VoxelState::Empty || state == VoxelState::Material, "VoxelNodeGroup reset state must be Empty or Material.");

    for (std::size_t index = 0; index < m_children.size(); ++index)
    {
        MYVOXEL_ASSERT_MESSAGE(!m_children[index].isSubdivided(), "Cannot reset a VoxelNodeGroup while it still connects descendant node groups.");
        m_children[index].setState(state);
    }
}

}