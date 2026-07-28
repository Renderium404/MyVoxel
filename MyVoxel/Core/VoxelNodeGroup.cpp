#include "VoxelNodeGroup.h"

#include <cassert>
#include <cstddef>

namespace
{

// 将体素角点转换为节点组数组索引。
std::size_t cornerIndex(MyVoxel::VoxelCorner corner)
{
    const std::size_t index = static_cast<std::size_t>(corner);
    assert(index < static_cast<std::size_t>(MyVoxel::VoxelCornerCount));
    return index;
}

}

namespace MyVoxel
{

VoxelNodeGroup::VoxelNodeGroup(VoxelState state)
{
    reset(state);
}

VoxelNode& VoxelNodeGroup::child(VoxelCorner corner)
{
    return m_children[cornerIndex(corner)];
}

const VoxelNode& VoxelNodeGroup::child(VoxelCorner corner) const
{
    return m_children[cornerIndex(corner)];
}

void VoxelNodeGroup::reset(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    for (std::size_t i = 0; i < m_children.size(); ++i)
    {
        assert(m_children[i].state() != VoxelState::Subdivided);
        m_children[i].setState(state);
    }
}

bool VoxelNodeGroup::canMerge() const
{
    const VoxelState state = m_children[0].state();

    if (state == VoxelState::Subdivided)
    {
        return false;
    }

    for (std::size_t i = 1; i < m_children.size(); ++i)
    {
        if (m_children[i].state() != state)
        {
            return false;
        }
    }

    return true;
}

VoxelState VoxelNodeGroup::mergedState() const
{
    assert(canMerge());
    return m_children[0].state();
}

}