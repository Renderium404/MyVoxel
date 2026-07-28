#include "VoxelNode.h"

#include <cassert>

namespace MyVoxel
{

VoxelNode::VoxelNode(VoxelState state)
{
    setState(state);
}

VoxelState VoxelNode::state() const
{
    return m_state;
}

bool VoxelNode::isValid() const
{
    if (m_state == VoxelState::Subdivided)
    {
        return m_childGroupIndex != InvalidVoxelNodeGroupIndex;
    }

    return m_childGroupIndex == InvalidVoxelNodeGroupIndex;
}

VoxelNodeGroupIndex VoxelNode::childGroupIndex() const
{
    assert(m_state == VoxelState::Subdivided);
    assert(m_childGroupIndex != InvalidVoxelNodeGroupIndex);

    return m_childGroupIndex;
}

void VoxelNode::setState(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    m_state = state;
    m_childGroupIndex = InvalidVoxelNodeGroupIndex;
}

void VoxelNode::setChildGroupIndex(VoxelNodeGroupIndex index)
{
    assert(index != InvalidVoxelNodeGroupIndex);

    m_state = VoxelState::Subdivided;
    m_childGroupIndex = index;
}

}