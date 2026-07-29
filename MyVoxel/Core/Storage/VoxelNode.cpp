#include "VoxelNode.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelNode::VoxelNode()
    : m_childGroupIndex(InvalidVoxelNodeGroupIndex)
    , m_state(VoxelState::Empty)
{
}

VoxelNode::VoxelNode(VoxelState state)
    : m_childGroupIndex(InvalidVoxelNodeGroupIndex)
    , m_state(VoxelState::Empty)
{
    setState(state);
}

/// 状态判断

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

    if (m_state == VoxelState::Empty || m_state == VoxelState::Material)
    {
        return m_childGroupIndex == InvalidVoxelNodeGroupIndex;
    }

    return false;
}

bool VoxelNode::isEmpty() const
{
    return m_state == VoxelState::Empty;
}

bool VoxelNode::isMaterial() const
{
    return m_state == VoxelState::Material;
}

bool VoxelNode::isSubdivided() const
{
    return m_state == VoxelState::Subdivided;
}

bool VoxelNode::isLeaf() const
{
    return m_state == VoxelState::Empty || m_state == VoxelState::Material;
}

bool VoxelNode::hasChildGroup() const
{
    return m_state == VoxelState::Subdivided && m_childGroupIndex != InvalidVoxelNodeGroupIndex;
}

/// 子节点组

VoxelNodeGroupIndex VoxelNode::childGroupIndex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid VoxelNode.");
    MYVOXEL_ASSERT_MESSAGE(hasChildGroup(), "Only a subdivided VoxelNode has a child node group.");
    return m_childGroupIndex;
}

/// 状态修改

void VoxelNode::setState(VoxelState state)
{
    MYVOXEL_ASSERT_MESSAGE(state == VoxelState::Empty || state == VoxelState::Material, "VoxelNode leaf state must be Empty or Material.");

    m_state = state;
    m_childGroupIndex = InvalidVoxelNodeGroupIndex;
}

void VoxelNode::setChildGroupIndex(VoxelNodeGroupIndex index)
{
    MYVOXEL_ASSERT_MESSAGE(index != InvalidVoxelNodeGroupIndex, "A subdivided VoxelNode requires a valid child node group index.");

    m_state = VoxelState::Subdivided;
    m_childGroupIndex = index;
}

}