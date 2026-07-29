#include "VoxelTreeConstCursor.h"

#include <cstddef>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelForest& forest, const VoxelCellAddress& address)
    : m_tree(nullptr)
    , m_node(nullptr)
    , m_virtualState(VoxelState::Empty)
    , m_virtual(false)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    m_tree = forest.findTree(rootAddress.index);

    if (!m_tree)
    {
        m_virtual = true;
        return;
    }

    const VoxelNode* node = &m_tree->root;
    std::vector<VoxelCorner> path;
    VoxelForest::buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelTreeConstCursor encountered an invalid node.");

        if (!node->isSubdivided())
        {
            m_virtualState = node->state();
            m_virtual = true;
            return;
        }

        const VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[pathIndex]);
    }

    MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelTreeConstCursor target node must be valid.");

    m_node = node;
}

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelTree& tree)
    : m_tree(&tree)
    , m_node(&tree.root)
    , m_virtualState(VoxelState::Empty)
    , m_virtual(false)
{
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeConstCursor root node must be valid.");
}

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelTree* tree, const VoxelNode* node)
    : m_tree(tree)
    , m_node(node)
    , m_virtualState(VoxelState::Empty)
    , m_virtual(false)
{
    MYVOXEL_ASSERT_MESSAGE(m_tree, "An actual VoxelTreeConstCursor requires a valid tree.");
    MYVOXEL_ASSERT_MESSAGE(m_node && m_node->isValid(), "An actual VoxelTreeConstCursor requires a valid node.");
}

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelTree* tree, VoxelState virtualState)
    : m_tree(tree)
    , m_node(nullptr)
    , m_virtualState(virtualState)
    , m_virtual(true)
{
    MYVOXEL_ASSERT_MESSAGE(virtualState == VoxelState::Empty || virtualState == VoxelState::Material, "A virtual VoxelTreeConstCursor requires a leaf state.");
}

/// 节点状态

VoxelState VoxelTreeConstCursor::state() const
{
    if (m_virtual)
    {
        return m_virtualState;
    }

    MYVOXEL_ASSERT_MESSAGE(m_node && m_node->isValid(), "VoxelTreeConstCursor actual node must be valid.");
    return m_node->state();
}

bool VoxelTreeConstCursor::hasActualNode() const
{
    return !m_virtual && m_node != nullptr;
}

bool VoxelTreeConstCursor::isVirtual() const
{
    return m_virtual;
}

bool VoxelTreeConstCursor::isEmpty() const
{
    return state() == VoxelState::Empty;
}

bool VoxelTreeConstCursor::isMaterial() const
{
    return state() == VoxelState::Material;
}

bool VoxelTreeConstCursor::isSubdivided() const
{
    return state() == VoxelState::Subdivided;
}

bool VoxelTreeConstCursor::isLeaf() const
{
    const VoxelState currentState = state();
    return currentState == VoxelState::Empty || currentState == VoxelState::Material;
}

/// 子节点访问

VoxelTreeConstCursor VoxelTreeConstCursor::child(VoxelCorner corner) const
{
    if (m_virtual)
    {
        return VoxelTreeConstCursor(m_tree, m_virtualState);
    }

    MYVOXEL_ASSERT_MESSAGE(m_node && m_node->isValid(), "VoxelTreeConstCursor actual node must be valid.");

    if (!m_node->isSubdivided())
    {
        return VoxelTreeConstCursor(m_tree, m_node->state());
    }

    MYVOXEL_ASSERT_MESSAGE(m_tree, "A subdivided VoxelTreeConstCursor requires a valid tree.");

    const VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(m_node->childGroupIndex());
    return VoxelTreeConstCursor(m_tree, &group.child(corner));
}

}