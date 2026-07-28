#include "VoxelNodeForestConstCursor.h"

#include <cassert>

namespace MyVoxel
{

VoxelNodeForestConstCursor::VoxelNodeForestConstCursor(const VoxelNodeForest& forest, const VoxelCellAddress& address)
    : m_tree(forest.findRootTree(VoxelNodeForest::rootCellAddress(address).index))
    , m_node(nullptr)
    , m_virtualMaterial(false)
{
    assert(m_tree);

    m_node = VoxelNodeForest::findNode(*m_tree, address);

    assert(m_node);
    assert(m_node->isValid());
}

VoxelNodeForestConstCursor::VoxelNodeForestConstCursor(const VoxelRootTree& tree, const VoxelNode* node, bool virtualMaterial)
    : m_tree(&tree)
    , m_node(node)
    , m_virtualMaterial(virtualMaterial)
{
    assert(m_virtualMaterial || m_node);
    assert(!m_node || m_node->isValid());
}

VoxelState VoxelNodeForestConstCursor::state() const
{
    assert(m_tree);

    if (m_virtualMaterial)
    {
        return VoxelState::Material;
    }

    assert(m_node);
    return m_node->state();
}

VoxelNodeForestConstCursor VoxelNodeForestConstCursor::child(VoxelCorner corner) const
{
    assert(m_tree);

    if (m_virtualMaterial)
    {
        return VoxelNodeForestConstCursor(*m_tree, nullptr, true);
    }

    assert(m_node);

    if (m_node->state() == VoxelState::Material)
    {
        return VoxelNodeForestConstCursor(*m_tree, nullptr, true);
    }

    assert(m_node->state() == VoxelState::Subdivided);

    const VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(m_node->childGroupIndex());
    return VoxelNodeForestConstCursor(*m_tree, &group.child(corner), false);
}

}