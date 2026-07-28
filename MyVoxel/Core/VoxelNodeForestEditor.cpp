#include "VoxelNodeForestEditor.h"

#include <cassert>

namespace MyVoxel
{

VoxelNodeForestEditor::VoxelNodeForestEditor(VoxelNodeForest& forest, const VoxelCellAddress& address)
    : m_tree(forest.findEditableRootTree(VoxelNodeForest::rootCellAddress(address).index))
    , m_node(nullptr)
{
    assert(m_tree);

    m_node = VoxelNodeForest::findNode(*m_tree, address);

    assert(m_node);
    assert(m_node->isValid());
}
VoxelNodeForestEditor::VoxelNodeForestEditor(VoxelRootTree& tree)
    : m_tree(&tree)
    , m_node(&tree.root)
{
    assert(m_node->isValid());
}
VoxelNodeForestEditor::VoxelNodeForestEditor(VoxelRootTree& tree, VoxelNode& node)
    : m_tree(&tree)
    , m_node(&node)
{
    assert(m_node->isValid());
}

VoxelState VoxelNodeForestEditor::state() const
{
    assert(m_tree);
    assert(m_node);
    assert(m_node->isValid());
    return m_node->state();
}

void VoxelNodeForestEditor::setState(VoxelState stateValue)
{
    assert(m_tree);
    assert(m_node);
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    if (m_node->state() == stateValue)
    {
        return;
    }

    if (m_node->state() == VoxelState::Subdivided)
    {
        const VoxelNodeGroupIndex groupIndex = m_node->childGroupIndex();

        m_node->setState(stateValue);
        m_tree->nodePool.releaseNodeGroup(groupIndex);
    }
    else
    {
        m_node->setState(stateValue);
    }

    assert(m_node->isValid());
}

void VoxelNodeForestEditor::split()
{
    assert(m_tree);
    assert(m_node);
    assert(m_node->state() == VoxelState::Material);

    const VoxelNodeGroupIndex groupIndex = m_tree->nodePool.allocateNodeGroup(VoxelState::Material);
    m_node->setChildGroupIndex(groupIndex);

    assert(m_node->isValid());
}

VoxelNodeForestEditor VoxelNodeForestEditor::child(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_node);
    assert(m_node->state() == VoxelState::Subdivided);

    VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(m_node->childGroupIndex());
    return VoxelNodeForestEditor(*m_tree, group.child(corner));
}

VoxelState VoxelNodeForestEditor::merge()
{
    assert(m_tree);
    assert(m_node);

    if (m_node->state() != VoxelState::Subdivided)
    {
        return m_node->state();
    }

    const VoxelNodeGroupIndex groupIndex = m_node->childGroupIndex();
    const VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(groupIndex);

    if (!group.canMerge())
    {
        return VoxelState::Subdivided;
    }

    const VoxelState mergedState = group.mergedState();

    m_node->setState(mergedState);
    m_tree->nodePool.releaseNodeGroup(groupIndex);

    assert(m_node->isValid());
    return mergedState;
}

}