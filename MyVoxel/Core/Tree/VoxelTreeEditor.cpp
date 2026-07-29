#include "VoxelTreeEditor.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelTreeEditor::VoxelTreeEditor(VoxelForest& forest, const VoxelCellAddress& address)
    : m_tree(nullptr)
    , m_node(nullptr)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    m_tree = forest.findEditableTree(rootAddress.index);

    MYVOXEL_ASSERT_MESSAGE(m_tree, "VoxelTreeEditor requires an existing VoxelTree.");

    m_node = VoxelForest::findNode(*m_tree, address);

    MYVOXEL_ASSERT_MESSAGE(m_node, "VoxelTreeEditor requires an actually created node.");
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor target node must be valid.");
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree)
    : m_tree(&tree)
    , m_node(&tree.root)
{
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor root node must be valid.");
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, VoxelNode& node)
    : m_tree(&tree)
    , m_node(&node)
{
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor child node must be valid.");
}

/// 节点状态

VoxelState VoxelTreeEditor::state() const
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor node must be valid.");
    return m_node->state();
}

bool VoxelTreeEditor::isEmpty() const
{
    return state() == VoxelState::Empty;
}

bool VoxelTreeEditor::isMaterial() const
{
    return state() == VoxelState::Material;
}

bool VoxelTreeEditor::isSubdivided() const
{
    return state() == VoxelState::Subdivided;
}

bool VoxelTreeEditor::isLeaf() const
{
    const VoxelState currentState = state();
    return currentState == VoxelState::Empty || currentState == VoxelState::Material;
}

bool VoxelTreeEditor::setState(VoxelState stateValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");
    MYVOXEL_ASSERT_MESSAGE(stateValue == VoxelState::Empty || stateValue == VoxelState::Material, "VoxelTreeEditor state must be Empty or Material.");

    if (m_node->state() == stateValue)
    {
        return false;
    }

    if (m_node->isSubdivided())
    {
        const VoxelNodeGroupIndex groupIndex = m_node->childGroupIndex();

        m_node->setState(stateValue);
        m_tree->nodePool.releaseNodeGroup(groupIndex);
    }
    else
    {
        m_node->setState(stateValue);
    }

    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor node must remain valid after changing state.");
    return true;
}

/// 节点结构

bool VoxelTreeEditor::split()
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");
    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor node must be valid.");

    if (m_node->isSubdivided())
    {
        return false;
    }

    const VoxelState initialState = m_node->state();

    MYVOXEL_ASSERT_MESSAGE(initialState == VoxelState::Empty || initialState == VoxelState::Material, "VoxelTreeEditor can only split a leaf node.");

    const VoxelNodeGroupIndex groupIndex = m_tree->nodePool.allocateNodeGroup(initialState);
    m_node->setChildGroupIndex(groupIndex);

    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor node must remain valid after splitting.");
    return true;
}

VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner) const
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");
    MYVOXEL_ASSERT_MESSAGE(m_node->isSubdivided(), "VoxelTreeEditor child access requires a subdivided node.");

    VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(m_node->childGroupIndex());
    return VoxelTreeEditor(*m_tree, group.child(corner));
}

VoxelTreeEditor VoxelTreeEditor::ensureChild(VoxelCorner corner)
{
    if (!m_node->isSubdivided())
    {
        split();
    }

    return child(corner);
}

bool VoxelTreeEditor::canMerge() const
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");

    if (!m_node->isSubdivided())
    {
        return false;
    }

    const VoxelNodeGroup& group = m_tree->nodePool.nodeGroup(m_node->childGroupIndex());
    return group.canMerge();
}

VoxelState VoxelTreeEditor::merge()
{
    MYVOXEL_ASSERT_MESSAGE(m_tree && m_node, "VoxelTreeEditor must reference a valid tree and node.");

    if (!m_node->isSubdivided())
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

    MYVOXEL_ASSERT_MESSAGE(m_node->isValid(), "VoxelTreeEditor node must remain valid after merging.");
    return mergedState;
}

}