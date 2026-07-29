#include "VoxelPackedTreeEditor.h"

#include <cassert>

namespace
{

// 返回用于初始化非活动角点字段的第0个角点。
MyVoxel::VoxelCorner zeroCorner()
{
    return static_cast<MyVoxel::VoxelCorner>(0);
}

}

namespace MyVoxel
{

VoxelPackedTreeEditor::VoxelPackedTreeEditor(VoxelPackedRootTree& tree)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_corner(zeroCorner())
    , m_coarseCorner(zeroCorner())
    , m_fineCorner(zeroCorner())
    , m_referenceType(ReferenceType::Root)
{
    assert(isValid());
}

VoxelPackedTreeEditor::VoxelPackedTreeEditor(
    VoxelPackedRootTree& tree,
    ReferenceType referenceType,
    VoxelNodeBlock* parentBlock,
    VoxelCorner corner,
    VoxelLeafBlock* leafBlock,
    VoxelCorner coarseCorner,
    VoxelCorner fineCorner)
    : m_tree(&tree)
    , m_parentBlock(parentBlock)
    , m_leafBlock(leafBlock)
    , m_corner(corner)
    , m_coarseCorner(coarseCorner)
    , m_fineCorner(fineCorner)
    , m_referenceType(referenceType)
{
    assert(isValid());
}

VoxelState VoxelPackedTreeEditor::state() const
{
    assert(isValid());

    switch (m_referenceType)
    {
    case ReferenceType::Root:
        return m_tree->rootState;

    case ReferenceType::BranchCell:
        return m_parentBlock->childState(m_corner);

    case ReferenceType::MaskLeaf:
        return VoxelState::Subdivided;

    case ReferenceType::MaskOctant:
        return m_leafBlock->octantState(m_coarseCorner);

    case ReferenceType::MaskVoxel:
        return m_leafBlock->state(m_coarseCorner, m_fineCorner);
    }

    assert(false);
    return VoxelState::Empty;
}

void VoxelPackedTreeEditor::setState(VoxelState stateValue)
{
    assert(isValid());
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    if (m_referenceType == ReferenceType::MaskOctant)
    {
        m_leafBlock->setOctantState(m_coarseCorner, stateValue);
        return;
    }

    if (m_referenceType == ReferenceType::MaskVoxel)
    {
        m_leafBlock->setState(m_coarseCorner, m_fineCorner, stateValue);
        return;
    }

    const VoxelState currentState = state();

    if (currentState == stateValue)
    {
        return;
    }

    if (m_referenceType == ReferenceType::Root)
    {
        if (currentState == VoxelState::Subdivided && m_tree->rootBlock.childMask != 0)
        {
            m_tree->blockPool.releaseChildren(m_tree->rootBlock);
        }

        m_tree->rootBlock.reset(stateValue);
        m_tree->rootState = stateValue;

        assert(isValid());
        assert(m_tree->isValid());
        return;
    }

    if (m_referenceType == ReferenceType::MaskLeaf)
    {
        const VoxelNodeIndex firstChildIndex = m_parentBlock->firstChildIndex;

        m_parentBlock->setChildLeafState(m_corner, stateValue);

        if (m_parentBlock->childMask == 0)
        {
            m_tree->blockPool.releaseUnusedChildren(firstChildIndex);
            m_parentBlock->firstChildIndex = InvalidVoxelNodeIndex;
        }

        m_leafBlock = nullptr;
        m_referenceType = ReferenceType::BranchCell;

        assert(isValid());
        return;
    }

    assert(m_referenceType == ReferenceType::BranchCell);

    const VoxelStorageState storageState = m_parentBlock->childStorageState(m_corner);

    if (storageState == VoxelStorageState::Branch)
    {
        VoxelNodeBlock& block = currentBlock();

        if (block.childMask != 0)
        {
            m_tree->blockPool.releaseChildren(block);
        }

        block.reset();
    }

    const VoxelNodeIndex firstChildIndex = m_parentBlock->firstChildIndex;

    m_parentBlock->setChildLeafState(m_corner, stateValue);

    if (m_parentBlock->childMask == 0 && firstChildIndex != InvalidVoxelNodeIndex)
    {
        m_tree->blockPool.releaseUnusedChildren(firstChildIndex);
        m_parentBlock->firstChildIndex = InvalidVoxelNodeIndex;
    }

    assert(isValid());
}

bool VoxelPackedTreeEditor::split()
{
    assert(isValid());

    if (m_referenceType == ReferenceType::MaskLeaf ||
        m_referenceType == ReferenceType::MaskOctant ||
        m_referenceType == ReferenceType::MaskVoxel)
    {
        return false;
    }

    const VoxelState inheritedState = state();

    if (inheritedState == VoxelState::Subdivided)
    {
        return false;
    }

    assert(inheritedState == VoxelState::Empty || inheritedState == VoxelState::Material);

    if (m_referenceType == ReferenceType::Root)
    {
        m_tree->rootBlock.reset(inheritedState);
        m_tree->rootState = VoxelState::Subdivided;

        assert(isValid());
        assert(m_tree->isValid());
        return true;
    }

    assert(m_referenceType == ReferenceType::BranchCell);

    if (m_parentBlock->childMask == 0)
    {
        assert(m_parentBlock->firstChildIndex == InvalidVoxelNodeIndex);
        m_parentBlock->firstChildIndex = m_tree->blockPool.allocateChildren();
    }

    const VoxelNodeIndex nodeIndex = m_parentBlock->firstChildIndex + static_cast<VoxelNodeIndex>(m_corner);

    m_tree->blockPool.initializeNode(nodeIndex, inheritedState);
    m_parentBlock->setChildBranch(m_corner);

    assert(isValid());
    assert(m_parentBlock->isValid());
    return true;
}

bool VoxelPackedTreeEditor::makeMaskLeaf()
{
    assert(isValid());

    if (m_referenceType != ReferenceType::BranchCell)
    {
        return false;
    }

    const VoxelState inheritedState = state();

    if (inheritedState == VoxelState::Subdivided)
    {
        return false;
    }

    assert(inheritedState == VoxelState::Empty || inheritedState == VoxelState::Material);

    if (m_parentBlock->childMask == 0)
    {
        assert(m_parentBlock->firstChildIndex == InvalidVoxelNodeIndex);
        m_parentBlock->firstChildIndex = m_tree->blockPool.allocateChildren();
    }

    const VoxelNodeIndex leafIndex = m_parentBlock->firstChildIndex + static_cast<VoxelNodeIndex>(m_corner);
    VoxelLeafBlock& leafBlock = m_tree->blockPool.initializeLeaf(leafIndex, inheritedState);

    m_parentBlock->setChildMaskLeaf(m_corner);
    m_leafBlock = &leafBlock;
    m_referenceType = ReferenceType::MaskLeaf;

    assert(isValid());
    assert(m_parentBlock->isValid());
    return true;
}

bool VoxelPackedTreeEditor::isMaskLeaf() const
{
    assert(isValid());
    return m_referenceType == ReferenceType::MaskLeaf;
}

bool VoxelPackedTreeEditor::canAccessChildren() const
{
    assert(isValid());

    if (m_referenceType == ReferenceType::MaskLeaf || m_referenceType == ReferenceType::MaskOctant)
    {
        return true;
    }

    if (m_referenceType == ReferenceType::MaskVoxel)
    {
        return false;
    }

    return state() == VoxelState::Subdivided;
}

VoxelPackedTreeEditor VoxelPackedTreeEditor::child(VoxelCorner corner) const
{
    assert(isValid());
    assert(canAccessChildren());

    if (m_referenceType == ReferenceType::MaskLeaf)
    {
        return VoxelPackedTreeEditor(
            *m_tree,
            ReferenceType::MaskOctant,
            m_parentBlock,
            m_corner,
            m_leafBlock,
            corner,
            zeroCorner());
    }

    if (m_referenceType == ReferenceType::MaskOctant)
    {
        return VoxelPackedTreeEditor(
            *m_tree,
            ReferenceType::MaskVoxel,
            m_parentBlock,
            m_corner,
            m_leafBlock,
            m_coarseCorner,
            corner);
    }

    assert(m_referenceType == ReferenceType::Root || m_referenceType == ReferenceType::BranchCell);

    VoxelNodeBlock& block = currentBlock();
    const VoxelStorageState storageState = block.childStorageState(corner);

    if (storageState == VoxelStorageState::MaskLeaf)
    {
        VoxelLeafBlock& leafBlock = m_tree->blockPool.leaf(block.childLeafIndex(corner));

        return VoxelPackedTreeEditor(
            *m_tree,
            ReferenceType::MaskLeaf,
            &block,
            corner,
            &leafBlock,
            zeroCorner(),
            zeroCorner());
    }

    return VoxelPackedTreeEditor(
        *m_tree,
        ReferenceType::BranchCell,
        &block,
        corner,
        nullptr,
        zeroCorner(),
        zeroCorner());
}

VoxelState VoxelPackedTreeEditor::merge()
{
    assert(isValid());

    if (m_referenceType == ReferenceType::MaskLeaf)
    {
        return mergeMaskLeaf();
    }

    if (m_referenceType == ReferenceType::MaskOctant)
    {
        return m_leafBlock->octantState(m_coarseCorner);
    }

    if (m_referenceType == ReferenceType::MaskVoxel)
    {
        return state();
    }

    if (state() != VoxelState::Subdivided)
    {
        return state();
    }

    VoxelNodeBlock& block = currentBlock();

    if (!block.canMerge())
    {
        return VoxelState::Subdivided;
    }

    const VoxelState mergedState = block.mergedState();

    if (m_referenceType == ReferenceType::Root)
    {
        m_tree->rootBlock.reset(mergedState);
        m_tree->rootState = mergedState;

        assert(isValid());
        assert(m_tree->isValid());
        return mergedState;
    }

    assert(m_referenceType == ReferenceType::BranchCell);

    block.reset();

    const VoxelNodeIndex firstChildIndex = m_parentBlock->firstChildIndex;

    m_parentBlock->setChildLeafState(m_corner, mergedState);

    if (m_parentBlock->childMask == 0)
    {
        m_tree->blockPool.releaseUnusedChildren(firstChildIndex);
        m_parentBlock->firstChildIndex = InvalidVoxelNodeIndex;
    }

    assert(isValid());
    assert(m_parentBlock->isValid());
    return mergedState;
}

VoxelLeafBlock& VoxelPackedTreeEditor::maskLeaf()
{
    assert(isMaskLeaf());
    return *m_leafBlock;
}

const VoxelLeafBlock& VoxelPackedTreeEditor::maskLeaf() const
{
    assert(isMaskLeaf());
    return *m_leafBlock;
}

VoxelNodeBlock& VoxelPackedTreeEditor::currentBlock() const
{
    assert(isValid());
    assert(m_referenceType == ReferenceType::Root || m_referenceType == ReferenceType::BranchCell);
    assert(state() == VoxelState::Subdivided);

    if (m_referenceType == ReferenceType::Root)
    {
        return m_tree->rootBlock;
    }

    assert(m_parentBlock->childStorageState(m_corner) == VoxelStorageState::Branch);
    return m_tree->blockPool.node(m_parentBlock->childNodeIndex(m_corner));
}

VoxelState VoxelPackedTreeEditor::mergeMaskLeaf()
{
    assert(isValid());
    assert(m_referenceType == ReferenceType::MaskLeaf);
    assert(m_leafBlock);

    if (!m_leafBlock->isEmpty() && !m_leafBlock->isFull())
    {
        return VoxelState::Subdivided;
    }

    const VoxelState mergedState = m_leafBlock->isFull() ? VoxelState::Material : VoxelState::Empty;
    const VoxelNodeIndex firstChildIndex = m_parentBlock->firstChildIndex;

    m_parentBlock->setChildLeafState(m_corner, mergedState);

    if (m_parentBlock->childMask == 0)
    {
        m_tree->blockPool.releaseUnusedChildren(firstChildIndex);
        m_parentBlock->firstChildIndex = InvalidVoxelNodeIndex;
    }

    m_leafBlock = nullptr;
    m_referenceType = ReferenceType::BranchCell;

    assert(isValid());
    return mergedState;
}

bool VoxelPackedTreeEditor::isValid() const
{
    if (!m_tree)
    {
        return false;
    }

    const unsigned int cornerIndex = static_cast<unsigned int>(m_corner);
    const unsigned int coarseCornerIndex = static_cast<unsigned int>(m_coarseCorner);
    const unsigned int fineCornerIndex = static_cast<unsigned int>(m_fineCorner);

    if (cornerIndex >= static_cast<unsigned int>(VoxelCornerCount) ||
        coarseCornerIndex >= static_cast<unsigned int>(VoxelCornerCount) ||
        fineCornerIndex >= static_cast<unsigned int>(VoxelCornerCount))
    {
        return false;
    }

    if (m_referenceType == ReferenceType::Root)
    {
        return !m_parentBlock && !m_leafBlock;
    }

    if (!m_parentBlock || !m_parentBlock->isValid())
    {
        return false;
    }

    if (m_referenceType == ReferenceType::BranchCell)
    {
        return !m_leafBlock && m_parentBlock->childStorageState(m_corner) != VoxelStorageState::MaskLeaf;
    }

    return m_leafBlock && m_parentBlock->childStorageState(m_corner) == VoxelStorageState::MaskLeaf;
}

}