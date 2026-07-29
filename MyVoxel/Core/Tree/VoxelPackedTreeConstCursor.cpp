#include "VoxelPackedTreeConstCursor.h"

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

VoxelPackedTreeConstCursor::VoxelPackedTreeConstCursor(const VoxelPackedRootTree& tree)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_corner(zeroCorner())
    , m_coarseCorner(zeroCorner())
    , m_fineCorner(zeroCorner())
    , m_virtualState(VoxelState::Empty)
    , m_referenceType(ReferenceType::Root)
{
    assert(isValid());
}

VoxelPackedTreeConstCursor::VoxelPackedTreeConstCursor(
    const VoxelPackedRootTree& tree,
    ReferenceType referenceType,
    const VoxelNodeBlock* parentBlock,
    VoxelCorner corner,
    const VoxelLeafBlock* leafBlock,
    VoxelCorner coarseCorner,
    VoxelCorner fineCorner,
    VoxelState virtualState)
    : m_tree(&tree)
    , m_parentBlock(parentBlock)
    , m_leafBlock(leafBlock)
    , m_corner(corner)
    , m_coarseCorner(coarseCorner)
    , m_fineCorner(fineCorner)
    , m_virtualState(virtualState)
    , m_referenceType(referenceType)
{
    assert(isValid());
}

VoxelState VoxelPackedTreeConstCursor::state() const
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

    case ReferenceType::VirtualLeaf:
        return m_virtualState;
    }

    assert(false);
    return VoxelState::Empty;
}

bool VoxelPackedTreeConstCursor::isMaskLeaf() const
{
    assert(isValid());
    return m_referenceType == ReferenceType::MaskLeaf;
}

bool VoxelPackedTreeConstCursor::canAccessChildren() const
{
    assert(isValid());

    if (m_referenceType == ReferenceType::MaskLeaf || m_referenceType == ReferenceType::MaskOctant)
    {
        return true;
    }

    if (m_referenceType == ReferenceType::MaskVoxel || m_referenceType == ReferenceType::VirtualLeaf)
    {
        return false;
    }

    return state() == VoxelState::Subdivided;
}

VoxelPackedTreeConstCursor VoxelPackedTreeConstCursor::child(VoxelCorner corner) const
{
    assert(isValid());

    if (m_referenceType == ReferenceType::VirtualLeaf)
    {
        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::VirtualLeaf,
            nullptr,
            zeroCorner(),
            nullptr,
            zeroCorner(),
            zeroCorner(),
            m_virtualState);
    }

    if (m_referenceType == ReferenceType::MaskVoxel)
    {
        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::VirtualLeaf,
            nullptr,
            zeroCorner(),
            nullptr,
            zeroCorner(),
            zeroCorner(),
            state());
    }

    if (m_referenceType == ReferenceType::MaskLeaf)
    {
        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::MaskOctant,
            m_parentBlock,
            m_corner,
            m_leafBlock,
            corner,
            zeroCorner(),
            VoxelState::Empty);
    }

    if (m_referenceType == ReferenceType::MaskOctant)
    {
        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::MaskVoxel,
            m_parentBlock,
            m_corner,
            m_leafBlock,
            m_coarseCorner,
            corner,
            VoxelState::Empty);
    }

    const VoxelState currentState = state();

    if (currentState == VoxelState::Empty || currentState == VoxelState::Material)
    {
        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::VirtualLeaf,
            nullptr,
            zeroCorner(),
            nullptr,
            zeroCorner(),
            zeroCorner(),
            currentState);
    }

    assert(canAccessChildren());

    const VoxelNodeBlock& block = currentBlock();
    const VoxelStorageState storageState = block.childStorageState(corner);

    if (storageState == VoxelStorageState::MaskLeaf)
    {
        const VoxelLeafBlock& leafBlock = m_tree->blockPool.leaf(block.childLeafIndex(corner));

        return VoxelPackedTreeConstCursor(
            *m_tree,
            ReferenceType::MaskLeaf,
            &block,
            corner,
            &leafBlock,
            zeroCorner(),
            zeroCorner(),
            VoxelState::Empty);
    }

    return VoxelPackedTreeConstCursor(
        *m_tree,
        ReferenceType::BranchCell,
        &block,
        corner,
        nullptr,
        zeroCorner(),
        zeroCorner(),
        VoxelState::Empty);
}

const VoxelLeafBlock& VoxelPackedTreeConstCursor::maskLeaf() const
{
    assert(isMaskLeaf());
    return *m_leafBlock;
}

const VoxelNodeBlock& VoxelPackedTreeConstCursor::currentBlock() const
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

bool VoxelPackedTreeConstCursor::isValid() const
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

    if (m_referenceType == ReferenceType::VirtualLeaf)
    {
        return !m_parentBlock &&
               !m_leafBlock &&
               (m_virtualState == VoxelState::Empty || m_virtualState == VoxelState::Material);
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