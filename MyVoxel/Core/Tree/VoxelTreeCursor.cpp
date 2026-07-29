#include "VoxelTreeCursor.h"

namespace MyVoxel
{

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(VoxelCursorState::Empty)
    , m_source(Source::Terminal)
{
    assert(tree.isValid());

    switch (tree.rootState)
    {
    case VoxelRootState::Empty:
        m_state = VoxelCursorState::Empty;
        return;

    case VoxelRootState::Material:
        m_state = VoxelCursorState::Material;
        return;

    case VoxelRootState::Subdivided:
        m_nodeBlock = &tree.rootBlock;
        m_state = VoxelCursorState::Subdivided;
        m_source = Source::NodeBlock;
        return;
    }

    assert(false);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, VoxelCursorState state)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(state)
    , m_source(Source::Terminal)
{
    assert(state == VoxelCursorState::Empty || state == VoxelCursorState::Material);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelNodeBlock& block)
    : m_tree(&tree)
    , m_nodeBlock(&block)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(VoxelCursorState::Subdivided)
    , m_source(Source::NodeBlock)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(VoxelCursorState::Subdivided)
    , m_source(Source::MaskLeaf)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_coarseCorner(coarseCorner)
    , m_state(VoxelCursorState::Subdivided)
    , m_source(Source::MaskLeafGroup)
{
    const std::uint8_t coarseMask = maskBits(leafBlock.materialMask, coarseCorner);

    assert(coarseMask != static_cast<std::uint8_t>(0));
    assert(coarseMask != static_cast<std::uint8_t>(0xFFU));
}

/// 节点状态

VoxelCursorState VoxelTreeCursor::state() const
{
    return m_state;
}

bool VoxelTreeCursor::isEmpty() const
{
    return m_state == VoxelCursorState::Empty;
}

bool VoxelTreeCursor::isMaterial() const
{
    return m_state == VoxelCursorState::Material;
}

bool VoxelTreeCursor::isSubdivided() const
{
    return m_state == VoxelCursorState::Subdivided;
}

bool VoxelTreeCursor::isTerminal() const
{
    return m_state != VoxelCursorState::Subdivided;
}

bool VoxelTreeCursor::canAccessChildren() const
{
    return m_state == VoxelCursorState::Subdivided;
}

/// 子节点访问

VoxelTreeCursor VoxelTreeCursor::child(VoxelCorner corner) const
{
    assert(canAccessChildren());

    switch (m_source)
    {
    case Source::NodeBlock:
        return nodeChild(corner);

    case Source::MaskLeaf:
        return maskLeafChild(corner);

    case Source::MaskLeafGroup:
        return maskLeafGroupChild(corner);

    case Source::Terminal:
        break;
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelCursorState::Empty);
}

/// 底层存储

bool VoxelTreeCursor::usesNodeBlock() const
{
    return m_source == Source::NodeBlock;
}

bool VoxelTreeCursor::usesMaskLeaf() const
{
    return m_source == Source::MaskLeaf || m_source == Source::MaskLeafGroup;
}

const VoxelNodeBlock& VoxelTreeCursor::nodeBlock() const
{
    assert(usesNodeBlock());
    assert(m_nodeBlock);

    return *m_nodeBlock;
}

const VoxelLeafBlock& VoxelTreeCursor::maskLeaf() const
{
    assert(usesMaskLeaf());
    assert(m_leafBlock);

    return *m_leafBlock;
}

/// 内部辅助

VoxelTreeCursor VoxelTreeCursor::nodeChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_tree->blockPool);
    assert(m_nodeBlock);

    const VoxelState currentState = childState(*m_nodeBlock, corner);

    switch (currentState)
    {
    case VoxelState::Empty:
        return VoxelTreeCursor(*m_tree, VoxelCursorState::Empty);

    case VoxelState::Material:
        return VoxelTreeCursor(*m_tree, VoxelCursorState::Material);

    case VoxelState::Branch:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        return VoxelTreeCursor(*m_tree, m_tree->blockPool->node(index));
    }

    case VoxelState::MaskLeaf:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        return VoxelTreeCursor(*m_tree, m_tree->blockPool->leaf(index));
    }
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelCursorState::Empty);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_leafBlock);

    const std::uint8_t coarseMask = maskBits(m_leafBlock->materialMask, corner);

    if (coarseMask == static_cast<std::uint8_t>(0))
    {
        return VoxelTreeCursor(*m_tree, VoxelCursorState::Empty);
    }

    if (coarseMask == static_cast<std::uint8_t>(0xFFU))
    {
        return VoxelTreeCursor(*m_tree, VoxelCursorState::Material);
    }

    return VoxelTreeCursor(*m_tree, *m_leafBlock, corner);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafGroupChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_leafBlock);

    const bool material = maskBit(m_leafBlock->materialMask, m_coarseCorner, corner);
    return VoxelTreeCursor(*m_tree, material ? VoxelCursorState::Material : VoxelCursorState::Empty);
}

}