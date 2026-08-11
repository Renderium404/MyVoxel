#include "VoxelTreeCursor.h"

#include <limits>

#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"

namespace MyVoxel
{

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_state(VoxelState::Empty)
    , m_source(Source::Terminal)
{
    assert(tree.isValid());

    switch (tree.m_rootState)
    {
    case VoxelState::Empty:
        m_state = VoxelState::Empty;
        return;

    case VoxelState::Material:
        m_state = VoxelState::Material;
        return;

    case VoxelState::Subdivided:
        m_nodeBlock = &tree.m_rootBlock;
        m_state = VoxelState::Subdivided;
        m_source = Source::NodeBlock;
        return;
    }

    assert(false);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, VoxelState state)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_state(state)
    , m_source(Source::Terminal)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const NodeBlock& block)
    : m_tree(&tree)
    , m_nodeBlock(&block)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_state(VoxelState::Subdivided)
    , m_source(Source::NodeBlock)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlockValue, const LeafBlock& leafBlockValue)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_state(VoxelState::Subdivided)
    , m_source(Source::MaskLeaf)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlockValue, const LeafBlock& leafBlockValue, VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_coarseCorner(coarseCorner)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_state(leafGroupState(maskBlockValue, coarseCorner))
    , m_source(Source::MaskLeafGroup)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlockValue, const LeafBlock& leafBlockValue, VoxelCorner coarseCorner, VoxelCorner fineCorner)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_coarseCorner(coarseCorner)
    , m_fineCorner(fineCorner)
    , m_state(leafMaterialBit(maskBlockValue, coarseCorner, fineCorner) ? VoxelState::Material : VoxelState::Empty)
    , m_source(Source::MaskLeafVoxel)
{
}

/// 子体素读取

VoxelChildStateMasks VoxelTreeCursor::childStateMasks() const
{
    assert(m_tree);
    assert(m_source != Source::MaskLeafVoxel);

    switch (m_source)
    {
    case Source::Terminal:
        return uniformChildStateMasks(m_state);

    case Source::NodeBlock:
        assert(m_nodeBlock);
        return nodeChildStateMasks(*m_nodeBlock);

    case Source::MaskLeaf:
        assert(m_maskBlock);
        return leafChildStateMasks(*m_maskBlock);

    case Source::MaskLeafGroup:
        assert(m_maskBlock);
        return leafGroupChildStateMasks(*m_maskBlock, m_coarseCorner);

    case Source::MaskLeafVoxel:
        break;
    }

    assert(false);
    return uniformChildStateMasks(VoxelState::Empty);
}

VoxelTreeCursor VoxelTreeCursor::child(VoxelCorner corner) const
{
    assert(isSubdivided());

    switch (m_source)
    {
    case Source::NodeBlock:
        return nodeChild(corner);

    case Source::MaskLeaf:
        return maskLeafChild(corner);

    case Source::MaskLeafGroup:
        return maskLeafGroupChild(corner);

    case Source::Terminal:
    case Source::MaskLeafVoxel:
        break;
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelState::Empty);
}

/// MaskLeaf读取

const MaskBlock& VoxelTreeCursor::maskBlock() const
{
    assert(hasLeafData());
    assert(m_maskBlock);
    return *m_maskBlock;
}

const LeafBlock& VoxelTreeCursor::leafBlock() const
{
    assert(hasLeafData());
    assert(m_leafBlock);
    return *m_leafBlock;
}

std::uint64_t VoxelTreeCursor::materialMask() const
{
    return maskBlock().materialMask;
}

/// 距离读取

float VoxelTreeCursor::distance() const
{
    assert(hasDistance());

    if (m_source == Source::MaskLeafVoxel)
    {
        assert(m_leafBlock);
        return m_leafBlock->distances[leafBitIndex(m_coarseCorner, m_fineCorner)];
    }

    const float infinity = (std::numeric_limits<float>::infinity)();
    return m_state == VoxelState::Material ? -infinity : infinity;
}

/// 内部辅助

VoxelTreeCursor VoxelTreeCursor::nodeChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_tree->m_blockPool);
    assert(m_nodeBlock);

    const VoxelNodeState currentState = nodeState(*m_nodeBlock, corner);

    switch (currentState)
    {
    case VoxelNodeState::Empty:
        return VoxelTreeCursor(*m_tree, VoxelState::Empty);

    case VoxelNodeState::Material:
        return VoxelTreeCursor(*m_tree, VoxelState::Material);

    case VoxelNodeState::Node:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        return VoxelTreeCursor(*m_tree, m_tree->m_blockPool->node(index));
    }

    case VoxelNodeState::MaskLeaf:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        const ConstLeafData leaf = m_tree->m_blockPool->leaf(index);
        return VoxelTreeCursor(*m_tree, *leaf.mask, *leaf.block);
    }
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelState::Empty);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafChild(VoxelCorner corner) const
{
    assert(m_maskBlock);
    assert(m_leafBlock);
    return VoxelTreeCursor(*m_tree, *m_maskBlock, *m_leafBlock, corner);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafGroupChild(VoxelCorner corner) const
{
    assert(m_maskBlock);
    assert(m_leafBlock);
    return VoxelTreeCursor(*m_tree, *m_maskBlock, *m_leafBlock, m_coarseCorner, corner);
}

}