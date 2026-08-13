#include "VoxelTreeCursor.h"

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
    , m_value(0.0f)
    , m_state(tree.m_rootState)
    , m_source(Source::Terminal)
{
    assert(tree.isValid());

    if (tree.m_rootState == VoxelState::Subdivided)
    {
        m_nodeBlock = &tree.m_rootBlock;
        m_source = Source::NodeBlock;
        return;
    }

    assert(tree.m_rootState == VoxelState::Empty || tree.m_rootState == VoxelState::Material);
    m_value = nodeValue(tree.m_rootBlock);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, VoxelState state, float value)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_fineCorner(VoxelCorner::Minimum)
    , m_value(value)
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
    , m_value(0.0f)
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
    , m_value(0.0f)
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
    , m_value(0.0f)
    , m_state(VoxelState::Subdivided)
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
    , m_value(0.0f)
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
    return VoxelTreeCursor(*m_tree, VoxelState::Empty, 1.0f);
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

    if (m_source == Source::Terminal)
    {
        return m_value;
    }

    assert(m_source == Source::MaskLeafVoxel && m_leafBlock);
    return leafDistance(*m_leafBlock, m_coarseCorner, m_fineCorner);
}

/// 内部辅助

VoxelTreeCursor VoxelTreeCursor::nodeChild(VoxelCorner corner) const
{
    assert(m_tree && m_tree->m_blockPool && m_nodeBlock);

    const VoxelNodeState currentState = nodeState(*m_nodeBlock, corner);
    const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
    const BlockPool& pool = *m_tree->m_blockPool;

    switch (currentState)
    {
    case VoxelNodeState::Empty:
        return VoxelTreeCursor(*m_tree, VoxelState::Empty, pool.value(index));
    case VoxelNodeState::Material:
        return VoxelTreeCursor(*m_tree, VoxelState::Material, pool.value(index));
    case VoxelNodeState::Node:
        return VoxelTreeCursor(*m_tree, pool.node(index));
    case VoxelNodeState::MaskLeaf:
    {
        const ConstLeafData leaf = pool.leaf(index);
        return VoxelTreeCursor(*m_tree, *leaf.mask, *leaf.block);
    }
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelState::Empty, 1.0f);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafChild(VoxelCorner corner) const
{
    assert(m_maskBlock && m_leafBlock);
    return VoxelTreeCursor(*m_tree, *m_maskBlock, *m_leafBlock, corner);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafGroupChild(VoxelCorner corner) const
{
    assert(m_maskBlock && m_leafBlock);
    return VoxelTreeCursor(*m_tree, *m_maskBlock, *m_leafBlock, m_coarseCorner, corner);
}

}