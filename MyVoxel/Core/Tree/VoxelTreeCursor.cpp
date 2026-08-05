#include "VoxelTreeCursor.h"

#include "MyVoxel/Core/Mask/VoxelChildMask.h"

namespace MyVoxel
{

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
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
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(state)
    , m_source(Source::Terminal)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelNodeBlock& block)
    : m_tree(&tree)
    , m_nodeBlock(&block)
    , m_leafBlock(nullptr)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(VoxelState::Subdivided)
    , m_source(Source::NodeBlock)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_state(VoxelState::Subdivided)
    , m_source(Source::MaskLeaf)
{
}

VoxelTreeCursor::VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_nodeBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_coarseCorner(coarseCorner)
    , m_state(VoxelState::Subdivided)
    , m_source(Source::MaskLeafGroup)
{
    assert(leafGroupState(leafBlock, coarseCorner) == VoxelState::Subdivided);
}

/// 子体素读取

VoxelChildStateMasks VoxelTreeCursor::childStateMasks() const
{
    assert(m_tree);

    switch (m_source)
    {
    case Source::Terminal:
        return uniformChildStateMasks(m_state);

    case Source::NodeBlock:
        assert(m_nodeBlock);
        return nodeChildStateMasks(*m_nodeBlock);

    case Source::MaskLeaf:
        assert(m_leafBlock);
        return leafChildStateMasks(*m_leafBlock);

    case Source::MaskLeafGroup:
        assert(m_leafBlock);
        return leafGroupChildStateMasks(*m_leafBlock, m_coarseCorner);
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
        break;
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelState::Empty);
}

/// 压缩材料读取


std::uint64_t VoxelTreeCursor::materialMask() const
{
    assert(hasMaterialMask());
    assert(m_leafBlock);

    return m_leafBlock->materialMask;
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

    case VoxelNodeState::Branch:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        return VoxelTreeCursor(*m_tree, m_tree->m_blockPool->node(index));
    }

    case VoxelNodeState::MaskLeaf:
    {
        const VoxelIndex index = childStorageIndex(*m_nodeBlock, corner);
        return VoxelTreeCursor(*m_tree, m_tree->m_blockPool->leaf(index));
    }
    }

    assert(false);
    return VoxelTreeCursor(*m_tree, VoxelState::Empty);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_leafBlock);

    const VoxelState childState = leafGroupState(*m_leafBlock, corner);

    if (childState == VoxelState::Empty || childState == VoxelState::Material)
    {
        return VoxelTreeCursor(*m_tree, childState);
    }

    return VoxelTreeCursor(*m_tree, *m_leafBlock, corner);
}

VoxelTreeCursor VoxelTreeCursor::maskLeafGroupChild(VoxelCorner corner) const
{
    assert(m_tree);
    assert(m_leafBlock);

    const bool material = maskBit(m_leafBlock->materialMask, m_coarseCorner, corner);
    return VoxelTreeCursor(*m_tree, material ? VoxelState::Material : VoxelState::Empty);
}

}


