#include "VoxelTreeEditor.h"

#include <cassert>

#include "../../Foundation/Diagnostic.h"
#include "../../Foundation/RefPtr.h"

namespace MyVoxel
{

namespace
{

// 将终止逻辑状态转换为普通节点掩码状态。
VoxelState terminalVoxelState(VoxelTreeState state)
{
    MYVOXEL_ASSERT(state == VoxelTreeState::Empty || state == VoxelTreeState::Material);
    return state == VoxelTreeState::Material ? VoxelState::Material : VoxelState::Empty;
}

// 将终止逻辑状态转换为根节点状态。
VoxelRootState terminalRootState(VoxelTreeState state)
{
    MYVOXEL_ASSERT(state == VoxelTreeState::Empty || state == VoxelTreeState::Material);
    return state == VoxelTreeState::Material ? VoxelRootState::Material : VoxelRootState::Empty;
}

}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::Root)
{
    MYVOXEL_REQUIRE_MESSAGE(tree.isValid(), "Cannot create VoxelTreeEditor for an invalid VoxelTree.");
    detachPool(tree);
    MYVOXEL_ASSERT(tree.isValid());
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, VoxelNodeBlock& parentBlock, VoxelCorner childCorner)
    : m_tree(&tree)
    , m_parentBlock(&parentBlock)
    , m_leafBlock(nullptr)
    , m_childCorner(childCorner)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::NodeChild)
{
    cornerBit(childCorner);
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(coarseCorner)
    , m_source(Source::MaskLeafGroup)
{
    cornerBit(coarseCorner);
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_childCorner(fineCorner)
    , m_coarseCorner(coarseCorner)
    , m_source(Source::MaskLeafVoxel)
{
    cornerBit(coarseCorner);
    cornerBit(fineCorner);
}

/// 节点状态

VoxelTreeState VoxelTreeEditor::state() const
{
    MYVOXEL_ASSERT(m_tree);

    switch (m_source)
    {
    case Source::Root:
        return m_tree->rootState;

    case Source::NodeChild:
    {
        const VoxelState currentState = nodeChildState();

        if (currentState == VoxelState::Empty)
        {
            return VoxelTreeState::Empty;
        }

        if (currentState == VoxelState::Material)
        {
            return VoxelTreeState::Material;
        }

        return VoxelTreeState::Subdivided;
    }

    case Source::MaskLeafGroup:
    {
        MYVOXEL_ASSERT(m_leafBlock);

        const std::uint8_t groupMask = maskBits(m_leafBlock->materialMask, m_coarseCorner);

        if (groupMask == static_cast<std::uint8_t>(0))
        {
            return VoxelTreeState::Empty;
        }

        if (groupMask == static_cast<std::uint8_t>(0xFFU))
        {
            return VoxelTreeState::Material;
        }

        return VoxelTreeState::Subdivided;
    }

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT(m_leafBlock);
        return maskBit(m_leafBlock->materialMask, m_coarseCorner, m_childCorner) ? VoxelTreeState::Material : VoxelTreeState::Empty;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return VoxelTreeState::Empty;
}

bool VoxelTreeEditor::isEmpty() const
{
    return state() == VoxelTreeState::Empty;
}

bool VoxelTreeEditor::isMaterial() const
{
    return state() == VoxelTreeState::Material;
}

bool VoxelTreeEditor::isSubdivided() const
{
    return state() == VoxelTreeState::Subdivided;
}

bool VoxelTreeEditor::isTerminal() const
{
    return state() != VoxelTreeState::Subdivided;
}

bool VoxelTreeEditor::canAccessChildren() const
{
    if (m_source == Source::MaskLeafGroup)
    {
        return true;
    }

    return state() == VoxelTreeState::Subdivided;
}

/// 状态修改

void VoxelTreeEditor::setEmpty()
{
    setTerminalState(VoxelTreeState::Empty);
}

void VoxelTreeEditor::setMaterial()
{
    setTerminalState(VoxelTreeState::Material);
}

void VoxelTreeEditor::setState(VoxelTreeState newState)
{
    switch (newState)
    {
    case VoxelTreeState::Empty:
        setEmpty();
        return;

    case VoxelTreeState::Material:
        setMaterial();
        return;

    case VoxelTreeState::Subdivided:
        subdivide();
        return;
    }

    MYVOXEL_REQUIRE_MESSAGE(false, "Unknown VoxelTreeState.");
}

void VoxelTreeEditor::subdivide()
{
    MYVOXEL_ASSERT(m_tree);
    VoxelBlockPool& pool = *m_tree->blockPool;

    switch (m_source)
    {
    case Source::Root:
    {
        if (m_tree->rootState == VoxelRootState::Subdivided)
        {
            return;
        }

        const VoxelState inheritedState = m_tree->rootState == VoxelRootState::Material ? VoxelState::Material : VoxelState::Empty;

        reset(m_tree->rootBlock, inheritedState);
        m_tree->rootState = VoxelRootState::Subdivided;
        return;
    }

    case Source::NodeChild:
    {
        const VoxelState currentState = nodeChildState();

        if (currentState == VoxelState::Branch || currentState == VoxelState::MaskLeaf)
        {
            return;
        }

        ensureParentStorage();

        const VoxelIndex index = m_parentBlock->firstChildIndex + static_cast<VoxelIndex>(m_childCorner);
        pool.initializeNode(index, currentState);
        setNodeChildState(*m_parentBlock, m_childCorner, VoxelState::Branch);
        return;
    }

    case Source::MaskLeafGroup:
        // 掩码叶块已经物理保存该粗层体素的八个最高层子体素。
        return;

    case Source::MaskLeafVoxel:
        MYVOXEL_REQUIRE_MESSAGE(false, "The highest-level voxel cannot be subdivided.");
        return;
    }

    MYVOXEL_REQUIRE_MESSAGE(false, "Unknown VoxelTreeEditor source.");
}

void VoxelTreeEditor::subdivideAsMaskLeaf()
{
    MYVOXEL_REQUIRE_MESSAGE(m_source == Source::NodeChild, "Only a normal node child can be converted to a mask leaf.");

    const VoxelState currentState = nodeChildState();

    if (currentState == VoxelState::MaskLeaf)
    {
        return;
    }

    MYVOXEL_REQUIRE_MESSAGE(currentState == VoxelState::Empty || currentState == VoxelState::Material,
                            "A branch cannot be converted directly to a mask leaf.");

    ensureParentStorage();

    VoxelBlockPool& pool = *m_tree->blockPool;
    const VoxelIndex index = m_parentBlock->firstChildIndex + static_cast<VoxelIndex>(m_childCorner);

    pool.initializeLeaf(index, currentState);
    setNodeChildState(*m_parentBlock, m_childCorner, VoxelState::MaskLeaf);
}

/// 子节点访问

VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner)
{
    cornerBit(corner);
    MYVOXEL_ASSERT(m_tree);

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->rootState != VoxelRootState::Subdivided)
        {
            subdivide();
        }

        return VoxelTreeEditor(*m_tree, m_tree->rootBlock, corner);

    case Source::NodeChild:
    {
        VoxelState currentState = nodeChildState();

        if (currentState == VoxelState::Empty || currentState == VoxelState::Material)
        {
            subdivide();
            currentState = VoxelState::Branch;
        }

        const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);

        if (currentState == VoxelState::Branch)
        {
            return VoxelTreeEditor(*m_tree, m_tree->blockPool->node(index), corner);
        }

        MYVOXEL_ASSERT(currentState == VoxelState::MaskLeaf);
        return VoxelTreeEditor(*m_tree, m_tree->blockPool->leaf(index), corner);
    }

    case Source::MaskLeafGroup:
        return VoxelTreeEditor(*m_tree, *m_leafBlock, m_coarseCorner, corner);

    case Source::MaskLeafVoxel:
        MYVOXEL_REQUIRE_MESSAGE(false, "The highest-level voxel has no children.");
        break;
    }

    MYVOXEL_REQUIRE_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return VoxelTreeEditor(*m_tree);
}

/// 底层存储

bool VoxelTreeEditor::usesNodeBlock() const
{
    if (m_source == Source::Root)
    {
        return m_tree->rootState == VoxelRootState::Subdivided;
    }

    return m_source == Source::NodeChild && nodeChildState() == VoxelState::Branch;
}

bool VoxelTreeEditor::usesMaskLeaf() const
{
    if (m_source == Source::MaskLeafGroup || m_source == Source::MaskLeafVoxel)
    {
        return true;
    }

    return m_source == Source::NodeChild && nodeChildState() == VoxelState::MaskLeaf;
}

VoxelNodeBlock& VoxelTreeEditor::nodeBlock()
{
    MYVOXEL_REQUIRE_MESSAGE(usesNodeBlock(), "The current voxel does not use a VoxelNodeBlock.");
    return currentNodeBlock();
}

VoxelLeafBlock& VoxelTreeEditor::maskLeaf()
{
    MYVOXEL_REQUIRE_MESSAGE(usesMaskLeaf(), "The current voxel does not use a VoxelLeafBlock.");
    return currentLeafBlock();
}

/// 内部辅助

VoxelState VoxelTreeEditor::nodeChildState() const
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_parentBlock);

    return childState(*m_parentBlock, m_childCorner);
}

VoxelNodeBlock& VoxelTreeEditor::currentNodeBlock() const
{
    MYVOXEL_ASSERT(m_tree);

    if (m_source == Source::Root)
    {
        MYVOXEL_ASSERT(m_tree->rootState == VoxelRootState::Subdivided);
        return m_tree->rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelState::Branch);

    const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
    return m_tree->blockPool->node(index);
}

VoxelLeafBlock& VoxelTreeEditor::currentLeafBlock() const
{
    MYVOXEL_ASSERT(m_tree);

    if (m_source == Source::MaskLeafGroup || m_source == Source::MaskLeafVoxel)
    {
        MYVOXEL_ASSERT(m_leafBlock);
        return *m_leafBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelState::MaskLeaf);

    const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
    return m_tree->blockPool->leaf(index);
}

void VoxelTreeEditor::setTerminalState(VoxelTreeState newState)
{
    MYVOXEL_REQUIRE_MESSAGE(newState == VoxelTreeState::Empty || newState == VoxelTreeState::Material,
                            "A terminal state must be Empty or Material.");

    MYVOXEL_ASSERT(m_tree);

    const VoxelState targetState = terminalVoxelState(newState);
    VoxelBlockPool& pool = *m_tree->blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->rootState == VoxelRootState::Subdivided)
        {
            releaseNodeChildren(m_tree->rootBlock, pool);
        }

        reset(m_tree->rootBlock, targetState);
        m_tree->rootState = terminalRootState(newState);
        return;

    case Source::NodeChild:
    {
        const VoxelState currentState = nodeChildState();

        if (currentState == VoxelState::Branch)
        {
            const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
            releaseNodeChildren(pool.node(index), pool);
        }

        setNodeChildState(*m_parentBlock, m_childCorner, targetState);
        releaseUnusedParentStorage();
        return;
    }

    case Source::MaskLeafGroup:
        setLeafGroupMask(*m_leafBlock, m_coarseCorner,
                         newState == VoxelTreeState::Material ? static_cast<std::uint8_t>(0xFFU) : static_cast<std::uint8_t>(0));
        return;

    case Source::MaskLeafVoxel:
        setMaskBit(m_leafBlock->materialMask, m_coarseCorner, m_childCorner, newState == VoxelTreeState::Material);
        return;
    }

    MYVOXEL_REQUIRE_MESSAGE(false, "Unknown VoxelTreeEditor source.");
}

void VoxelTreeEditor::ensureParentStorage()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_parentBlock);

    if (m_parentBlock->storageMask != 0)
    {
        MYVOXEL_ASSERT(m_parentBlock->firstChildIndex != InvalidVoxelIndex);
        MYVOXEL_ASSERT(m_tree->blockPool->containsChildren(m_parentBlock->firstChildIndex));
        return;
    }

    MYVOXEL_ASSERT(m_parentBlock->firstChildIndex == InvalidVoxelIndex);
    m_parentBlock->firstChildIndex = m_tree->blockPool->allocateChildren();
}

void VoxelTreeEditor::releaseUnusedParentStorage()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_parentBlock);

    if (m_parentBlock->storageMask != 0)
    {
        return;
    }

    if (m_parentBlock->firstChildIndex == InvalidVoxelIndex)
    {
        return;
    }

    m_tree->blockPool->releaseChildren(m_parentBlock->firstChildIndex);
    m_parentBlock->firstChildIndex = InvalidVoxelIndex;
}

void VoxelTreeEditor::detachPool(VoxelTree& tree)
{
    MYVOXEL_REQUIRE_MESSAGE(tree.blockPool, "VoxelTree must contain a VoxelBlockPool.");

    if (tree.blockPool->referenceCount() <= 1)
    {
        return;
    }

    Foundation::RefPtr<VoxelBlockPool> detachedPool = Foundation::makeRef<VoxelBlockPool>();
    VoxelNodeBlock detachedRootBlock;

    if (tree.rootState == VoxelRootState::Subdivided)
    {
        cloneNodeBlock(tree.rootBlock, *tree.blockPool, detachedRootBlock, *detachedPool);
    }
    else
    {
        reset(detachedRootBlock, tree.rootState == VoxelRootState::Material ? VoxelState::Material : VoxelState::Empty);
    }

    tree.rootBlock = detachedRootBlock;
    tree.blockPool = detachedPool;
}

void VoxelTreeEditor::cloneNodeBlock(const VoxelNodeBlock& sourceBlock, const VoxelBlockPool& sourcePool,
                                     VoxelNodeBlock& targetBlock, VoxelBlockPool& targetPool)
{
    targetBlock.storageMask = sourceBlock.storageMask;
    targetBlock.leafMask = sourceBlock.leafMask;
    targetBlock.userData = sourceBlock.userData;
    targetBlock.firstChildIndex = InvalidVoxelIndex;

    if (sourceBlock.storageMask == 0)
    {
        MYVOXEL_ASSERT(sourceBlock.firstChildIndex == InvalidVoxelIndex);
        return;
    }

    MYVOXEL_ASSERT(sourceBlock.firstChildIndex != InvalidVoxelIndex);
    MYVOXEL_ASSERT(sourcePool.containsChildren(sourceBlock.firstChildIndex));

    targetBlock.firstChildIndex = targetPool.allocateChildren();

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);

        if (!hasChildStorage(sourceBlock, corner))
        {
            continue;
        }

        const VoxelIndex sourceIndex = sourceBlock.firstChildIndex + static_cast<VoxelIndex>(corner);
        const VoxelIndex targetIndex = targetBlock.firstChildIndex + static_cast<VoxelIndex>(corner);

        if (childState(sourceBlock, corner) == VoxelState::MaskLeaf)
        {
            VoxelLeafBlock& targetLeaf = targetPool.initializeLeaf(targetIndex);
            targetLeaf.materialMask = sourcePool.leaf(sourceIndex).materialMask;
            continue;
        }

        VoxelNodeBlock& targetChild = targetPool.initializeNode(targetIndex);
        cloneNodeBlock(sourcePool.node(sourceIndex), sourcePool, targetChild, targetPool);
    }
}

void VoxelTreeEditor::releaseNodeChildren(VoxelNodeBlock& block, VoxelBlockPool& pool)
{
    if (block.storageMask == 0)
    {
        MYVOXEL_ASSERT(block.firstChildIndex == InvalidVoxelIndex);
        return;
    }

    MYVOXEL_ASSERT(block.firstChildIndex != InvalidVoxelIndex);
    MYVOXEL_ASSERT(pool.containsChildren(block.firstChildIndex));

    const VoxelIndex firstChildIndex = block.firstChildIndex;

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);

        if (childState(block, corner) != VoxelState::Branch)
        {
            continue;
        }

        const VoxelIndex index = firstChildIndex + static_cast<VoxelIndex>(corner);
        releaseNodeChildren(pool.node(index), pool);
    }

    pool.releaseChildren(firstChildIndex);
    block.storageMask = 0;
    block.firstChildIndex = InvalidVoxelIndex;
}

void VoxelTreeEditor::setNodeChildState(VoxelNodeBlock& block, VoxelCorner corner, VoxelState state)
{
    switch (state)
    {
    case VoxelState::Empty:
        setMaskBit(block.storageMask, corner, false);
        setMaskBit(block.leafMask, corner, false);
        return;

    case VoxelState::Material:
        setMaskBit(block.storageMask, corner, false);
        setMaskBit(block.leafMask, corner, true);
        return;

    case VoxelState::Branch:
        setMaskBit(block.storageMask, corner, true);
        setMaskBit(block.leafMask, corner, false);
        return;

    case VoxelState::MaskLeaf:
        setMaskBit(block.storageMask, corner, true);
        setMaskBit(block.leafMask, corner, true);
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelState.");
}

void VoxelTreeEditor::setLeafGroupMask(VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner, std::uint8_t mask)
{
    const unsigned int offset = leafMaskOffset(coarseCorner);
    const std::uint64_t groupMask = static_cast<std::uint64_t>(0xFFULL << offset);
    const std::uint64_t shiftedMask = static_cast<std::uint64_t>(mask) << offset;

    leafBlock.materialMask = (leafBlock.materialMask & ~groupMask) | shiftedMask;
}

}