#include "VoxelTreeEditor.h"

#include "MyVoxel/Core/Mask/VoxelChildMask.h"

#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 将终止逻辑状态转换为普通节点存储状态。
MyVoxel::VoxelNodeState terminalNodeState(MyVoxel::VoxelState state)
{
    MYVOXEL_ASSERT(state == MyVoxel::VoxelState::Empty ||
                   state == MyVoxel::VoxelState::Material);

    return state == MyVoxel::VoxelState::Material
               ? MyVoxel::VoxelNodeState::Material
               : MyVoxel::VoxelNodeState::Empty;
}

// 递归释放一个普通节点块记录的全部物理后代。
void releaseDescendants(MyVoxel::VoxelNodeBlock& block, MyVoxel::VoxelBlockPool& pool)
{
    if (block.storageMask == 0)
    {
        MYVOXEL_ASSERT(block.firstChildIndex == MyVoxel::InvalidVoxelIndex);
        return;
    }

    MYVOXEL_ASSERT(block.firstChildIndex != MyVoxel::InvalidVoxelIndex);
    MYVOXEL_ASSERT(pool.containsGroup(block.firstChildIndex));

    const MyVoxel::VoxelIndex firstChildIndex = block.firstChildIndex;

    for (unsigned int cornerIndex = 0;
         cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner =
            static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        if (!MyVoxel::hasChildStorage(block, corner))
        {
            continue;
        }

        const MyVoxel::VoxelNodeState state =
            MyVoxel::nodeState(block, corner);

        if (state == MyVoxel::VoxelNodeState::Branch)
        {
            const MyVoxel::VoxelIndex index =
                MyVoxel::childStorageIndex(block, corner);

            releaseDescendants(pool.node(index), pool);
            continue;
        }

        MYVOXEL_ASSERT(state == MyVoxel::VoxelNodeState::MaskLeaf);
    }

    pool.releaseGroup(firstChildIndex);
    block.storageMask = 0;
    block.firstChildIndex = MyVoxel::InvalidVoxelIndex;
}

}

namespace MyVoxel
{

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::Root)
{
    MYVOXEL_ASSERT(tree.isValid());

    tree.ensureUniqueStorage();

    MYVOXEL_ASSERT(tree.isValid());
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree,
                                 VoxelNodeBlock& parentBlock,
                                 VoxelCorner childCorner)
    : m_tree(&tree)
    , m_parentBlock(&parentBlock)
    , m_leafBlock(nullptr)
    , m_childCorner(childCorner)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::NodeChild)
{
    cornerBit(childCorner);
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree,
                                 VoxelLeafBlock& leafBlock,
                                 VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_leafBlock(&leafBlock)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(coarseCorner)
    , m_source(Source::MaskLeafGroup)
{
    cornerBit(coarseCorner);
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree,
                                 VoxelLeafBlock& leafBlock,
                                 VoxelCorner coarseCorner,
                                 VoxelCorner fineCorner)
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

/// 体素状态

VoxelState VoxelTreeEditor::state() const
{
    MYVOXEL_ASSERT(m_tree);

    switch (m_source)
    {
    case Source::Root:
        return m_tree->m_rootState;

    case Source::NodeChild:
        return voxelState(nodeChildState());

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_leafBlock);
        return leafGroupState(*m_leafBlock, m_coarseCorner);

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT(m_leafBlock);

        return maskBit(m_leafBlock->materialMask, m_coarseCorner, m_childCorner)
                   ? VoxelState::Material
                   : VoxelState::Empty;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return VoxelState::Empty;
}



/// 子体素读取

VoxelChildStateMasks VoxelTreeEditor::childStateMasks() const
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);
    MYVOXEL_ASSERT(m_source != Source::MaskLeafVoxel);

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == VoxelState::Empty ||
            m_tree->m_rootState == VoxelState::Material)
        {
            return uniformChildStateMasks(m_tree->m_rootState);
        }

        return nodeChildStateMasks(m_tree->m_rootBlock);

    case Source::NodeChild:
    {
        const VoxelNodeState state = nodeChildState();

        if (state == VoxelNodeState::Empty ||
            state == VoxelNodeState::Material)
        {
            return uniformChildStateMasks(voxelState(state));
        }

        if (state == VoxelNodeState::Branch)
        {
            return nodeChildStateMasks(currentNodeBlock());
        }

        MYVOXEL_ASSERT(state == VoxelNodeState::MaskLeaf);
        return leafChildStateMasks(currentLeafBlock());
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_leafBlock);
        return leafGroupChildStateMasks(*m_leafBlock, m_coarseCorner);

    case Source::MaskLeafVoxel:
        break;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "A mask-leaf fine voxel has no child state mask.");
    return uniformChildStateMasks(VoxelState::Empty);
}

/// 体素修改

void VoxelTreeEditor::setEmpty()
{
    setTerminal(VoxelState::Empty);
}

void VoxelTreeEditor::setMaterial()
{
    setTerminal(VoxelState::Material);
}

bool VoxelTreeEditor::subdivide()
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    VoxelBlockPool& pool = *m_tree->m_blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            return false;
        }

        MyVoxel::reset(
            m_tree->m_rootBlock,
            terminalNodeState(m_tree->m_rootState));

        m_tree->m_rootState = VoxelState::Subdivided;
        return true;

    case Source::NodeChild:
    {
        const VoxelNodeState state = nodeChildState();

        if (state == VoxelNodeState::Branch ||
            state == VoxelNodeState::MaskLeaf)
        {
            return false;
        }

        ensureParentGroup();

        const VoxelIndex index =
            m_parentBlock->firstChildIndex +
            static_cast<VoxelIndex>(m_childCorner);

        pool.initializeNode(index, state);
        setNodeStateBits(*m_parentBlock, m_childCorner, VoxelNodeState::Branch);
        return true;
    }

    case Source::MaskLeafGroup:
        return false;

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT_MESSAGE(false, "A mask-leaf fine voxel cannot be subdivided.");
        return false;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return false;
}

bool VoxelTreeEditor::setChildrenState(std::uint8_t childMask, VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty ||
                   newState == VoxelState::Material);

    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);
    MYVOXEL_ASSERT(m_source != Source::MaskLeafVoxel);

    if (childMask == EmptyVoxelNodeMask)
    {
        return false;
    }

    const VoxelChildStateMasks currentStates = childStateMasks();

    const std::uint8_t changedMask =
        static_cast<std::uint8_t>(
            childMask &
            static_cast<std::uint8_t>(
                ~currentStates.stateMask(newState)));

    if (changedMask == EmptyVoxelNodeMask)
    {
        return false;
    }

    switch (m_source)
    {
    case Source::Root:
        subdivide();
        return setNodeChildren(m_tree->m_rootBlock, changedMask, newState);

    case Source::NodeChild:
    {
        VoxelNodeState state = nodeChildState();

        if (state == VoxelNodeState::Empty ||
            state == VoxelNodeState::Material)
        {
            subdivide();
            state = nodeChildState();
        }

        if (state == VoxelNodeState::Branch)
        {
            return setNodeChildren(currentNodeBlock(), changedMask, newState);
        }

        MYVOXEL_ASSERT(state == VoxelNodeState::MaskLeaf);
        return setLeafGroupStates(currentLeafBlock(), changedMask, newState);
    }

    case Source::MaskLeafGroup:
    {
        MYVOXEL_ASSERT(m_leafBlock);

        const unsigned int offset = leafMaskOffset(m_coarseCorner);

        const std::uint64_t materialMask =
            static_cast<std::uint64_t>(changedMask) << offset;

        return setLeafMaterialBits(
            *m_leafBlock,
            materialMask,
            newState == VoxelState::Material);
    }

    case Source::MaskLeafVoxel:
        break;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "A mask-leaf fine voxel has no children to modify.");
    return false;
}

VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner)
{
    cornerBit(corner);

    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    switch (m_source)
    {
    case Source::Root:
        subdivide();
        return VoxelTreeEditor(*m_tree, m_tree->m_rootBlock, corner);

    case Source::NodeChild:
    {
        VoxelNodeState state = nodeChildState();

        if (state == VoxelNodeState::Empty ||
            state == VoxelNodeState::Material)
        {
            subdivide();
            state = nodeChildState();
        }

        MYVOXEL_ASSERT(state == VoxelNodeState::Branch ||
                       state == VoxelNodeState::MaskLeaf);

        const VoxelIndex index =
            childStorageIndex(*m_parentBlock, m_childCorner);

        if (state == VoxelNodeState::Branch)
        {
            return VoxelTreeEditor(
                *m_tree,
                m_tree->m_blockPool->node(index),
                corner);
        }

        return VoxelTreeEditor(
            *m_tree,
            m_tree->m_blockPool->leaf(index),
            corner);
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_leafBlock);

        return VoxelTreeEditor(
            *m_tree,
            *m_leafBlock,
            m_coarseCorner,
            corner);

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT_MESSAGE(false, "A mask-leaf fine voxel has no children.");
        return *this;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return *this;
}

/// 压缩材料修改


bool VoxelTreeEditor::setMaterialMask(std::uint64_t materialMask)
{
    MYVOXEL_ASSERT(canSetMaterialMask());
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_parentBlock);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (materialMask == EmptyVoxelLeafMask)
    {
        const bool changed = !isEmpty();

        if (changed)
        {
            setEmpty();
        }

        return changed;
    }

    if (materialMask == FullVoxelLeafMask)
    {
        const bool changed = !isMaterial();

        if (changed)
        {
            setMaterial();
        }

        return changed;
    }

    const VoxelNodeState state = nodeChildState();
    VoxelBlockPool& pool = *m_tree->m_blockPool;

    if (state == VoxelNodeState::MaskLeaf)
    {
        return setLeafMaterialMask(currentLeafBlock(), materialMask);
    }

    VoxelIndex index = InvalidVoxelIndex;

    if (state == VoxelNodeState::Branch)
    {
        index = childStorageIndex(*m_parentBlock, m_childCorner);
        releaseDescendants(pool.node(index), pool);
    }
    else
    {
        MYVOXEL_ASSERT(state == VoxelNodeState::Empty ||
                       state == VoxelNodeState::Material);

        ensureParentGroup();

        index =
            m_parentBlock->firstChildIndex +
            static_cast<VoxelIndex>(m_childCorner);
    }

    VoxelLeafBlock& leaf = pool.initializeLeaf(index);
    leaf.materialMask = materialMask;

    setNodeStateBits(
        *m_parentBlock,
        m_childCorner,
        VoxelNodeState::MaskLeaf);

    return true;
}

/// 内部辅助

VoxelNodeState VoxelTreeEditor::nodeChildState() const
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_parentBlock);

    return nodeState(*m_parentBlock, m_childCorner);
}

VoxelNodeBlock& VoxelTreeEditor::currentNodeBlock()
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_source == Source::Root)
    {
        MYVOXEL_ASSERT(m_tree->m_rootState == VoxelState::Subdivided);
        return m_tree->m_rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::Branch);

    const VoxelIndex index =
        childStorageIndex(*m_parentBlock, m_childCorner);

    return m_tree->m_blockPool->node(index);
}

const VoxelNodeBlock& VoxelTreeEditor::currentNodeBlock() const
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_source == Source::Root)
    {
        MYVOXEL_ASSERT(m_tree->m_rootState == VoxelState::Subdivided);
        return m_tree->m_rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::Branch);

    const VoxelIndex index =
        childStorageIndex(*m_parentBlock, m_childCorner);

    const VoxelBlockPool& pool = *m_tree->m_blockPool;
    return pool.node(index);
}

VoxelLeafBlock& VoxelTreeEditor::currentLeafBlock()
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_source == Source::MaskLeafGroup ||
        m_source == Source::MaskLeafVoxel)
    {
        MYVOXEL_ASSERT(m_leafBlock);
        return *m_leafBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::MaskLeaf);

    const VoxelIndex index =
        childStorageIndex(*m_parentBlock, m_childCorner);

    return m_tree->m_blockPool->leaf(index);
}

const VoxelLeafBlock& VoxelTreeEditor::currentLeafBlock() const
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_source == Source::MaskLeafGroup ||
        m_source == Source::MaskLeafVoxel)
    {
        MYVOXEL_ASSERT(m_leafBlock);
        return *m_leafBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::MaskLeaf);

    const VoxelIndex index =
        childStorageIndex(*m_parentBlock, m_childCorner);

    const VoxelBlockPool& pool = *m_tree->m_blockPool;
    return pool.leaf(index);
}

void VoxelTreeEditor::setTerminal(VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty ||
                   newState == VoxelState::Material);

    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (state() == newState)
    {
        return;
    }

    const VoxelNodeState targetState = terminalNodeState(newState);
    VoxelBlockPool& pool = *m_tree->m_blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            releaseDescendants(m_tree->m_rootBlock, pool);
        }

        MyVoxel::reset(m_tree->m_rootBlock, targetState);
        m_tree->m_rootState = newState;
        return;

    case Source::NodeChild:
    {
        const VoxelNodeState state = nodeChildState();

        if (state == VoxelNodeState::Branch)
        {
            const VoxelIndex index =
                childStorageIndex(*m_parentBlock, m_childCorner);

            releaseDescendants(pool.node(index), pool);
        }

        setNodeStateBits(*m_parentBlock, m_childCorner, targetState);
        releaseUnusedParentGroup();
        return;
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_leafBlock);
        setLeafGroupState(*m_leafBlock, m_coarseCorner, newState);
        return;

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT(m_leafBlock);

        setMaskBit(
            m_leafBlock->materialMask,
            m_coarseCorner,
            m_childCorner,
            newState == VoxelState::Material);

        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
}

bool VoxelTreeEditor::setNodeChildren(VoxelNodeBlock& block,
                                      std::uint8_t childMask,
                                      VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty ||
                   newState == VoxelState::Material);

    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    const VoxelNodeState targetState = terminalNodeState(newState);

    const std::uint8_t changedMask =
        static_cast<std::uint8_t>(
            childMask &
            static_cast<std::uint8_t>(
                ~nodeStateMask(block, targetState)));

    if (changedMask == EmptyVoxelNodeMask)
    {
        return false;
    }

    VoxelBlockPool& pool = *m_tree->m_blockPool;

    std::uint8_t storageMask =
        static_cast<std::uint8_t>(
            block.storageMask & changedMask);

    while (storageMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(storageMask);
        const VoxelNodeState state = nodeState(block, corner);

        if (state == VoxelNodeState::Branch)
        {
            const VoxelIndex index = childStorageIndex(block, corner);
            releaseDescendants(pool.node(index), pool);
        }
        else
        {
            MYVOXEL_ASSERT(state == VoxelNodeState::MaskLeaf);
        }
    }

    setNodeStateBits(block, changedMask, targetState);

    if (block.storageMask == EmptyVoxelNodeMask &&
        block.firstChildIndex != InvalidVoxelIndex)
    {
        pool.releaseGroup(block.firstChildIndex);
        block.firstChildIndex = InvalidVoxelIndex;
    }

    return true;
}

void VoxelTreeEditor::ensureParentGroup()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_parentBlock);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_parentBlock->storageMask != 0)
    {
        MYVOXEL_ASSERT(
            m_parentBlock->firstChildIndex != InvalidVoxelIndex);

        MYVOXEL_ASSERT(
            m_tree->m_blockPool->containsGroup(
                m_parentBlock->firstChildIndex));

        return;
    }

    MYVOXEL_ASSERT(
        m_parentBlock->firstChildIndex == InvalidVoxelIndex);

    m_parentBlock->firstChildIndex =
        m_tree->m_blockPool->allocateGroup();
}

void VoxelTreeEditor::releaseUnusedParentGroup()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_parentBlock);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    if (m_parentBlock->storageMask != 0 ||
        m_parentBlock->firstChildIndex == InvalidVoxelIndex)
    {
        return;
    }

    m_tree->m_blockPool->releaseGroup(
        m_parentBlock->firstChildIndex);

    m_parentBlock->firstChildIndex = InvalidVoxelIndex;
}

}