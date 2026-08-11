#include "VoxelTreeEditor.h"

#include <limits>

#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 将终止逻辑状态转换为NodeBlock物理状态。
MyVoxel::VoxelNodeState terminalNodeState(MyVoxel::VoxelState state)
{
    MYVOXEL_ASSERT(state == MyVoxel::VoxelState::Empty || state == MyVoxel::VoxelState::Material);
    return state == MyVoxel::VoxelState::Material ? MyVoxel::VoxelNodeState::Material : MyVoxel::VoxelNodeState::Empty;
}

// 返回Empty或Material终止状态对应的隐式距离。
// Empty使用正无穷，Material使用负无穷，使SDF布尔min/max保持正确代数语义。
float terminalDistance(MyVoxel::VoxelState state)
{
    MYVOXEL_ASSERT(state == MyVoxel::VoxelState::Empty || state == MyVoxel::VoxelState::Material);
    const float infinity = (std::numeric_limits<float>::infinity)();
    return state == MyVoxel::VoxelState::Material ? -infinity : infinity;
}

// 根据64个距离样本生成材料符号缓存，distance<=0表示材料。
std::uint64_t materialMaskFromLeaf(const MyVoxel::LeafBlock& block)
{
    std::uint64_t materialMask = 0;

    for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::LeafBlockSampleCount; ++sampleIndex)
    {
        const float distance = block.distances[sampleIndex];

        MYVOXEL_ASSERT_MESSAGE(distance == distance, "Voxel leaf distance cannot be NaN.");

        if (distance <= 0.0f)
        {
            materialMask |= static_cast<std::uint64_t>(1ULL) << sampleIndex;
        }
    }

    return materialMask;
}

// 判断两个距离叶块内容是否完全一致。
bool leafBlocksEqual(const MyVoxel::LeafBlock& first, const MyVoxel::LeafBlock& second)
{
    for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::LeafBlockSampleCount; ++sampleIndex)
    {
        if (first.distances[sampleIndex] != second.distances[sampleIndex])
        {
            return false;
        }
    }

    return true;
}

// 判断LeafBlock是否可以无损退化为Empty或Material终止节点。
bool leafTerminalState(const MyVoxel::LeafBlock& block, MyVoxel::VoxelState& state)
{
    const float infinity = (std::numeric_limits<float>::infinity)();
    bool allEmpty = true;
    bool allMaterial = true;

    for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::LeafBlockSampleCount; ++sampleIndex)
    {
        const float distance = block.distances[sampleIndex];

        if (distance != infinity)
        {
            allEmpty = false;
        }

        if (distance != -infinity)
        {
            allMaterial = false;
        }

        if (!allEmpty && !allMaterial)
        {
            return false;
        }
    }

    state = allMaterial ? MyVoxel::VoxelState::Material : MyVoxel::VoxelState::Empty;
    return true;
}

// 递归释放一个NodeBlock记录的全部物理后代。
void releaseDescendants(MyVoxel::NodeBlock& block, MyVoxel::BlockPool& pool)
{
    if (block.storageMask == 0)
    {
        MYVOXEL_ASSERT(block.firstChildIndex == MyVoxel::InvalidVoxelIndex);
        return;
    }

    MYVOXEL_ASSERT(block.firstChildIndex != MyVoxel::InvalidVoxelIndex);
    MYVOXEL_ASSERT(pool.containsGroup(block.firstChildIndex));

    const MyVoxel::VoxelIndex firstChildIndex = block.firstChildIndex;

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        if (!MyVoxel::hasChildStorage(block, corner))
        {
            continue;
        }

        const MyVoxel::VoxelNodeState state = MyVoxel::nodeState(block, corner);
        const MyVoxel::VoxelIndex index = MyVoxel::childStorageIndex(block, corner);

        if (state == MyVoxel::VoxelNodeState::Node)
        {
            releaseDescendants(pool.node(index), pool);
        }
        else
        {
            MYVOXEL_ASSERT(state == MyVoxel::VoxelNodeState::MaskLeaf);
            pool.releaseLeaf(index);
        }
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
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::Root)
{
    MYVOXEL_ASSERT(tree.isValid());
    tree.ensureUniqueStorage();
    MYVOXEL_ASSERT(tree.isValid());
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, NodeBlock& parentBlock, VoxelCorner childCorner)
    : m_tree(&tree)
    , m_parentBlock(&parentBlock)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(childCorner)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_source(Source::NodeChild)
{
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlockValue, LeafBlock& leafBlockValue, VoxelCorner coarseCorner)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(coarseCorner)
    , m_source(Source::MaskLeafGroup)
{
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlockValue, LeafBlock& leafBlockValue, VoxelCorner coarseCorner, VoxelCorner fineCorner)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_childCorner(fineCorner)
    , m_coarseCorner(coarseCorner)
    , m_source(Source::MaskLeafVoxel)
{
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
        MYVOXEL_ASSERT(m_maskBlock);
        return leafGroupState(*m_maskBlock, m_coarseCorner);

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT(m_maskBlock);
        return leafMaterialBit(*m_maskBlock, m_coarseCorner, m_childCorner) ? VoxelState::Material : VoxelState::Empty;
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
        if (m_tree->m_rootState == VoxelState::Empty || m_tree->m_rootState == VoxelState::Material)
        {
            return uniformChildStateMasks(m_tree->m_rootState);
        }

        return nodeChildStateMasks(m_tree->m_rootBlock);

    case Source::NodeChild:
    {
        const VoxelNodeState currentState = nodeChildState();

        if (currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material)
        {
            return uniformChildStateMasks(voxelState(currentState));
        }

        if (currentState == VoxelNodeState::Node)
        {
            return nodeChildStateMasks(currentNodeBlock());
        }

        MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);
        return leafChildStateMasks(*currentLeafData().mask);
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_maskBlock);
        return leafGroupChildStateMasks(*m_maskBlock, m_coarseCorner);

    case Source::MaskLeafVoxel:
        break;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "A MaskLeaf fine voxel has no child state mask.");
    return uniformChildStateMasks(VoxelState::Empty);
}

/// 结构和终止状态修改

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

    BlockPool& pool = *m_tree->m_blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            return false;
        }

        MyVoxel::reset(m_tree->m_rootBlock, terminalNodeState(m_tree->m_rootState));
        m_tree->m_rootState = VoxelState::Subdivided;
        return true;

    case Source::NodeChild:
    {
        const VoxelNodeState currentState = nodeChildState();

        if (currentState == VoxelNodeState::Node || currentState == VoxelNodeState::MaskLeaf)
        {
            return false;
        }

        ensureParentGroup();

        const VoxelIndex index = m_parentBlock->firstChildIndex + static_cast<VoxelIndex>(m_childCorner);

        pool.initializeNode(index, currentState);
        setNodeStateBits(*m_parentBlock, m_childCorner, VoxelNodeState::Node);
        return true;
    }

    case Source::MaskLeafGroup:
        return false;

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT_MESSAGE(false, "A MaskLeaf distance sample cannot be subdivided.");
        return false;
    }

    return false;
}

bool VoxelTreeEditor::setChildrenState(std::uint8_t childMask, VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty || newState == VoxelState::Material);
    MYVOXEL_ASSERT(m_source != Source::MaskLeafVoxel);

    if (childMask == EmptyVoxelNodeMask)
    {
        return false;
    }

    const VoxelChildStateMasks currentStates = childStateMasks();
    const std::uint8_t changedMask = static_cast<std::uint8_t>(childMask & static_cast<std::uint8_t>(~currentStates.stateMask(newState)));

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
        VoxelNodeState currentState = nodeChildState();

        if (currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material)
        {
            subdivide();
            currentState = nodeChildState();
        }

        if (currentState == VoxelNodeState::Node)
        {
            return setNodeChildren(currentNodeBlock(), changedMask, newState);
        }

        MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);
        return setLeafGroups(changedMask, newState);
    }

    case Source::MaskLeafGroup:
        return setLeafGroupChildren(changedMask, newState);

    case Source::MaskLeafVoxel:
        break;
    }

    return false;
}

VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner)
{
    MYVOXEL_ASSERT(m_tree);
    MYVOXEL_ASSERT(m_tree->m_blockPool);

    switch (m_source)
    {
    case Source::Root:
        subdivide();
        return VoxelTreeEditor(*m_tree, m_tree->m_rootBlock, corner);

    case Source::NodeChild:
    {
        VoxelNodeState currentState = nodeChildState();

        if (currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material)
        {
            subdivide();
            currentState = nodeChildState();
        }

        const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);

        if (currentState == VoxelNodeState::Node)
        {
            return VoxelTreeEditor(*m_tree, m_tree->m_blockPool->node(index), corner);
        }

        MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);

        LeafData leaf = m_tree->m_blockPool->leaf(index);
        return VoxelTreeEditor(*m_tree, *leaf.mask, *leaf.block, corner);
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_maskBlock && m_leafBlock);
        return VoxelTreeEditor(*m_tree, *m_maskBlock, *m_leafBlock, m_coarseCorner, corner);

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT_MESSAGE(false, "A MaskLeaf distance sample has no children.");
        return *this;
    }

    return *this;
}

/// MaskLeaf修改

bool VoxelTreeEditor::setLeafBlock(const LeafBlock& block)
{
    MYVOXEL_ASSERT(canSetLeafBlock());
    MYVOXEL_ASSERT(m_tree && m_parentBlock && m_tree->m_blockPool);

    VoxelState terminalState = VoxelState::Empty;

    if (leafTerminalState(block, terminalState))
    {
        const bool changed = state() != terminalState;

        if (changed)
        {
            setTerminal(terminalState);
        }

        return changed;
    }

    const std::uint64_t materialMask = materialMaskFromLeaf(block);
    const VoxelNodeState currentState = nodeChildState();
    BlockPool& pool = *m_tree->m_blockPool;

    if (currentState == VoxelNodeState::MaskLeaf)
    {
        LeafData leaf = currentLeafData();

        if (leaf.mask->materialMask == materialMask && leafBlocksEqual(*leaf.block, block))
        {
            return false;
        }

        *leaf.block = block;
        leaf.mask->materialMask = materialMask;
        return true;
    }

    VoxelIndex index = InvalidVoxelIndex;

    if (currentState == VoxelNodeState::Node)
    {
        index = childStorageIndex(*m_parentBlock, m_childCorner);
        releaseDescendants(pool.node(index), pool);
    }
    else
    {
        MYVOXEL_ASSERT(currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material);
        ensureParentGroup();
        index = m_parentBlock->firstChildIndex + static_cast<VoxelIndex>(m_childCorner);
    }

    LeafData leaf = pool.initializeLeaf(index, terminalDistance(VoxelState::Empty));
    *leaf.block = block;
    leaf.mask->materialMask = materialMask;

    setNodeStateBits(*m_parentBlock, m_childCorner, VoxelNodeState::MaskLeaf);
    return true;
}

bool VoxelTreeEditor::setMaterialMask(std::uint64_t materialMask)
{
    MYVOXEL_ASSERT(canSetLeafBlock());

    LeafBlock block;
    const float emptyDistance = terminalDistance(VoxelState::Empty);
    const float materialDistance = terminalDistance(VoxelState::Material);

    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        block.distances[sampleIndex] = (materialMask & (static_cast<std::uint64_t>(1ULL) << sampleIndex)) != 0 ? materialDistance : emptyDistance;
    }

    return setLeafBlock(block);
}

/// 单距离修改

float VoxelTreeEditor::distance() const
{
    MYVOXEL_ASSERT(canSetDistance());
    MYVOXEL_ASSERT(m_leafBlock);

    return m_leafBlock->distances[leafBitIndex(m_coarseCorner, m_childCorner)];
}

bool VoxelTreeEditor::setDistance(float newDistance)
{
    MYVOXEL_ASSERT(canSetDistance());
    MYVOXEL_ASSERT(m_maskBlock && m_leafBlock);
    MYVOXEL_ASSERT_MESSAGE(newDistance == newDistance, "Voxel distance cannot be NaN.");

    const unsigned int sampleIndex = leafBitIndex(m_coarseCorner, m_childCorner);

    if (m_leafBlock->distances[sampleIndex] == newDistance)
    {
        return false;
    }

    setLeafDistance(*m_maskBlock, *m_leafBlock, sampleIndex, newDistance);
    return true;
}

/// 内部辅助

VoxelNodeState VoxelTreeEditor::nodeChildState() const
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_parentBlock);
    return nodeState(*m_parentBlock, m_childCorner);
}

NodeBlock& VoxelTreeEditor::currentNodeBlock()
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    if (m_source == Source::Root)
    {
        MYVOXEL_ASSERT(m_tree->m_rootState == VoxelState::Subdivided);
        return m_tree->m_rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::Node);

    return m_tree->m_blockPool->node(childStorageIndex(*m_parentBlock, m_childCorner));
}

const NodeBlock& VoxelTreeEditor::currentNodeBlock() const
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    if (m_source == Source::Root)
    {
        return m_tree->m_rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::Node);

    return m_tree->m_blockPool->node(childStorageIndex(*m_parentBlock, m_childCorner));
}

LeafData VoxelTreeEditor::currentLeafData()
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    if (m_source == Source::MaskLeafGroup || m_source == Source::MaskLeafVoxel)
    {
        LeafData result;
        result.mask = m_maskBlock;
        result.block = m_leafBlock;
        return result;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::MaskLeaf);

    return m_tree->m_blockPool->leaf(childStorageIndex(*m_parentBlock, m_childCorner));
}

ConstLeafData VoxelTreeEditor::currentLeafData() const
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    if (m_source == Source::MaskLeafGroup || m_source == Source::MaskLeafVoxel)
    {
        ConstLeafData result;
        result.mask = m_maskBlock;
        result.block = m_leafBlock;
        return result;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(nodeChildState() == VoxelNodeState::MaskLeaf);

    return m_tree->m_blockPool->leaf(childStorageIndex(*m_parentBlock, m_childCorner));
}

void VoxelTreeEditor::setTerminal(VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty || newState == VoxelState::Material);

    const float distanceValue = terminalDistance(newState);
    BlockPool& pool = *m_tree->m_blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == newState)
        {
            return;
        }

        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            releaseDescendants(m_tree->m_rootBlock, pool);
        }

        MyVoxel::reset(m_tree->m_rootBlock, terminalNodeState(newState));
        m_tree->m_rootState = newState;
        return;

    case Source::NodeChild:
    {
        if (state() == newState)
        {
            return;
        }

        const VoxelNodeState currentState = nodeChildState();

        if (currentState == VoxelNodeState::Node)
        {
            releaseDescendants(pool.node(childStorageIndex(*m_parentBlock, m_childCorner)), pool);
        }
        else if (currentState == VoxelNodeState::MaskLeaf)
        {
            pool.releaseLeaf(childStorageIndex(*m_parentBlock, m_childCorner));
        }

        setNodeStateBits(*m_parentBlock, m_childCorner, terminalNodeState(newState));
        releaseUnusedParentGroup();
        return;
    }

    case Source::MaskLeafGroup:
        setLeafGroups(nodeCornerMask(m_coarseCorner), newState);
        return;

    case Source::MaskLeafVoxel:
    {
        const unsigned int sampleIndex = leafBitIndex(m_coarseCorner, m_childCorner);

        if (m_leafBlock->distances[sampleIndex] != distanceValue)
        {
            setLeafDistance(*m_maskBlock, *m_leafBlock, sampleIndex, distanceValue);
        }

        return;
    }
    }
}

bool VoxelTreeEditor::setNodeChildren(NodeBlock& block, std::uint8_t childMask, VoxelState newState)
{
    const VoxelNodeState targetState = terminalNodeState(newState);
    const std::uint8_t changedMask = static_cast<std::uint8_t>(childMask & static_cast<std::uint8_t>(~nodeStateMask(block, targetState)));

    if (changedMask == EmptyVoxelNodeMask)
    {
        return false;
    }

    BlockPool& pool = *m_tree->m_blockPool;
    std::uint8_t storageMask = static_cast<std::uint8_t>(block.storageMask & changedMask);

    while (storageMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(storageMask);
        const VoxelNodeState currentState = nodeState(block, corner);
        const VoxelIndex index = childStorageIndex(block, corner);

        if (currentState == VoxelNodeState::Node)
        {
            releaseDescendants(pool.node(index), pool);
        }
        else
        {
            MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);
            pool.releaseLeaf(index);
        }
    }

    setNodeStateBits(block, changedMask, targetState);

    if (block.storageMask == EmptyVoxelNodeMask && block.firstChildIndex != InvalidVoxelIndex)
    {
        pool.releaseGroup(block.firstChildIndex);
        block.firstChildIndex = InvalidVoxelIndex;
    }

    return true;
}

bool VoxelTreeEditor::setLeafGroups(std::uint8_t groupMask, VoxelState newState)
{
    LeafData leaf = currentLeafData();
    const float distanceValue = terminalDistance(newState);
    bool changed = false;

    for (unsigned int coarseIndex = 0; coarseIndex < static_cast<unsigned int>(VoxelCornerCount); ++coarseIndex)
    {
        if ((groupMask & static_cast<std::uint8_t>(1U << coarseIndex)) == 0)
        {
            continue;
        }

        for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(VoxelCornerCount); ++fineIndex)
        {
            const unsigned int sampleIndex = coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;

            if (leaf.block->distances[sampleIndex] == distanceValue)
            {
                continue;
            }

            setLeafDistance(*leaf.mask, *leaf.block, sampleIndex, distanceValue);
            changed = true;
        }
    }

    return changed;
}

bool VoxelTreeEditor::setLeafGroupChildren(std::uint8_t childMask, VoxelState newState)
{
    MYVOXEL_ASSERT(m_maskBlock && m_leafBlock);

    const float distanceValue = terminalDistance(newState);
    const unsigned int coarseIndex = static_cast<unsigned int>(m_coarseCorner);
    bool changed = false;

    for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(VoxelCornerCount); ++fineIndex)
    {
        if ((childMask & static_cast<std::uint8_t>(1U << fineIndex)) == 0)
        {
            continue;
        }

        const unsigned int sampleIndex = coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;

        if (m_leafBlock->distances[sampleIndex] == distanceValue)
        {
            continue;
        }

        setLeafDistance(*m_maskBlock, *m_leafBlock, sampleIndex, distanceValue);
        changed = true;
    }

    return changed;
}

void VoxelTreeEditor::ensureParentGroup()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_tree && m_parentBlock && m_tree->m_blockPool);

    if (m_parentBlock->storageMask != 0)
    {
        MYVOXEL_ASSERT(m_parentBlock->firstChildIndex != InvalidVoxelIndex);
        MYVOXEL_ASSERT(m_tree->m_blockPool->containsGroup(m_parentBlock->firstChildIndex));
        return;
    }

    MYVOXEL_ASSERT(m_parentBlock->firstChildIndex == InvalidVoxelIndex);
    m_parentBlock->firstChildIndex = m_tree->m_blockPool->allocateGroup();
}

void VoxelTreeEditor::releaseUnusedParentGroup()
{
    MYVOXEL_ASSERT(m_source == Source::NodeChild);
    MYVOXEL_ASSERT(m_tree && m_parentBlock && m_tree->m_blockPool);

    if (m_parentBlock->storageMask != 0 || m_parentBlock->firstChildIndex == InvalidVoxelIndex)
    {
        return;
    }

    m_tree->m_blockPool->releaseGroup(m_parentBlock->firstChildIndex);
    m_parentBlock->firstChildIndex = InvalidVoxelIndex;
}

}