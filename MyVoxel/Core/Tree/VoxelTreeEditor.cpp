#include "VoxelTreeEditor.h"

#include <cmath>

#include "MyVoxel/Core/Mask/MaskUtils.h"
#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

MyVoxel::VoxelNodeState terminalNodeState(float value)
{
    return value <= 0.0f ? MyVoxel::VoxelNodeState::Material : MyVoxel::VoxelNodeState::Empty;
}

MyVoxel::VoxelState terminalState(float value)
{
    return value <= 0.0f ? MyVoxel::VoxelState::Material : MyVoxel::VoxelState::Empty;
}

bool isValidBackgroundDistance(float backgroundDistance)
{
    return std::isfinite(static_cast<double>(backgroundDistance)) && backgroundDistance > 0.0f;
}

bool isValidTsdfDistance(float distance, float backgroundDistance)
{
    return std::isfinite(static_cast<double>(distance)) && distance >= -backgroundDistance && distance <= backgroundDistance;
}

void resetRootTerminalBlock(MyVoxel::NodeBlock& block, float value)
{
    block.mChildMask = 0;
    block.mValueMask = 0;
    block.reserved = 0;
    block.nodeData.value = value;
}

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

bool leafUniformValue(const MyVoxel::LeafBlock& block, float& value)
{
    value = block.distances[0];
    for (unsigned int sampleIndex = 1; sampleIndex < MyVoxel::LeafBlockSampleCount; ++sampleIndex)
    {
        if (block.distances[sampleIndex] != value)
        {
            return false;
        }
    }
    return true;
}

void releaseDescendants(MyVoxel::NodeBlock& block, MyVoxel::BlockPool& pool)
{
    MYVOXEL_ASSERT(MyVoxel::hasChildStorage(block));
    MYVOXEL_ASSERT(pool.containsGroup(block.nodeData.firstChildIndex));

    const MyVoxel::VoxelIndex firstChildIndex = block.nodeData.firstChildIndex;
    std::uint8_t subdividedMask = block.mChildMask;

    while (subdividedMask != 0)
    {
        const MyVoxel::VoxelCorner corner = MyVoxel::takeFirstNodeCorner(subdividedMask);
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
    MyVoxel::clearChildStorage(block);
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
    , m_backgroundDistance(0.0f)
    , m_source(Source::Root)
{
    MYVOXEL_ASSERT(tree.isValid());
    tree.ensureUniqueStorage();
    MYVOXEL_ASSERT(tree.isValid());
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, float backgroundDistanceValue)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_backgroundDistance(backgroundDistanceValue)
    , m_source(Source::Root)
{
    MYVOXEL_ASSERT(tree.isValid());
    MYVOXEL_REQUIRE_MESSAGE(isValidBackgroundDistance(backgroundDistanceValue), "VoxelTreeEditor background distance must be finite and positive.");
    tree.ensureUniqueStorage();
    MYVOXEL_ASSERT(tree.isValid());
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, NodeBlock& parentBlock, VoxelCorner childCorner, float backgroundDistanceValue)
    : m_tree(&tree)
    , m_parentBlock(&parentBlock)
    , m_maskBlock(nullptr)
    , m_leafBlock(nullptr)
    , m_childCorner(childCorner)
    , m_coarseCorner(VoxelCorner::Minimum)
    , m_backgroundDistance(backgroundDistanceValue)
    , m_source(Source::NodeChild)
{
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlockValue, LeafBlock& leafBlockValue, VoxelCorner coarseCorner, float backgroundDistanceValue)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_childCorner(VoxelCorner::Minimum)
    , m_coarseCorner(coarseCorner)
    , m_backgroundDistance(backgroundDistanceValue)
    , m_source(Source::MaskLeafGroup)
{
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlockValue, LeafBlock& leafBlockValue, VoxelCorner coarseCorner, VoxelCorner fineCorner, float backgroundDistanceValue)
    : m_tree(&tree)
    , m_parentBlock(nullptr)
    , m_maskBlock(&maskBlockValue)
    , m_leafBlock(&leafBlockValue)
    , m_childCorner(fineCorner)
    , m_coarseCorner(coarseCorner)
    , m_backgroundDistance(backgroundDistanceValue)
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
        return VoxelState::Subdivided;
    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT(m_maskBlock);
        return leafMaterialBit(*m_maskBlock, m_coarseCorner, m_childCorner) ? VoxelState::Material : VoxelState::Empty;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown VoxelTreeEditor source.");
    return VoxelState::Empty;
}

float VoxelTreeEditor::backgroundDistance() const
{
    requireBackgroundDistance();
    return m_backgroundDistance;
}

/// 子体素读取

VoxelChildStateMasks VoxelTreeEditor::childStateMasks() const
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);
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

    MYVOXEL_ASSERT_MESSAGE(false, "A MaskLeaf distance sample has no child state mask.");
    return uniformChildStateMasks(VoxelState::Empty);
}

/// 结构和终止Value修改

void VoxelTreeEditor::setEmpty()
{
    requireBackgroundDistance();
    setValue(m_backgroundDistance);
}

void VoxelTreeEditor::setMaterial()
{
    requireBackgroundDistance();
    setValue(-m_backgroundDistance);
}

bool VoxelTreeEditor::setValue(float value)
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);
    MYVOXEL_REQUIRE_MESSAGE(std::isfinite(static_cast<double>(value)), "Voxel terminal value must be finite.");
    if (hasBackgroundDistance())
    {
        MYVOXEL_REQUIRE_MESSAGE(isValidTsdfDistance(value, m_backgroundDistance), "Voxel terminal value must be inside the configured TSDF range.");
    }

    BlockPool& pool = *m_tree->m_blockPool;
    const VoxelState newState = terminalState(value);
    const VoxelNodeState newNodeState = terminalNodeState(value);

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState != VoxelState::Subdivided && m_tree->m_rootState == newState && nodeValue(m_tree->m_rootBlock) == value)
        {
            return false;
        }
        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            releaseDescendants(m_tree->m_rootBlock, pool);
        }
        m_tree->m_rootState = newState;
        resetRootTerminalBlock(m_tree->m_rootBlock, value);
        return true;

    case Source::NodeChild:
    {
        const VoxelNodeState currentState = nodeChildState();
        const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);

        if ((currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material) && currentState == newNodeState && pool.value(index) == value)
        {
            return false;
        }

        if (currentState == VoxelNodeState::Node)
        {
            releaseDescendants(pool.node(index), pool);
        }
        else if (currentState == VoxelNodeState::MaskLeaf)
        {
            pool.releaseLeaf(index);
        }

        pool.initializeValue(index, value);
        setNodeState(*m_parentBlock, m_childCorner, newNodeState);
        return true;
    }

    case Source::MaskLeafGroup:
        return setLeafGroupChildren(FullVoxelChildMask, value);

    case Source::MaskLeafVoxel:
        return setDistance(value);
    }

    return false;
}

bool VoxelTreeEditor::subdivide()
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);
    BlockPool& pool = *m_tree->m_blockPool;

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState == VoxelState::Subdivided)
        {
            return false;
        }
        {
            const float inheritedValue = nodeValue(m_tree->m_rootBlock);
            const VoxelState inheritedState = m_tree->m_rootState;
            const VoxelIndex firstChildIndex = pool.allocateGroup(inheritedValue);
            MyVoxel::reset(m_tree->m_rootBlock, inheritedState == VoxelState::Material ? VoxelNodeState::Material : VoxelNodeState::Empty);
            setChildStorage(m_tree->m_rootBlock, firstChildIndex);
            m_tree->m_rootState = VoxelState::Subdivided;
            return true;
        }

    case Source::NodeChild:
    {
        const VoxelNodeState currentState = nodeChildState();
        if (currentState == VoxelNodeState::Node || currentState == VoxelNodeState::MaskLeaf)
        {
            return false;
        }

        const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
        const float inheritedValue = pool.value(index);
        const VoxelIndex firstChildIndex = pool.allocateGroup(inheritedValue);
        NodeBlock& childBlock = pool.initializeNode(index, currentState);
        setChildStorage(childBlock, firstChildIndex);
        setNodeState(*m_parentBlock, m_childCorner, VoxelNodeState::Node);
        return true;
    }

    case Source::MaskLeafGroup:
    case Source::MaskLeafVoxel:
        return false;
    }

    return false;
}

bool VoxelTreeEditor::setChildrenState(std::uint8_t childMask, VoxelState newState)
{
    MYVOXEL_ASSERT(newState == VoxelState::Empty || newState == VoxelState::Material);

    if (childMask == EmptyVoxelChildMask)
    {
        return false;
    }

    requireBackgroundDistance();
    const float value = terminalDistance(newState);

    switch (m_source)
    {
    case Source::Root:
        if (m_tree->m_rootState != VoxelState::Subdivided && nodeValue(m_tree->m_rootBlock) == value)
        {
            return false;
        }
        subdivide();
        return setNodeChildren(m_tree->m_rootBlock, childMask, value);

    case Source::NodeChild:
    {
        VoxelNodeState currentState = nodeChildState();
        if (currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material)
        {
            const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
            if (m_tree->m_blockPool->value(index) == value)
            {
                return false;
            }
            subdivide();
            currentState = nodeChildState();
        }
        if (currentState == VoxelNodeState::Node)
        {
            return setNodeChildren(currentNodeBlock(), childMask, value);
        }
        MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);
        return setLeafGroups(childMask, value);
    }

    case Source::MaskLeafGroup:
        return setLeafGroupChildren(childMask, value);

    case Source::MaskLeafVoxel:
        break;
    }

    return false;
}

VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner)
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    switch (m_source)
    {
    case Source::Root:
        subdivide();
        return VoxelTreeEditor(*m_tree, m_tree->m_rootBlock, corner, m_backgroundDistance);

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
            return VoxelTreeEditor(*m_tree, m_tree->m_blockPool->node(index), corner, m_backgroundDistance);
        }

        MYVOXEL_ASSERT(currentState == VoxelNodeState::MaskLeaf);
        LeafData leaf = m_tree->m_blockPool->leaf(index);
        return VoxelTreeEditor(*m_tree, *leaf.mask, *leaf.block, corner, m_backgroundDistance);
    }

    case Source::MaskLeafGroup:
        MYVOXEL_ASSERT(m_maskBlock && m_leafBlock);
        return VoxelTreeEditor(*m_tree, *m_maskBlock, *m_leafBlock, m_coarseCorner, corner, m_backgroundDistance);

    case Source::MaskLeafVoxel:
        MYVOXEL_ASSERT_MESSAGE(false, "A MaskLeaf distance sample has no children.");
        return *this;
    }

    return *this;
}

/// MaskLeaf结构

bool VoxelTreeEditor::canTouchLeaf() const
{
    if (m_source != Source::NodeChild)
    {
        return false;
    }

    const VoxelNodeState currentState = nodeChildState();
    return currentState == VoxelNodeState::Empty ||
           currentState == VoxelNodeState::Material ||
           currentState == VoxelNodeState::MaskLeaf;
}

bool VoxelTreeEditor::touchLeaf()
{
    MYVOXEL_ASSERT(canTouchLeaf());

    const VoxelNodeState currentState = nodeChildState();
    if (currentState == VoxelNodeState::MaskLeaf)
    {
        return false;
    }

    MYVOXEL_ASSERT(currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material);

    const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);
    const float inheritedValue = m_tree->m_blockPool->value(index);
    m_tree->m_blockPool->initializeLeaf(index, inheritedValue);
    setNodeState(*m_parentBlock, m_childCorner, VoxelNodeState::MaskLeaf);
    return true;
}

bool VoxelTreeEditor::setLeafBlock(const LeafBlock& block)
{
    MYVOXEL_ASSERT(canSetLeafBlock());

    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        MYVOXEL_REQUIRE_MESSAGE(std::isfinite(static_cast<double>(block.distances[sampleIndex])), "LeafBlock distance must be finite.");
        if (hasBackgroundDistance())
        {
            MYVOXEL_REQUIRE_MESSAGE(isValidTsdfDistance(block.distances[sampleIndex], m_backgroundDistance), "LeafBlock distance must be inside the configured TSDF range.");
        }
    }

    float uniformValue = 0.0f;
    if (leafUniformValue(block, uniformValue))
    {
        return setValue(uniformValue);
    }

    const VoxelNodeState currentState = nodeChildState();
    BlockPool& pool = *m_tree->m_blockPool;
    const VoxelIndex index = childStorageIndex(*m_parentBlock, m_childCorner);

    if (currentState == VoxelNodeState::MaskLeaf)
    {
        LeafData leaf = currentLeafData();
        MaskBlock rebuiltMask;
        rebuildMask(block, rebuiltMask);

        if (leaf.mask->materialMask == rebuiltMask.materialMask && leafBlocksEqual(*leaf.block, block))
        {
            return false;
        }

        *leaf.block = block;
        *leaf.mask = rebuiltMask;
        return true;
    }

    if (currentState == VoxelNodeState::Node)
    {
        releaseDescendants(pool.node(index), pool);
    }

    LeafData leaf = pool.initializeLeaf(index, block.distances[0]);
    *leaf.block = block;
    rebuildMask(*leaf.block, *leaf.mask);
    setNodeState(*m_parentBlock, m_childCorner, VoxelNodeState::MaskLeaf);
    return true;
}

bool VoxelTreeEditor::setMaterialMask(std::uint64_t materialMask)
{
    MYVOXEL_ASSERT(canSetLeafBlock());
    requireBackgroundDistance();

    LeafBlock block;
    const float emptyDistance = m_backgroundDistance;
    const float materialDistance = -m_backgroundDistance;

    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        block.distances[sampleIndex] = maskBit(materialMask, sampleIndex) ? materialDistance : emptyDistance;
    }

    return setLeafBlock(block);
}

/// 单距离修改

float VoxelTreeEditor::distance() const
{
    MYVOXEL_ASSERT(canSetDistance() && m_leafBlock);
    return leafDistance(*m_leafBlock, m_coarseCorner, m_childCorner);
}

bool VoxelTreeEditor::setDistance(float newDistance)
{
    MYVOXEL_ASSERT(canSetDistance() && m_maskBlock && m_leafBlock);
    MYVOXEL_REQUIRE_MESSAGE(std::isfinite(static_cast<double>(newDistance)), "Voxel distance must be finite.");

    if (hasBackgroundDistance())
    {
        MYVOXEL_REQUIRE_MESSAGE(isValidTsdfDistance(newDistance, m_backgroundDistance), "Voxel distance must be inside the configured TSDF range.");
    }

    const unsigned int sampleIndex = leafSampleIndex(m_coarseCorner, m_childCorner);
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
    MYVOXEL_ASSERT(m_source == Source::NodeChild && m_parentBlock);
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

    MYVOXEL_ASSERT(m_source == Source::NodeChild && nodeChildState() == VoxelNodeState::Node);
    return m_tree->m_blockPool->node(childStorageIndex(*m_parentBlock, m_childCorner));
}

const NodeBlock& VoxelTreeEditor::currentNodeBlock() const
{
    MYVOXEL_ASSERT(m_tree && m_tree->m_blockPool);

    if (m_source == Source::Root)
    {
        MYVOXEL_ASSERT(m_tree->m_rootState == VoxelState::Subdivided);
        return m_tree->m_rootBlock;
    }

    MYVOXEL_ASSERT(m_source == Source::NodeChild && nodeChildState() == VoxelNodeState::Node);
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

    MYVOXEL_ASSERT(m_source == Source::NodeChild && nodeChildState() == VoxelNodeState::MaskLeaf);
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

    MYVOXEL_ASSERT(m_source == Source::NodeChild && nodeChildState() == VoxelNodeState::MaskLeaf);
    const BlockPool& pool = *m_tree->m_blockPool;
    return pool.leaf(childStorageIndex(*m_parentBlock, m_childCorner));
}

float VoxelTreeEditor::terminalDistance(VoxelState stateValue) const
{
    requireBackgroundDistance();
    MYVOXEL_ASSERT(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);
    return stateValue == VoxelState::Material ? -m_backgroundDistance : m_backgroundDistance;
}

void VoxelTreeEditor::requireBackgroundDistance() const
{
    MYVOXEL_REQUIRE_MESSAGE(isValidBackgroundDistance(m_backgroundDistance), "This VoxelTreeEditor operation requires a finite positive background distance.");
}

bool VoxelTreeEditor::setNodeChildren(NodeBlock& block, std::uint8_t childMask, float value)
{
    if (childMask == EmptyVoxelChildMask)
    {
        return false;
    }

    BlockPool& pool = *m_tree->m_blockPool;
    const VoxelNodeState targetState = terminalNodeState(value);
    std::uint8_t changedMask = 0;

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const std::uint8_t bit = cornerBit(corner);
        if ((childMask & bit) == 0)
        {
            continue;
        }

        const VoxelNodeState currentState = nodeState(block, corner);
        const VoxelIndex index = childStorageIndex(block, corner);

        if ((currentState == VoxelNodeState::Empty || currentState == VoxelNodeState::Material) &&
            currentState == targetState &&
            pool.value(index) == value)
        {
            continue;
        }

        if (currentState == VoxelNodeState::Node)
        {
            releaseDescendants(pool.node(index), pool);
        }
        else if (currentState == VoxelNodeState::MaskLeaf)
        {
            pool.releaseLeaf(index);
        }

        pool.initializeValue(index, value);
        changedMask = static_cast<std::uint8_t>(changedMask | bit);
    }

    if (changedMask == 0)
    {
        return false;
    }

    setNodeStateBits(block, changedMask, targetState);
    return true;
}

bool VoxelTreeEditor::setLeafGroups(std::uint8_t groupMask, float value)
{
    LeafData leaf = currentLeafData();
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
            if (leaf.block->distances[sampleIndex] == value)
            {
                continue;
            }

            setLeafDistance(*leaf.mask, *leaf.block, sampleIndex, value);
            changed = true;
        }
    }

    return changed;
}

bool VoxelTreeEditor::setLeafGroupChildren(std::uint8_t childMask, float value)
{
    MYVOXEL_ASSERT(m_maskBlock && m_leafBlock);

    const unsigned int coarseIndex = static_cast<unsigned int>(m_coarseCorner);
    bool changed = false;

    for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(VoxelCornerCount); ++fineIndex)
    {
        if ((childMask & static_cast<std::uint8_t>(1U << fineIndex)) == 0)
        {
            continue;
        }

        const unsigned int sampleIndex = coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;
        if (m_leafBlock->distances[sampleIndex] == value)
        {
            continue;
        }

        setLeafDistance(*m_maskBlock, *m_leafBlock, sampleIndex, value);
        changed = true;
    }

    return changed;
}

}