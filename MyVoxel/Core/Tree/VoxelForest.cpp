#include "VoxelForest.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 返回包含两个端点的体素索引范围长度。
std::uint64_t inclusiveIndexRangeLength(MyVoxel::VoxelIndex minimumValue, MyVoxel::VoxelIndex maximumValue)
{
    MYVOXEL_ASSERT_MESSAGE(minimumValue <= maximumValue, "Voxel index range minimum must not exceed maximum.");

    const std::int64_t difference = static_cast<std::int64_t>(maximumValue) - static_cast<std::int64_t>(minimumValue);
    return static_cast<std::uint64_t>(difference) + 1;
}

// 执行无符号64位饱和乘法，溢出时返回最大值。
std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    const std::uint64_t maximumValue = (std::numeric_limits<std::uint64_t>::max)();

    if (first > maximumValue / second)
    {
        return maximumValue;
    }

    return first * second;
}

}

namespace MyVoxel
{

/// 节点状态

VoxelState VoxelForest::state(const VoxelCellAddress& address) const
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* tree = findTree(rootAddress.index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    const VoxelNode* node = &tree->root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelForest contains an invalid node.");

        if (!node->isSubdivided())
        {
            return node->state();
        }

        const VoxelNodeGroup& group = tree->nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[pathIndex]);
    }

    MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelForest contains an invalid target node.");
    return node->state();
}

bool VoxelForest::hasNode(const VoxelCellAddress& address) const
{
    return findNode(address) != nullptr;
}

bool VoxelForest::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    const bool changed = setStateDeferred(address, stateValue);

    if (changed && stateValue == VoxelState::Empty && address.level != BaseVoxelLevel)
    {
        removeEmptyBranch(address);
    }

    return changed;
}

bool VoxelForest::setStateDeferred(const VoxelCellAddress& address, VoxelState stateValue)
{
    MYVOXEL_ASSERT_MESSAGE(stateValue == VoxelState::Empty || stateValue == VoxelState::Material, "VoxelForest state must be Empty or Material.");

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);

    if (address.level == BaseVoxelLevel && stateValue == VoxelState::Empty)
    {
        return m_trees.erase(rootAddress.index) > 0;
    }

    if (state(address) == stateValue)
    {
        return false;
    }

    VoxelTree& tree = ensureEditableTree(rootAddress.index, VoxelState::Empty);
    VoxelNode& node = ensureNode(tree, address);

    if (node.isSubdivided())
    {
        const VoxelNodeGroupIndex groupIndex = node.childGroupIndex();

        node.setState(stateValue);
        tree.nodePool.releaseNodeGroup(groupIndex);
    }
    else
    {
        node.setState(stateValue);
    }

    return true;
}

/// 节点结构

bool VoxelForest::split(const VoxelCellAddress& address)
{
    if (state(address) != VoxelState::Material)
    {
        return false;
    }

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    VoxelTree& tree = ensureEditableTree(rootAddress.index, VoxelState::Empty);
    VoxelNode& node = ensureNode(tree, address);

    MYVOXEL_ASSERT_MESSAGE(node.isMaterial(), "VoxelForest split target must resolve to a Material node.");

    const VoxelNodeGroupIndex groupIndex = tree.nodePool.allocateNodeGroup(VoxelState::Material);
    node.setChildGroupIndex(groupIndex);
    return true;
}

bool VoxelForest::merge(const VoxelCellAddress& address)
{
    const VoxelState mergedState = mergeDeferred(address);

    if (mergedState == VoxelState::Empty)
    {
        removeEmptyBranch(address);
    }

    return mergedState != VoxelState::Subdivided;
}

VoxelState VoxelForest::mergeDeferred(const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* readOnlyTree = findTree(rootAddress.index);

    if (!readOnlyTree)
    {
        return VoxelState::Empty;
    }

    const VoxelNode* readOnlyNode = findNode(*readOnlyTree, address);

    if (!readOnlyNode)
    {
        return state(address);
    }

    if (!readOnlyNode->isSubdivided())
    {
        return readOnlyNode->state();
    }

    const VoxelNodeGroup& readOnlyGroup = readOnlyTree->nodePool.nodeGroup(readOnlyNode->childGroupIndex());

    if (!readOnlyGroup.canMerge())
    {
        return VoxelState::Subdivided;
    }

    const VoxelState mergedState = readOnlyGroup.mergedState();
    VoxelTree* editableTree = findEditableTree(rootAddress.index);

    MYVOXEL_ASSERT_MESSAGE(editableTree, "An existing VoxelTree must remain available during merge.");

    VoxelNode* editableNode = findNode(*editableTree, address);

    MYVOXEL_ASSERT_MESSAGE(editableNode && editableNode->isSubdivided(), "VoxelForest editable merge node must remain subdivided.");

    const VoxelNodeGroupIndex groupIndex = editableNode->childGroupIndex();

    editableNode->setState(mergedState);
    editableTree->nodePool.releaseNodeGroup(groupIndex);
    return mergedState;
}

void VoxelForest::pruneEmptyBranch(const VoxelCellAddress& address)
{
    removeEmptyBranch(address);
}

/// 森林管理

std::size_t VoxelForest::rootCount() const
{
    return m_trees.size();
}

bool VoxelForest::isEmpty() const
{
    return m_trees.empty();
}

void VoxelForest::clear()
{
    m_trees.clear();
}

/// 节点遍历

std::size_t VoxelForest::forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const
{
    MYVOXEL_ASSERT_MESSAGE(visitor, "VoxelForest root cell visitor must be valid.");

    return visitTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&visitor](const VoxelCellIndex& rootIndex, const VoxelTree&)
        {
            visitor(VoxelCellAddress(rootIndex, BaseVoxelLevel));
        });
}

void VoxelForest::forEachMaterialCell(const MaterialCellVisitor& visitor) const
{
    MYVOXEL_ASSERT_MESSAGE(visitor, "VoxelForest material cell visitor must be valid.");

    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        MYVOXEL_ASSERT_MESSAGE(iterator->second, "VoxelForest tree reference must not be null.");

        const VoxelTree& tree = *iterator->second;
        visitMaterialCells(VoxelCellAddress(iterator->first, BaseVoxelLevel), tree.root, tree.nodePool, visitor);
    }
}

std::size_t VoxelForest::forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const
{
    MYVOXEL_ASSERT_MESSAGE(visitor, "VoxelForest material cell visitor must be valid.");

    return visitTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&visitor](const VoxelCellIndex& rootIndex, const VoxelTree& tree)
        {
            visitMaterialCells(VoxelCellAddress(rootIndex, BaseVoxelLevel), tree.root, tree.nodePool, visitor);
        });
}

/// 内部路径与树管理

void VoxelForest::buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path)
{
    path.clear();
    path.reserve(static_cast<std::size_t>(address.level));

    VoxelCellAddress currentAddress = address;

    while (hasParentCell(currentAddress))
    {
        path.push_back(childCornerInParent(currentAddress));
        currentAddress = parentCellAddress(currentAddress);
    }

    std::reverse(path.begin(), path.end());
}

const VoxelTree* VoxelForest::findTree(const VoxelCellIndex& rootIndex) const
{
    const TreeMap::const_iterator iterator = m_trees.find(rootIndex);
    return iterator == m_trees.end() ? nullptr : iterator->second.get();
}

VoxelTree* VoxelForest::findEditableTree(const VoxelCellIndex& rootIndex)
{
    TreeMap::iterator iterator = m_trees.find(rootIndex);

    if (iterator == m_trees.end())
    {
        return nullptr;
    }

    MYVOXEL_ASSERT_MESSAGE(iterator->second, "VoxelForest tree reference must not be null.");

    if (iterator->second->referenceCount() > 1)
    {
        iterator->second = Foundation::makeRef<VoxelTree>(*iterator->second);
    }

    return iterator->second.get();
}

VoxelTree& VoxelForest::ensureEditableTree(const VoxelCellIndex& rootIndex, VoxelState initialState)
{
    MYVOXEL_ASSERT_MESSAGE(initialState == VoxelState::Empty || initialState == VoxelState::Material, "VoxelTree initial state must be Empty or Material.");

    TreeMap::iterator iterator = m_trees.find(rootIndex);

    if (iterator == m_trees.end())
    {
        const TreePtr tree = Foundation::makeRef<VoxelTree>(initialState);
        iterator = m_trees.insert(std::make_pair(rootIndex, tree)).first;
    }
    else if (iterator->second->referenceCount() > 1)
    {
        iterator->second = Foundation::makeRef<VoxelTree>(*iterator->second);
    }

    MYVOXEL_ASSERT_MESSAGE(iterator->second, "VoxelForest editable tree must not be null.");
    return *iterator->second;
}

/// 内部节点访问

VoxelNode* VoxelForest::findNode(VoxelTree& tree, const VoxelCellAddress& address)
{
    VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelForest contains an invalid node.");

        if (!node->isSubdivided())
        {
            return nullptr;
        }

        VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[pathIndex]);
    }

    return node;
}

const VoxelNode* VoxelForest::findNode(const VoxelTree& tree, const VoxelCellAddress& address)
{
    const VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelForest contains an invalid node.");

        if (!node->isSubdivided())
        {
            return nullptr;
        }

        const VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[pathIndex]);
    }

    return node;
}

const VoxelNode* VoxelForest::findNode(const VoxelCellAddress& address) const
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* tree = findTree(rootAddress.index);
    return tree ? findNode(*tree, address) : nullptr;
}

VoxelNode& VoxelForest::ensureNode(VoxelTree& tree, const VoxelCellAddress& address)
{
    VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(node->isValid(), "VoxelForest contains an invalid node.");

        if (!node->isSubdivided())
        {
            const VoxelState currentState = node->state();
            const VoxelNodeGroupIndex groupIndex = tree.nodePool.allocateNodeGroup(currentState);
            node->setChildGroupIndex(groupIndex);
        }

        VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[pathIndex]);
    }

    return *node;
}

void VoxelForest::removeEmptyBranch(VoxelCellAddress address)
{
    if (state(address) != VoxelState::Empty)
    {
        return;
    }

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    VoxelTree* tree = findEditableTree(rootAddress.index);

    if (!tree)
    {
        return;
    }

    VoxelNode* node = findNode(*tree, address);

    if (!node || !node->isEmpty())
    {
        return;
    }

    while (hasParentCell(address))
    {
        const VoxelCellAddress parentAddress = parentCellAddress(address);
        VoxelNode* parent = findNode(*tree, parentAddress);

        if (!parent || !parent->isSubdivided())
        {
            break;
        }

        const VoxelNodeGroupIndex groupIndex = parent->childGroupIndex();
        const VoxelNodeGroup& group = tree->nodePool.nodeGroup(groupIndex);

        if (!group.canMerge() || group.mergedState() != VoxelState::Empty)
        {
            break;
        }

        parent->setState(VoxelState::Empty);
        tree->nodePool.releaseNodeGroup(groupIndex);
        address = parentAddress;
    }

    if (address.level == BaseVoxelLevel && tree->root.isEmpty())
    {
        m_trees.erase(rootAddress.index);
    }
}

/// 内部遍历

std::size_t VoxelForest::visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const
{
    MYVOXEL_ASSERT_MESSAGE(visitor, "VoxelForest tree visitor must be valid.");
    MYVOXEL_ASSERT_MESSAGE(minimumRootIndex.x <= maximumRootIndex.x, "VoxelForest X range minimum must not exceed maximum.");
    MYVOXEL_ASSERT_MESSAGE(minimumRootIndex.y <= maximumRootIndex.y, "VoxelForest Y range minimum must not exceed maximum.");
    MYVOXEL_ASSERT_MESSAGE(minimumRootIndex.z <= maximumRootIndex.z, "VoxelForest Z range minimum must not exceed maximum.");

    if (m_trees.empty())
    {
        return 0;
    }

    const std::uint64_t xRangeCount = inclusiveIndexRangeLength(minimumRootIndex.x, maximumRootIndex.x);
    const std::uint64_t yRangeCount = inclusiveIndexRangeLength(minimumRootIndex.y, maximumRootIndex.y);
    const std::uint64_t xyRangeCount = saturatedMultiply(xRangeCount, yRangeCount);

    std::size_t existingTreeCount = 0;

    // XY候选行数较小时，按每个XY网格行使用lower_bound扫描对应Z区间。
    if (xyRangeCount <= static_cast<std::uint64_t>(m_trees.size()))
    {
        for (std::int64_t x = static_cast<std::int64_t>(minimumRootIndex.x); x <= static_cast<std::int64_t>(maximumRootIndex.x); ++x)
        {
            const VoxelIndex xIndex = static_cast<VoxelIndex>(x);

            for (std::int64_t y = static_cast<std::int64_t>(minimumRootIndex.y); y <= static_cast<std::int64_t>(maximumRootIndex.y); ++y)
            {
                const VoxelIndex yIndex = static_cast<VoxelIndex>(y);
                TreeMap::const_iterator iterator = m_trees.lower_bound(VoxelCellIndex(xIndex, yIndex, minimumRootIndex.z));

                while (iterator != m_trees.end() &&
                       iterator->first.x == xIndex &&
                       iterator->first.y == yIndex &&
                       iterator->first.z <= maximumRootIndex.z)
                {
                    MYVOXEL_ASSERT_MESSAGE(iterator->second, "VoxelForest tree reference must not be null.");

                    visitor(iterator->first, *iterator->second);
                    ++existingTreeCount;
                    ++iterator;
                }
            }
        }

        return existingTreeCount;
    }

    // XY候选行数较大时直接过滤现有根树，避免查询大量不存在的网格行。
    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        const VoxelCellIndex& rootIndex = iterator->first;

        if (rootIndex.x < minimumRootIndex.x || rootIndex.x > maximumRootIndex.x ||
            rootIndex.y < minimumRootIndex.y || rootIndex.y > maximumRootIndex.y ||
            rootIndex.z < minimumRootIndex.z || rootIndex.z > maximumRootIndex.z)
        {
            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(iterator->second, "VoxelForest tree reference must not be null.");

        visitor(rootIndex, *iterator->second);
        ++existingTreeCount;
    }

    return existingTreeCount;
}

void VoxelForest::visitMaterialCells(const VoxelCellAddress& address, const VoxelNode& node, const VoxelNodePool& nodePool, const MaterialCellVisitor& visitor)
{
    MYVOXEL_ASSERT_MESSAGE(node.isValid(), "VoxelForest material traversal encountered an invalid node.");

    if (node.isEmpty())
    {
        return;
    }

    if (node.isMaterial())
    {
        visitor(address);
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(node.isSubdivided(), "VoxelForest material traversal requires a valid node state.");

    const VoxelNodeGroup& group = nodePool.nodeGroup(node.childGroupIndex());

    for (std::size_t cornerIndex = 0; cornerIndex < VoxelCornerCount; ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        visitMaterialCells(childCellAddress(address, corner), group.child(corner), nodePool, visitor);
    }
}

}