#include "VoxelNodeForest.h"

#include <cassert>
#include <cstdint>
#include <limits>
#include <utility>
#include <algorithm>
namespace
{

// 返回包含两个端点的体素索引范围长度。
std::uint64_t inclusiveIndexRangeLength(MyVoxel::VoxelIndex minimumValue, MyVoxel::VoxelIndex maximumValue)
{
    assert(minimumValue <= maximumValue);
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(maximumValue) - static_cast<std::int64_t>(minimumValue)) + 1;
}

// 执行无符号64位饱和乘法。
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

VoxelState VoxelNodeForest::state(const VoxelCellAddress& address) const
{
    const VoxelRootTree* tree = findRootTree(rootCellAddress(address).index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    const VoxelNode* node = &tree->root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (node->state() != VoxelState::Subdivided)
        {
            return node->state();
        }

        const VoxelNodeGroup& group = tree->nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[i]);
    }

    return node->state();
}

bool VoxelNodeForest::hasNode(const VoxelCellAddress& address) const
{
    return findNode(address) != nullptr;
}

bool VoxelNodeForest::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    const bool changed = setStateDeferred(address, stateValue);

    if (changed && stateValue == VoxelState::Empty && address.level != BaseVoxelLevel)
    {
        removeEmptyBranch(address);
    }

    return changed;
}

bool VoxelNodeForest::setStateDeferred(const VoxelCellAddress& address, VoxelState stateValue)
{
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    const VoxelCellAddress rootAddress = rootCellAddress(address);

    // 删除第0层根节点时直接移除共享根树引用，不复制完整根树。
    if (address.level == BaseVoxelLevel && stateValue == VoxelState::Empty)
    {
        return m_roots.erase(rootAddress.index) > 0;
    }

    if (state(address) == stateValue)
    {
        return false;
    }

    VoxelRootTree& tree = ensureEditableRootTree(rootAddress.index, VoxelState::Empty);
    VoxelNode& node = ensureNode(tree, address);

    if (node.state() == VoxelState::Subdivided)
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

bool VoxelNodeForest::split(const VoxelCellAddress& address)
{
    if (state(address) != VoxelState::Material)
    {
        return false;
    }

    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelRootTree& tree = ensureEditableRootTree(rootAddress.index, VoxelState::Empty);
    VoxelNode& node = ensureNode(tree, address);

    if (node.state() != VoxelState::Material)
    {
        return false;
    }

    const VoxelNodeGroupIndex groupIndex = tree.nodePool.allocateNodeGroup(VoxelState::Material);
    node.setChildGroupIndex(groupIndex);
    return true;
}

bool VoxelNodeForest::merge(const VoxelCellAddress& address)
{
    const VoxelState mergedState = mergeDeferred(address);

    if (mergedState == VoxelState::Empty)
    {
        removeEmptyBranch(address);
    }

    return mergedState != VoxelState::Subdivided;
}

VoxelState VoxelNodeForest::mergeDeferred(const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelRootTree* tree = findEditableRootTree(rootAddress.index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    VoxelNode* node = findNode(*tree, address);

    if (!node)
    {
        return VoxelState::Empty;
    }

    if (node->state() != VoxelState::Subdivided)
    {
        return node->state();
    }

    const VoxelNodeGroupIndex groupIndex = node->childGroupIndex();
    const VoxelNodeGroup& group = tree->nodePool.nodeGroup(groupIndex);

    if (!group.canMerge())
    {
        return VoxelState::Subdivided;
    }

    const VoxelState mergedState = group.mergedState();

    node->setState(mergedState);
    tree->nodePool.releaseNodeGroup(groupIndex);
    return mergedState;
}

void VoxelNodeForest::pruneEmptyBranch(const VoxelCellAddress& address)
{
    removeEmptyBranch(address);
}

std::size_t VoxelNodeForest::rootCount() const
{
    return m_roots.size();
}

void VoxelNodeForest::clear()
{
    m_roots.clear();
}
VoxelRootTree* VoxelNodeForest::detachRootTree(const VoxelCellIndex& rootIndex)
{
    return findEditableRootTree(rootIndex);
}

bool VoxelNodeForest::eraseRootTree(const VoxelCellIndex& rootIndex)
{
    return m_roots.erase(rootIndex) > 0;
}
std::size_t VoxelNodeForest::forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const
{
    assert(visitor);

    return visitRootTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&](const VoxelCellIndex& index, const VoxelRootTree&)
        {
            visitor(VoxelCellAddress(index, BaseVoxelLevel));
        });
}

void VoxelNodeForest::forEachMaterialCell(const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    for (RootMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        assert(iterator->second);

        const VoxelRootTree& tree = *iterator->second;
        visitMaterialCells(VoxelCellAddress(iterator->first, BaseVoxelLevel), tree.root, tree.nodePool, visitor);
    }
}

std::size_t VoxelNodeForest::forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    return visitRootTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&](const VoxelCellIndex& index, const VoxelRootTree& tree)
        {
            visitMaterialCells(VoxelCellAddress(index, BaseVoxelLevel), tree.root, tree.nodePool, visitor);
        });
}

VoxelCellAddress VoxelNodeForest::rootCellAddress(const VoxelCellAddress& address)
{
    VoxelCellAddress rootAddress = address;

    while (hasParentCell(rootAddress))
    {
        rootAddress = parentCellAddress(rootAddress);
    }

    return rootAddress;
}

void VoxelNodeForest::buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path)
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

const VoxelRootTree* VoxelNodeForest::findRootTree(const VoxelCellIndex& rootIndex) const
{
    const RootMap::const_iterator iterator = m_roots.find(rootIndex);
    return iterator == m_roots.end() ? nullptr : iterator->second.get();
}

VoxelRootTree* VoxelNodeForest::findEditableRootTree(const VoxelCellIndex& rootIndex)
{
    RootMap::iterator iterator = m_roots.find(rootIndex);

    if (iterator == m_roots.end())
    {
        return nullptr;
    }

    assert(iterator->second);

    if (!iterator->second.unique())
    {
        iterator->second = std::make_shared<VoxelRootTree>(*iterator->second);
    }

    return iterator->second.get();
}

VoxelRootTree& VoxelNodeForest::ensureEditableRootTree(const VoxelCellIndex& rootIndex, VoxelState initialState)
{
    assert(initialState == VoxelState::Empty || initialState == VoxelState::Material);

    std::pair<RootMap::iterator, bool> result =
        m_roots.insert(std::make_pair(rootIndex, std::make_shared<VoxelRootTree>(initialState)));

    assert(result.first->second);

    if (!result.second && !result.first->second.unique())
    {
        result.first->second = std::make_shared<VoxelRootTree>(*result.first->second);
    }

    return *result.first->second;
}

VoxelNode* VoxelNodeForest::findNode(VoxelRootTree& tree, const VoxelCellAddress& address)
{
    VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (node->state() != VoxelState::Subdivided)
        {
            return nullptr;
        }

        VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[i]);
    }

    return node;
}

const VoxelNode* VoxelNodeForest::findNode(const VoxelRootTree& tree, const VoxelCellAddress& address)
{
    const VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (node->state() != VoxelState::Subdivided)
        {
            return nullptr;
        }

        const VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[i]);
    }

    return node;
}

VoxelNode* VoxelNodeForest::findNode(const VoxelCellAddress& address)
{
    VoxelRootTree* tree = findEditableRootTree(rootCellAddress(address).index);
    return tree ? findNode(*tree, address) : nullptr;
}

const VoxelNode* VoxelNodeForest::findNode(const VoxelCellAddress& address) const
{
    const VoxelRootTree* tree = findRootTree(rootCellAddress(address).index);
    return tree ? findNode(*tree, address) : nullptr;
}

VoxelNode& VoxelNodeForest::ensureNode(VoxelRootTree& tree, const VoxelCellAddress& address)
{
    VoxelNode* node = &tree.root;
    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (node->state() != VoxelState::Subdivided)
        {
            const VoxelState currentState = node->state();
            const VoxelNodeGroupIndex groupIndex = tree.nodePool.allocateNodeGroup(currentState);
            node->setChildGroupIndex(groupIndex);
        }

        VoxelNodeGroup& group = tree.nodePool.nodeGroup(node->childGroupIndex());
        node = &group.child(path[i]);
    }

    return *node;
}

void VoxelNodeForest::removeEmptyBranch(VoxelCellAddress address)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelRootTree* tree = findEditableRootTree(rootAddress.index);

    if (!tree)
    {
        return;
    }

    VoxelNode* node = findNode(*tree, address);

    if (!node || node->state() != VoxelState::Empty)
    {
        return;
    }

    while (hasParentCell(address))
    {
        const VoxelCellAddress parentAddress = parentCellAddress(address);
        VoxelNode* parent = findNode(*tree, parentAddress);

        if (!parent || parent->state() != VoxelState::Subdivided)
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

    if (address.level == BaseVoxelLevel && tree->root.state() == VoxelState::Empty)
    {
        m_roots.erase(rootAddress.index);
    }
}

std::size_t VoxelNodeForest::visitRootTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootTreeVisitor& visitor) const
{
    assert(visitor);
    assert(minimumRootIndex.x <= maximumRootIndex.x);
    assert(minimumRootIndex.y <= maximumRootIndex.y);
    assert(minimumRootIndex.z <= maximumRootIndex.z);

    if (m_roots.empty())
    {
        return 0;
    }

    const std::uint64_t xRangeCount = inclusiveIndexRangeLength(minimumRootIndex.x, maximumRootIndex.x);
    const std::uint64_t yRangeCount = inclusiveIndexRangeLength(minimumRootIndex.y, maximumRootIndex.y);
    const std::uint64_t xyRangeCount = saturatedMultiply(xRangeCount, yRangeCount);

    std::size_t existingRootCount = 0;

    // XY候选行数较小时，按每个XY网格行使用lower_bound扫描对应Z区间。
    if (xyRangeCount <= static_cast<std::uint64_t>(m_roots.size()))
    {
        for (std::int64_t x = static_cast<std::int64_t>(minimumRootIndex.x); x <= static_cast<std::int64_t>(maximumRootIndex.x); ++x)
        {
            const VoxelIndex xIndex = static_cast<VoxelIndex>(x);

            for (std::int64_t y = static_cast<std::int64_t>(minimumRootIndex.y); y <= static_cast<std::int64_t>(maximumRootIndex.y); ++y)
            {
                const VoxelIndex yIndex = static_cast<VoxelIndex>(y);
                RootMap::const_iterator iterator = m_roots.lower_bound(VoxelCellIndex(xIndex, yIndex, minimumRootIndex.z));

                while (iterator != m_roots.end() &&
                       iterator->first.x == xIndex &&
                       iterator->first.y == yIndex &&
                       iterator->first.z <= maximumRootIndex.z)
                {
                    assert(iterator->second);

                    visitor(iterator->first, *iterator->second);
                    ++existingRootCount;
                    ++iterator;
                }
            }
        }

        return existingRootCount;
    }

    // XY候选行数较大时直接过滤现有根节点，避免查询大量不存在的网格行。
    for (RootMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        const VoxelCellIndex& index = iterator->first;

        if (index.x < minimumRootIndex.x || index.x > maximumRootIndex.x ||
            index.y < minimumRootIndex.y || index.y > maximumRootIndex.y ||
            index.z < minimumRootIndex.z || index.z > maximumRootIndex.z)
        {
            continue;
        }

        assert(iterator->second);

        visitor(index, *iterator->second);
        ++existingRootCount;
    }

    return existingRootCount;
}

void VoxelNodeForest::visitMaterialCells(const VoxelCellAddress& address, const VoxelNode& node, const VoxelNodePool& nodePool, const MaterialCellVisitor& visitor)
{
    if (node.state() == VoxelState::Empty)
    {
        return;
    }

    if (node.state() == VoxelState::Material)
    {
        visitor(address);
        return;
    }

    assert(node.state() == VoxelState::Subdivided);

    const VoxelNodeGroup& group = nodePool.nodeGroup(node.childGroupIndex());

    for (int i = 0; i < VoxelCornerCount; ++i)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(i);
        visitMaterialCells(childCellAddress(address, corner), group.child(corner), nodePool, visitor);
    }
}

}