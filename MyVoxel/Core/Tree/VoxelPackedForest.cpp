#include "VoxelPackedForest.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <utility>

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

VoxelState VoxelPackedForest::state(const VoxelCellAddress& address) const
{
    const VoxelPackedRootTree* tree = findRootTree(rootCellAddress(address).index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    VoxelPackedTreeConstCursor cursor(*tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        const VoxelState currentState = cursor.state();

        if (currentState != VoxelState::Subdivided)
        {
            return currentState;
        }

        cursor = cursor.child(path[pathIndex]);
    }

    return cursor.state();
}

bool VoxelPackedForest::hasNode(const VoxelCellAddress& address) const
{
    const VoxelPackedRootTree* tree = findRootTree(rootCellAddress(address).index);
    return tree && hasNode(*tree, address);
}

bool VoxelPackedForest::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    const bool changed = setStateDeferred(address, stateValue);

    if (changed && stateValue == VoxelState::Empty && address.level != BaseVoxelLevel)
    {
        removeEmptyBranch(address);
    }

    return changed;
}

bool VoxelPackedForest::setStateDeferred(const VoxelCellAddress& address, VoxelState stateValue)
{
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    const VoxelCellAddress rootAddress = rootCellAddress(address);

    // 删除第0层空根时直接移除共享根树引用，避免无意义深复制。
    if (address.level == BaseVoxelLevel && stateValue == VoxelState::Empty)
    {
        return m_roots.erase(rootAddress.index) > 0;
    }

    if (state(address) == stateValue)
    {
        return false;
    }

    VoxelPackedRootTree& tree = ensureEditableRootTree(rootAddress.index, VoxelState::Empty);
    VoxelPackedTreeEditor editor = ensureEditor(tree, address);

    editor.setState(stateValue);
    return true;
}

bool VoxelPackedForest::split(const VoxelCellAddress& address)
{
    if (state(address) != VoxelState::Material)
    {
        return false;
    }

    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelPackedRootTree& tree = ensureEditableRootTree(rootAddress.index, VoxelState::Empty);
    VoxelPackedTreeEditor editor = ensureEditor(tree, address);

    if (editor.state() != VoxelState::Material)
    {
        return false;
    }

    editor.split();
    return true;
}

bool VoxelPackedForest::merge(const VoxelCellAddress& address)
{
    const VoxelState mergedState = mergeDeferred(address);

    if (mergedState == VoxelState::Empty)
    {
        removeEmptyBranch(address);
    }

    return mergedState != VoxelState::Subdivided;
}

VoxelState VoxelPackedForest::mergeDeferred(const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelPackedRootTree* tree = findEditableRootTree(rootAddress.index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    if (!hasNode(*tree, address))
    {
        return VoxelState::Empty;
    }

    VoxelPackedTreeEditor editor = locateEditor(*tree, address);
    return editor.merge();
}

void VoxelPackedForest::pruneEmptyBranch(const VoxelCellAddress& address)
{
    removeEmptyBranch(address);
}
bool VoxelPackedForest::isEmpty() const
{
    return m_roots.empty();
}
std::size_t VoxelPackedForest::rootCount() const
{
    return m_roots.size();
}

void VoxelPackedForest::clear()
{
    m_roots.clear();
}
const VoxelPackedRootTree* VoxelPackedForest::rootTree(const VoxelCellIndex& rootIndex) const
{
    const RootMap::const_iterator iterator = m_roots.find(rootIndex);
    return iterator == m_roots.end() ? nullptr : iterator->second.get();
}


VoxelPackedRootTree* VoxelPackedForest::detachRootTree(const VoxelCellIndex& rootIndex)
{
    return findEditableRootTree(rootIndex);
}

bool VoxelPackedForest::eraseRootTree(const VoxelCellIndex& rootIndex)
{
    return m_roots.erase(rootIndex) > 0;
}
std::size_t VoxelPackedForest::forEachRootCell(const RootCellVisitor& visitor) const
{
    assert(visitor);

    for (RootMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        assert(iterator->second);
        visitor(VoxelCellAddress(iterator->first, BaseVoxelLevel));
    }

    return m_roots.size();
}
std::size_t VoxelPackedForest::forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const
{
    assert(visitor);

    return visitRootTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&](const VoxelCellIndex& index, const VoxelPackedRootTree&)
        {
            visitor(VoxelCellAddress(index, BaseVoxelLevel));
        });
}

void VoxelPackedForest::forEachMaterialCell(const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    for (RootMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        assert(iterator->second);

        const VoxelPackedRootTree& tree = *iterator->second;
        visitMaterialCells(VoxelCellAddress(iterator->first, BaseVoxelLevel), VoxelPackedTreeConstCursor(tree), visitor);
    }
}

std::size_t VoxelPackedForest::forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    return visitRootTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&](const VoxelCellIndex& index, const VoxelPackedRootTree& tree)
        {
            visitMaterialCells(VoxelCellAddress(index, BaseVoxelLevel), VoxelPackedTreeConstCursor(tree), visitor);
        });
}

VoxelCellAddress VoxelPackedForest::rootCellAddress(const VoxelCellAddress& address)
{
    VoxelCellAddress rootAddress = address;

    while (hasParentCell(rootAddress))
    {
        rootAddress = parentCellAddress(rootAddress);
    }

    return rootAddress;
}

void VoxelPackedForest::buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path)
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

const VoxelPackedRootTree* VoxelPackedForest::findRootTree(const VoxelCellIndex& rootIndex) const
{
    const RootMap::const_iterator iterator = m_roots.find(rootIndex);
    return iterator == m_roots.end() ? nullptr : iterator->second.get();
}

VoxelPackedRootTree* VoxelPackedForest::findEditableRootTree(const VoxelCellIndex& rootIndex)
{
    RootMap::iterator iterator = m_roots.find(rootIndex);

    if (iterator == m_roots.end())
    {
        return nullptr;
    }

    assert(iterator->second);

    if (!iterator->second.unique())
    {
        iterator->second = std::make_shared<VoxelPackedRootTree>(*iterator->second);
    }

    return iterator->second.get();
}

VoxelPackedRootTree& VoxelPackedForest::ensureEditableRootTree(const VoxelCellIndex& rootIndex, VoxelState initialState)
{
    assert(initialState == VoxelState::Empty || initialState == VoxelState::Material);

    std::pair<RootMap::iterator, bool> result =
        m_roots.insert(std::make_pair(rootIndex, std::make_shared<VoxelPackedRootTree>(initialState)));

    assert(result.first->second);

    if (!result.second && !result.first->second.unique())
    {
        result.first->second = std::make_shared<VoxelPackedRootTree>(*result.first->second);
    }

    return *result.first->second;
}

VoxelPackedTreeConstCursor VoxelPackedForest::locateCursor(const VoxelPackedRootTree& tree, const VoxelCellAddress& address)
{
    VoxelPackedTreeConstCursor cursor(tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        assert(cursor.canAccessChildren());
        cursor = cursor.child(path[pathIndex]);
    }

    return cursor;
}

VoxelPackedTreeEditor VoxelPackedForest::locateEditor(VoxelPackedRootTree& tree, const VoxelCellAddress& address)
{
    VoxelPackedTreeEditor editor(tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        assert(editor.canAccessChildren());
        editor = editor.child(path[pathIndex]);
    }

    return editor;
}

VoxelPackedTreeEditor VoxelPackedForest::ensureEditor(VoxelPackedRootTree& tree, const VoxelCellAddress& address)
{
    VoxelPackedTreeEditor editor(tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        if (!editor.canAccessChildren())
        {
            editor.split();
        }

        assert(editor.canAccessChildren());
        editor = editor.child(path[pathIndex]);
    }

    return editor;
}

bool VoxelPackedForest::hasNode(const VoxelPackedRootTree& tree, const VoxelCellAddress& address)
{
    VoxelPackedTreeConstCursor cursor(tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        if (!cursor.canAccessChildren())
        {
            return false;
        }

        cursor = cursor.child(path[pathIndex]);
    }

    return true;
}

void VoxelPackedForest::removeEmptyBranch(VoxelCellAddress address)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelPackedRootTree* tree = findEditableRootTree(rootAddress.index);

    if (!tree || state(address) != VoxelState::Empty)
    {
        return;
    }

    while (hasParentCell(address))
    {
        const VoxelCellAddress parentAddress = parentCellAddress(address);

        if (!hasNode(*tree, parentAddress))
        {
            break;
        }

        VoxelPackedTreeEditor parentEditor = locateEditor(*tree, parentAddress);

        if (parentEditor.state() != VoxelState::Subdivided)
        {
            break;
        }

        if (parentEditor.merge() != VoxelState::Empty)
        {
            break;
        }

        address = parentAddress;
    }

    if (tree->rootState == VoxelState::Empty)
    {
        m_roots.erase(rootAddress.index);
    }
}

std::size_t VoxelPackedForest::visitRootTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootTreeVisitor& visitor) const
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

    // XY候选行数较大时直接过滤实际根树，避免查询大量不存在的网格行。
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

void VoxelPackedForest::visitMaterialCells(const VoxelCellAddress& address, const VoxelPackedTreeConstCursor& cursor, const MaterialCellVisitor& visitor)
{
    const VoxelState currentState = cursor.state();

    if (currentState == VoxelState::Empty)
    {
        return;
    }

    if (currentState == VoxelState::Material)
    {
        visitor(address);
        return;
    }

    assert(currentState == VoxelState::Subdivided);

    for (int cornerIndex = 0; cornerIndex < VoxelCornerCount; ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        visitMaterialCells(childCellAddress(address, corner), cursor.child(corner), visitor);
    }
}

}