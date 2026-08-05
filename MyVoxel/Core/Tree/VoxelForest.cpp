#include "VoxelForest.h"

#include <cassert>
#include <cstdint>
#include <limits>
#include <utility>

#include "VoxelChildStateMasks.h"
#include "VoxelTreeCursor.h"
#include "VoxelTreeEditor.h"

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

// 在指定根树中定位目标地址，沿途终止体素由编辑器按需细分。
MyVoxel::VoxelTreeEditor ensureEditorAt(MyVoxel::VoxelTree& tree,
                                        const std::vector<MyVoxel::VoxelCorner>& path)
{
    MyVoxel::VoxelTreeEditor editor = tree.editor();

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        editor = editor.child(path[pathIndex]);
    }

    return editor;
}

// 在指定根树中精确定位目标地址，沿途出现终止体素时返回其继承状态。
MyVoxel::VoxelState stateAt(const MyVoxel::VoxelTree& tree, const std::vector<MyVoxel::VoxelCorner>& path, bool& exact)
{
    MyVoxel::VoxelTreeCursor cursor = tree.cursor();

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        if (cursor.state() != MyVoxel::VoxelState::Subdivided)
        {
            exact = false;
            return cursor.state();
        }

        cursor = cursor.child(path[pathIndex]);
    }

    exact = true;
    return cursor.state();
}

}

namespace MyVoxel
{

VoxelForest::VoxelForest(VoxelForest&& other)
    : m_trees(std::move(other.m_trees))
{
    other.m_trees.clear();
}

VoxelForest& VoxelForest::operator=(VoxelForest&& other)
{
    if (this == &other)
    {
        return *this;
    }

    m_trees = std::move(other.m_trees);
    other.m_trees.clear();
    return *this;
}

/// 节点状态

VoxelState VoxelForest::state(const VoxelCellAddress& address) const
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* tree = findTree(rootAddress.index);

    if (!tree)
    {
        return VoxelState::Empty;
    }

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);

    bool exact = false;
    return stateAt(*tree, path, exact);
}

bool VoxelForest::hasNode(const VoxelCellAddress& address) const
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* tree = findTree(rootAddress.index);

    if (!tree)
    {
        return false;
    }

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);

    bool exact = false;
    stateAt(*tree, path, exact);
    return exact;
}

bool VoxelForest::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);

    // 第0层根体素设置为空时直接删除根树，不触发节点池分离。
    if (address.level == BaseVoxelLevel && stateValue == VoxelState::Empty)
    {
        return m_trees.erase(rootAddress.index) > 0;
    }

    if (state(address) == stateValue)
    {
        return false;
    }

    VoxelTree& tree = ensureTree(rootAddress.index, VoxelState::Empty);

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);

    VoxelTreeEditor editor = ensureEditorAt(tree, path);

    if (stateValue == VoxelState::Material)
    {
        editor.setMaterial();
    }
    else
    {
        editor.setEmpty();
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
    VoxelTree* tree = findTree(rootAddress.index);

    assert(tree);

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);

    VoxelTreeEditor editor = ensureEditorAt(*tree, path);
    return editor.subdivide();
}

bool VoxelForest::merge(const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    VoxelTree* tree = findTree(rootAddress.index);

    if (!tree)
    {
        return false;
    }

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);

    bool exact = false;
    const VoxelState currentState = stateAt(*tree, path, exact);

    if (!exact || currentState != VoxelState::Subdivided)
    {
        return false;
    }

    VoxelTreeCursor cursor = tree->cursor();

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        cursor = cursor.child(path[pathIndex]);
    }

    const VoxelChildStateMasks childStates =
        cursor.childStateMasks();

    if (!childStates.isCollapsible())
    {
        return false;
    }

    const VoxelState mergedState =
        childStates.collapsedState();

    VoxelTreeEditor editor = ensureEditorAt(*tree, path);

    if (mergedState == VoxelState::Material)
    {
        editor.setMaterial();
    }
    else
    {
        editor.setEmpty();
    }

    // Forest不保存终止Empty根树，显式合并根体素为空时删除对应根树。
    if (address.level == BaseVoxelLevel && mergedState == VoxelState::Empty)
    {
        m_trees.erase(rootAddress.index);
    }

    return true;
}

/// 森林管理

bool VoxelForest::isEmpty() const
{
    return m_trees.empty();
}

std::size_t VoxelForest::rootCount() const
{
    return m_trees.size();
}

void VoxelForest::clear()
{
    m_trees.clear();
}

void VoxelForest::moveTreesTo(std::vector<TreeEntry>& rootTrees)
{
    if (m_trees.empty())
    {
        return;
    }

    rootTrees.reserve(rootTrees.size() + m_trees.size());

    for (TreeMap::iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        rootTrees.push_back(TreeEntry(iterator->first, std::move(iterator->second)));
    }

    m_trees.clear();
}

bool VoxelForest::eraseTree(const VoxelCellIndex& rootIndex)
{
    return m_trees.erase(rootIndex) > 0;
}

const VoxelTree* VoxelForest::getTree(const VoxelCellIndex& rootIndex) const
{
    return findTree(rootIndex);
}

VoxelTree* VoxelForest::getTree(const VoxelCellIndex& rootIndex)
{
    return findTree(rootIndex);
}

VoxelTree& VoxelForest::setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree)
{
    assert(tree.isValid());
    assert(tree.state() != VoxelState::Empty);

    TreeMap::iterator iterator = m_trees.lower_bound(rootIndex);

    if (iterator != m_trees.end() && !(rootIndex < iterator->first))
    {
        iterator->second = tree;
        return iterator->second;
    }

    iterator = m_trees.emplace_hint(iterator, rootIndex, tree);
    return iterator->second;
}

VoxelTree& VoxelForest::setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree)
{
    assert(tree.isValid());
    assert(tree.state() != VoxelState::Empty);

    TreeMap::iterator iterator = m_trees.lower_bound(rootIndex);

    if (iterator != m_trees.end() && !(rootIndex < iterator->first))
    {
        iterator->second = std::move(tree);
        return iterator->second;
    }

    iterator = m_trees.emplace_hint(iterator, rootIndex, std::move(tree));
    return iterator->second;
}

/// 节点遍历

std::size_t VoxelForest::forEachRootCell(const RootCellVisitor& visitor) const
{
    assert(visitor);

    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        visitor(VoxelCellAddress(iterator->first, BaseVoxelLevel));
    }

    return m_trees.size();
}

std::size_t VoxelForest::forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const
{
    assert(visitor);

    return visitTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&visitor](const VoxelCellIndex& index, const VoxelTree&)
        {
            visitor(VoxelCellAddress(index, BaseVoxelLevel));
        });
}

void VoxelForest::forEachMaterialCell(const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        visitMaterialCells(VoxelCellAddress(iterator->first, BaseVoxelLevel), iterator->second.cursor(), visitor);
    }
}

std::size_t VoxelForest::forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const
{
    assert(visitor);

    return visitTreesInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&visitor](const VoxelCellIndex& index, const VoxelTree& tree)
        {
            visitMaterialCells(VoxelCellAddress(index, BaseVoxelLevel), tree.cursor(), visitor);
        });
}

/// 内部辅助

const VoxelTree* VoxelForest::findTree(const VoxelCellIndex& rootIndex) const
{
    const TreeMap::const_iterator iterator = m_trees.find(rootIndex);
    return iterator == m_trees.end() ? nullptr : &iterator->second;
}

VoxelTree* VoxelForest::findTree(const VoxelCellIndex& rootIndex)
{
    TreeMap::iterator iterator = m_trees.find(rootIndex);
    return iterator == m_trees.end() ? nullptr : &iterator->second;
}

VoxelTree& VoxelForest::ensureTree(const VoxelCellIndex& rootIndex, VoxelState initialState)
{
    assert(initialState == VoxelState::Empty || initialState == VoxelState::Material);

    std::pair<TreeMap::iterator, bool> result = m_trees.insert(std::make_pair(rootIndex, VoxelTree(initialState)));
    return result.first->second;
}

std::size_t VoxelForest::visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const
{
    assert(visitor);
    assert(minimumRootIndex.x <= maximumRootIndex.x);
    assert(minimumRootIndex.y <= maximumRootIndex.y);
    assert(minimumRootIndex.z <= maximumRootIndex.z);

    if (m_trees.empty())
    {
        return 0;
    }

    const std::uint64_t xRangeCount = inclusiveIndexRangeLength(minimumRootIndex.x, maximumRootIndex.x);
    const std::uint64_t yRangeCount = inclusiveIndexRangeLength(minimumRootIndex.y, maximumRootIndex.y);
    const std::uint64_t xyRangeCount = saturatedMultiply(xRangeCount, yRangeCount);

    std::size_t visitedRootCount = 0;

    // XY候选行数较少时，按每个XY行使用lower_bound扫描对应Z区间。
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
                    visitor(iterator->first, iterator->second);
                    ++visitedRootCount;
                    ++iterator;
                }
            }
        }

        return visitedRootCount;
    }

    // XY候选行数较多时直接过滤现有根树，避免查询大量不存在的网格行。
    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        const VoxelCellIndex& index = iterator->first;

        if (index.x < minimumRootIndex.x || index.x > maximumRootIndex.x ||
            index.y < minimumRootIndex.y || index.y > maximumRootIndex.y ||
            index.z < minimumRootIndex.z || index.z > maximumRootIndex.z)
        {
            continue;
        }

        visitor(index, iterator->second);
        ++visitedRootCount;
    }

    return visitedRootCount;
}

void VoxelForest::visitMaterialCells(const VoxelCellAddress& address, const VoxelTreeCursor& cursor, const MaterialCellVisitor& visitor)
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

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        visitMaterialCells(childCellAddress(address, corner), cursor.child(corner), visitor);
    }
}

}