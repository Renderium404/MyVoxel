#include "VoxelForest.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "VoxelTreeCursor.h"
#include "VoxelTreeEditor.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Storage/LeafBlock.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const float DefaultVoxelBackgroundDistance = 1.0f; // 独立默认VoxelForest兼容旧构造固定使用1个长度单位的截断背景距离。
const unsigned int MaskLeafCoveredLevelCount = 2; // 一个MaskLeaf入口下面固定展开两级并保存64个最高层距离样本。
const unsigned int MinimumExplicitDistanceLevel = MaskLeafCoveredLevelCount + 1; // MaskLeaf必须位于NodeBlock子槽，因此首个可显式保存距离的目标层级为第3层。

bool isValidBackgroundDistance(float backgroundDistance)
{
    return std::isfinite(static_cast<double>(backgroundDistance)) && backgroundDistance > 0.0f;
}

bool isValidTsdfDistance(float distance, float backgroundDistance)
{
    return std::isfinite(static_cast<double>(distance)) && distance >= -backgroundDistance && distance <= backgroundDistance;
}

float terminalDistance(MyVoxel::VoxelState state, float backgroundDistance)
{
    assert(state == MyVoxel::VoxelState::Empty || state == MyVoxel::VoxelState::Material);
    return state == MyVoxel::VoxelState::Material ? -backgroundDistance : backgroundDistance;
}

std::uint64_t inclusiveIndexRangeLength(MyVoxel::VoxelIndex minimumValue, MyVoxel::VoxelIndex maximumValue)
{
    assert(minimumValue <= maximumValue);
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(maximumValue) - static_cast<std::int64_t>(minimumValue)) + 1;
}

std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    const std::uint64_t maximumValue = (std::numeric_limits<std::uint64_t>::max)();
    return first > maximumValue / second ? maximumValue : first * second;
}

MyVoxel::VoxelTreeEditor ensureEditorAt(MyVoxel::VoxelTree& tree, const std::vector<MyVoxel::VoxelCorner>& path, float backgroundDistance)
{
    MyVoxel::VoxelTreeEditor editor = tree.editor(backgroundDistance);
    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        editor = editor.child(path[pathIndex]);
    }
    return editor;
}

MyVoxel::VoxelTreeCursor cursorAt(const MyVoxel::VoxelTree& tree, const std::vector<MyVoxel::VoxelCorner>& path, bool& exact)
{
    MyVoxel::VoxelTreeCursor cursor = tree.cursor();
    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        if (!cursor.isSubdivided())
        {
            exact = false;
            return cursor;
        }
        cursor = cursor.child(path[pathIndex]);
    }
    exact = true;
    return cursor;
}

float cursorDistance(const MyVoxel::VoxelTreeCursor& cursor)
{
    MYVOXEL_ASSERT_MESSAGE(cursor.hasDistance(), "Voxel distance query must resolve to a terminal Tile Value or explicit LeafBlock sample.");
    return cursor.distance();
}

bool cursorDirectUniformValue(const MyVoxel::VoxelTreeCursor& cursor, float& value)
{
    if (cursor.hasLeafData())
    {
        const MyVoxel::LeafBlock& block = cursor.leafBlock();
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

    if (!cursor.isSubdivided())
    {
        value = cursor.distance();
        return true;
    }

    bool initialized = false;
    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelTreeCursor child = cursor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex));
        if (!child.isTerminal())
        {
            return false;
        }

        const float childValue = child.distance();
        if (!initialized)
        {
            value = childValue;
            initialized = true;
        }
        else if (childValue != value)
        {
            return false;
        }
    }
    return initialized;
}

void fillLeafBlockFromField(const MyVoxel::VoxelTreeCursor& entryCursor, MyVoxel::LeafBlock& block)
{
    if (entryCursor.isTerminal())
    {
        MyVoxel::reset(block, entryCursor.distance());
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(entryCursor.isSubdivided(), "MaskLeaf materialization source must be terminal or subdivided.");

    for (unsigned int coarseIndex = 0; coarseIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++coarseIndex)
    {
        const MyVoxel::VoxelCorner coarseCorner = static_cast<MyVoxel::VoxelCorner>(coarseIndex);
        const MyVoxel::VoxelTreeCursor coarseCursor = entryCursor.child(coarseCorner);

        for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++fineIndex)
        {
            const MyVoxel::VoxelCorner fineCorner = static_cast<MyVoxel::VoxelCorner>(fineIndex);
            const float distance = coarseCursor.isTerminal() ? coarseCursor.distance() : cursorDistance(coarseCursor.child(fineCorner));
            block.distances[MyVoxel::leafSampleIndex(coarseCorner, fineCorner)] = distance;
        }
    }
}

}

namespace MyVoxel
{

VoxelForest::VoxelForest()
    : m_backgroundDistance(DefaultVoxelBackgroundDistance)
{
}

VoxelForest::VoxelForest(float backgroundDistanceValue)
    : m_backgroundDistance(backgroundDistanceValue)
{
    MYVOXEL_ASSERT_MESSAGE(isValidBackgroundDistance(m_backgroundDistance), "VoxelForest background distance must be finite and positive.");
}

VoxelForest::VoxelForest(VoxelForest&& other)
    : m_backgroundDistance(other.m_backgroundDistance)
    , m_trees(std::move(other.m_trees))
{
    other.m_backgroundDistance = DefaultVoxelBackgroundDistance;
    other.m_trees.clear();
}

VoxelForest& VoxelForest::operator=(VoxelForest&& other)
{
    if (this == &other)
    {
        return *this;
    }

    m_backgroundDistance = other.m_backgroundDistance;
    m_trees = std::move(other.m_trees);
    other.m_backgroundDistance = DefaultVoxelBackgroundDistance;
    other.m_trees.clear();
    return *this;
}

/// 场属性

bool VoxelForest::isValid() const
{
    if (!isValidBackgroundDistance(m_backgroundDistance))
    {
        return false;
    }

    for (TreeMap::const_iterator iterator = m_trees.begin(); iterator != m_trees.end(); ++iterator)
    {
        if (!iterator->second.isTsdfValid(m_backgroundDistance) || isBackgroundTree(iterator->second))
        {
            return false;
        }
    }
    return true;
}

float VoxelForest::backgroundDistance() const
{
    return m_backgroundDistance;
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
    return cursorAt(*tree, path, exact).state();
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
    cursorAt(*tree, path, exact);
    return exact;
}

bool VoxelForest::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const float targetDistance = terminalDistance(stateValue, m_backgroundDistance);
    VoxelTree* existingTree = findTree(rootAddress.index);

    if (!existingTree)
    {
        if (stateValue == VoxelState::Empty)
        {
            return false;
        }
    }
    else
    {
        std::vector<VoxelCorner> existingPath;
        MyVoxel::buildCornerPath(rootAddress, address, existingPath);
        bool exact = false;
        const VoxelTreeCursor current = cursorAt(*existingTree, existingPath, exact);
        if (current.hasDistance() && current.distance() == targetDistance)
        {
            return false;
        }
    }

    if (address.level == BaseVoxelLevel && stateValue == VoxelState::Empty)
    {
        return m_trees.erase(rootAddress.index) > 0;
    }

    VoxelTree& tree = ensureTree(rootAddress.index, VoxelState::Empty, m_backgroundDistance);
    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);
    VoxelTreeEditor editor = ensureEditorAt(tree, path, m_backgroundDistance);

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

/// 距离场

float VoxelForest::distance(const VoxelCellAddress& address) const
{
    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    const VoxelTree* tree = findTree(rootAddress.index);
    if (!tree)
    {
        return m_backgroundDistance;
    }

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);
    bool exact = false;
    return cursorDistance(cursorAt(*tree, path, exact));
}

bool VoxelForest::setDistance(const VoxelCellAddress& address, float newDistance)
{
    MYVOXEL_ASSERT_MESSAGE(isValidTsdfDistance(newDistance, m_backgroundDistance), "Voxel distance must be finite and inside the Forest TSDF range.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<unsigned int>(address.level) >= MinimumExplicitDistanceLevel, "Explicit TSDF samples require a target level of at least 3.");

    if (distance(address) == newDistance)
    {
        return false;
    }

    const VoxelCellAddress rootAddress = MyVoxel::rootCellAddress(address);
    VoxelTree& tree = ensureTree(rootAddress.index, VoxelState::Empty, m_backgroundDistance);

    std::vector<VoxelCorner> path;
    MyVoxel::buildCornerPath(rootAddress, address, path);
    MYVOXEL_ASSERT_MESSAGE(path.size() >= MaskLeafCoveredLevelCount + 1U, "Explicit TSDF sample path is too short for MaskLeaf storage.");

    const std::size_t leafEntryPathCount = path.size() - MaskLeafCoveredLevelCount;
    std::vector<VoxelCorner> leafEntryPath(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(leafEntryPathCount));

    bool exact = false;
    const VoxelTreeCursor sourceCursor = cursorAt(tree, leafEntryPath, exact);
    LeafBlock block;
    fillLeafBlockFromField(sourceCursor, block);

    const VoxelCorner coarseCorner = path[leafEntryPathCount];
    const VoxelCorner fineCorner = path[leafEntryPathCount + 1U];
    block.distances[leafSampleIndex(coarseCorner, fineCorner)] = newDistance;

    VoxelTreeEditor editor = ensureEditorAt(tree, leafEntryPath, m_backgroundDistance);
    MYVOXEL_ASSERT_MESSAGE(editor.canSetLeafBlock(), "MaskLeaf entry must resolve to a normal Node child editor.");
    return editor.setLeafBlock(block);
}

/// 节点结构

bool VoxelForest::split(const VoxelCellAddress& address)
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
    const VoxelTreeCursor current = cursorAt(*tree, path, exact);
    if (exact && current.isSubdivided())
    {
        return false;
    }
    if (!current.isTerminal())
    {
        return false;
    }

    VoxelTreeEditor editor = ensureEditorAt(*tree, path, m_backgroundDistance);
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
    const VoxelTreeCursor cursor = cursorAt(*tree, path, exact);
    if (!exact || !cursor.isSubdivided())
    {
        return false;
    }

    if (!cursor.hasLeafData())
    {
        for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
        {
            if (cursor.child(static_cast<VoxelCorner>(cornerIndex)).hasLeafDistance())
            {
                return false;
            }
        }
    }

    float mergedValue = 0.0f;
    if (!cursorDirectUniformValue(cursor, mergedValue))
    {
        return false;
    }

    VoxelTreeEditor editor = ensureEditorAt(*tree, path, m_backgroundDistance);
    if (!editor.setValue(mergedValue))
    {
        return false;
    }

    if (address.level == BaseVoxelLevel && tree->state() == VoxelState::Empty && tree->value() == m_backgroundDistance)
    {
        m_trees.erase(rootAddress.index);
    }
    return true;
}

bool VoxelForest::pruneTree(const VoxelCellIndex& rootIndex)
{
    TreeMap::iterator iterator = m_trees.find(rootIndex);
    if (iterator == m_trees.end())
    {
        return false;
    }

    const bool changed = iterator->second.prune(m_backgroundDistance);
    if (isBackgroundTree(iterator->second))
    {
        m_trees.erase(iterator);
        return true;
    }
    return changed;
}

std::size_t VoxelForest::prune()
{
    std::size_t changedRootCount = 0;
    TreeMap::iterator iterator = m_trees.begin();

    while (iterator != m_trees.end())
    {
        const bool changed = iterator->second.prune(m_backgroundDistance);
        if (isBackgroundTree(iterator->second))
        {
            iterator = m_trees.erase(iterator);
            ++changedRootCount;
            continue;
        }
        if (changed)
        {
            ++changedRootCount;
        }
        ++iterator;
    }

    return changedRootCount;
}

/// 森林管理

bool VoxelForest::isEmpty() const{return m_trees.empty();}
std::size_t VoxelForest::rootCount() const{return m_trees.size();}
void VoxelForest::clear(){m_trees.clear();}

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

bool VoxelForest::eraseTree(const VoxelCellIndex& rootIndex){return m_trees.erase(rootIndex) > 0;}
const VoxelTree* VoxelForest::getTree(const VoxelCellIndex& rootIndex) const{return findTree(rootIndex);}
VoxelTree* VoxelForest::getTree(const VoxelCellIndex& rootIndex){return findTree(rootIndex);}

VoxelTree& VoxelForest::setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree)
{
    assert(tree.isTsdfValid(m_backgroundDistance));
    assert(!isBackgroundTree(tree));

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
    assert(tree.isTsdfValid(m_backgroundDistance));
    assert(!isBackgroundTree(tree));

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
    return visitTreesInRange(minimumRootIndex, maximumRootIndex,
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
    return visitTreesInRange(minimumRootIndex, maximumRootIndex,
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

VoxelTree& VoxelForest::ensureTree(const VoxelCellIndex& rootIndex, VoxelState initialState, float initialValue)
{
    assert(initialState == VoxelState::Empty || initialState == VoxelState::Material);
    std::pair<TreeMap::iterator, bool> result = m_trees.insert(std::make_pair(rootIndex, VoxelTree(initialState, initialValue)));
    return result.first->second;
}

bool VoxelForest::isBackgroundTree(const VoxelTree& tree) const
{
    return tree.state() == VoxelState::Empty && tree.value() == m_backgroundDistance;
}

std::size_t VoxelForest::visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const
{
    assert(visitor);
    assert(minimumRootIndex.x <= maximumRootIndex.x && minimumRootIndex.y <= maximumRootIndex.y && minimumRootIndex.z <= maximumRootIndex.z);

    if (m_trees.empty())
    {
        return 0;
    }

    const std::uint64_t xRangeCount = inclusiveIndexRangeLength(minimumRootIndex.x, maximumRootIndex.x);
    const std::uint64_t yRangeCount = inclusiveIndexRangeLength(minimumRootIndex.y, maximumRootIndex.y);
    const std::uint64_t xyRangeCount = saturatedMultiply(xRangeCount, yRangeCount);
    std::size_t visitedRootCount = 0;

    if (xyRangeCount <= static_cast<std::uint64_t>(m_trees.size()))
    {
        for (std::int64_t x = static_cast<std::int64_t>(minimumRootIndex.x); x <= static_cast<std::int64_t>(maximumRootIndex.x); ++x)
        {
            const VoxelIndex xIndex = static_cast<VoxelIndex>(x);
            for (std::int64_t y = static_cast<std::int64_t>(minimumRootIndex.y); y <= static_cast<std::int64_t>(maximumRootIndex.y); ++y)
            {
                const VoxelIndex yIndex = static_cast<VoxelIndex>(y);
                TreeMap::const_iterator iterator = m_trees.lower_bound(VoxelCellIndex(xIndex, yIndex, minimumRootIndex.z));
                while (iterator != m_trees.end() && iterator->first.x == xIndex && iterator->first.y == yIndex && iterator->first.z <= maximumRootIndex.z)
                {
                    visitor(iterator->first, iterator->second);
                    ++visitedRootCount;
                    ++iterator;
                }
            }
        }
        return visitedRootCount;
    }

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