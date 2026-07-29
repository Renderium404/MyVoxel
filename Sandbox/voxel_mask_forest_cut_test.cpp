#include <cstdint>
#include <iostream>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/Storage/VoxelLeafBlock.h"
#include "MyVoxel/Core/Tree/VoxelPackedForest.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeEditor.h"
#include "MyVoxel/Operation/Algorithm/VoxelMaskForestCutAlgorithm.h"

namespace
{

int g_passedCount = 0;
int g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[Passed] " << name << std::endl;
        return;
    }

    ++g_failedCount;
    std::cout << "[Failed] " << name << std::endl;
}

// 返回指定数值对应的体素角点。
MyVoxel::VoxelCorner corner(unsigned int value)
{
    return static_cast<MyVoxel::VoxelCorner>(value);
}

// 返回指定索引对应的第0层根地址。
MyVoxel::VoxelCellAddress rootAddress(int x, int y, int z)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), MyVoxel::BaseVoxelLevel);
}

// 在指定根树直接子节点中创建掩码叶块。
bool createRootChildMaskLeaf(
    MyVoxel::VoxelPackedForest& forest,
    const MyVoxel::VoxelCellIndex& rootIndex,
    unsigned int rootCorner,
    std::uint64_t materialMask)
{
    const MyVoxel::VoxelCellAddress rootCell(rootIndex, MyVoxel::BaseVoxelLevel);
    const MyVoxel::VoxelCellAddress childCell = MyVoxel::childCellAddress(rootCell, corner(rootCorner));

    forest.setState(childCell, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelPackedRootTree* tree = forest.detachRootTree(rootIndex);

    if (!tree)
    {
        return false;
    }

    MyVoxel::VoxelPackedTreeEditor rootEditor(*tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(rootCorner));

    if (!leafEditor.makeMaskLeaf())
    {
        return false;
    }

    leafEditor.maskLeaf().materialMask = materialMask;
    return true;
}

// 返回指定根树直接子节点对应的材料掩码。
std::uint64_t rootChildMask(
    const MyVoxel::VoxelPackedForest& forest,
    const MyVoxel::VoxelCellIndex& rootIndex,
    unsigned int rootCorner)
{
    const MyVoxel::VoxelPackedRootTree* tree = forest.rootTree(rootIndex);

    if (!tree)
    {
        return 0;
    }

    MyVoxel::VoxelPackedTreeConstCursor rootCursor(*tree);
    const MyVoxel::VoxelPackedTreeConstCursor childCursor = rootCursor.child(corner(rootCorner));
    const MyVoxel::VoxelState childState = childCursor.state();

    if (childState == MyVoxel::VoxelState::Empty)
    {
        return 0;
    }

    if (childState == MyVoxel::VoxelState::Material)
    {
        return MyVoxel::VoxelLeafBlock::fullMask();
    }

    return childCursor.isMaskLeaf() ? childCursor.maskLeaf().materialMask : 0;
}

// 验证空工具森林不会修改对象。
bool testEmptyToolForest()
{
    MyVoxel::VoxelPackedForest objectForest;
    const MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    objectForest.setState(rootAddress(0, 0, 0), MyVoxel::VoxelState::Material);

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, &changes, &statistics);

    return !changed &&
           objectForest.rootCount() == 1 &&
           objectForest.state(rootAddress(0, 0, 0)) == MyVoxel::VoxelState::Material &&
           !changes.hasChanges() &&
           statistics.rootCandidateCount == 0;
}

// 验证工具根不存在对应对象根时直接跳过。
bool testMissingObjectRoot()
{
    MyVoxel::VoxelPackedForest objectForest;
    MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    objectForest.setState(rootAddress(0, 0, 0), MyVoxel::VoxelState::Material);
    toolForest.setState(rootAddress(4, 0, 0), MyVoxel::VoxelState::Material);

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, nullptr, &statistics);

    return !changed &&
           objectForest.rootCount() == 1 &&
           statistics.rootCandidateCount == 1 &&
           statistics.existingObjectRootCount == 0 &&
           statistics.detachedRootCount == 0;
}

// 验证完整材料工具根直接删除对象根。
bool testDirectRootErase()
{
    const MyVoxel::VoxelCellIndex index(1, 2, 3);

    MyVoxel::VoxelPackedForest objectForest;
    MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    objectForest.setState(MyVoxel::VoxelCellAddress(index, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);
    toolForest.setState(MyVoxel::VoxelCellAddress(index, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, &changes, &statistics);

    return changed &&
           objectForest.rootTree(index) == nullptr &&
           changes.modifiedRootCount() == 1 &&
           changes.containsModifiedRoot(index) &&
           statistics.directRootEraseCount == 1 &&
           statistics.detachedRootCount == 0;
}

// 验证同索引掩码叶块直接执行64位差集。
bool testMaskLeafCut()
{
    const MyVoxel::VoxelCellIndex index(0, 0, 0);
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0xFFFF0000FFFF0000ULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0x00FF00FF00FF00FFULL);

    MyVoxel::VoxelPackedForest objectForest;
    MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    if (!createRootChildMaskLeaf(objectForest, index, 3, objectMask) ||
        !createRootChildMaskLeaf(toolForest, index, 3, toolMask))
    {
        return false;
    }

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, &changes, &statistics);

    return changed &&
           rootChildMask(objectForest, index, 3) == (objectMask & ~toolMask) &&
           changes.containsModifiedRoot(index) &&
           statistics.intersectingRootCount == 1 &&
           statistics.detachedRootCount == 1 &&
           statistics.treeStatistics.maskOperationCount == 1 &&
           statistics.treeStatistics.maskChangedCount == 1;
}

// 验证无交集掩码不会触发根级写时复制。
bool testDisjointMaskPreservesSharing()
{
    const MyVoxel::VoxelCellIndex index(2, 0, 0);
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0x00000000FFFFFFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0xFFFFFFFF00000000ULL);

    MyVoxel::VoxelPackedForest sourceForest;
    MyVoxel::VoxelPackedForest toolForest;

    if (!createRootChildMaskLeaf(sourceForest, index, 4, objectMask) ||
        !createRootChildMaskLeaf(toolForest, index, 4, toolMask))
    {
        return false;
    }

    MyVoxel::VoxelPackedForest resultForest = sourceForest;

    const MyVoxel::VoxelPackedRootTree* sourceTreeBefore = sourceForest.rootTree(index);
    const MyVoxel::VoxelPackedRootTree* resultTreeBefore = resultForest.rootTree(index);

    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;
    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(resultForest, toolForest, nullptr, &statistics);

    return !changed &&
           sourceTreeBefore == resultTreeBefore &&
           sourceForest.rootTree(index) == resultForest.rootTree(index) &&
           rootChildMask(resultForest, index, 4) == objectMask &&
           statistics.intersectionMaskTestCount == 1 &&
           statistics.intersectingRootCount == 0 &&
           statistics.detachedRootCount == 0;
}

// 验证实际修改时只分离结果森林对应根树。
bool testModifiedRootCopyOnWrite()
{
    const MyVoxel::VoxelCellIndex index(3, 0, 0);
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0xFFFFFFFF0000FFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0x00000000000000FFULL);

    MyVoxel::VoxelPackedForest sourceForest;
    MyVoxel::VoxelPackedForest toolForest;

    if (!createRootChildMaskLeaf(sourceForest, index, 6, objectMask) ||
        !createRootChildMaskLeaf(toolForest, index, 6, toolMask))
    {
        return false;
    }

    MyVoxel::VoxelPackedForest resultForest = sourceForest;

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(resultForest, toolForest);

    return changed &&
           sourceForest.rootTree(index) != resultForest.rootTree(index) &&
           rootChildMask(sourceForest, index, 6) == objectMask &&
           rootChildMask(resultForest, index, 6) == (objectMask & ~toolMask);
}

// 验证多个工具根只修改实际存在材料交集的对象根。
bool testMultipleRoots()
{
    const MyVoxel::VoxelCellIndex firstIndex(0, 0, 0);
    const MyVoxel::VoxelCellIndex secondIndex(1, 0, 0);
    const MyVoxel::VoxelCellIndex thirdIndex(2, 0, 0);
    const MyVoxel::VoxelCellIndex missingIndex(3, 0, 0);

    MyVoxel::VoxelPackedForest objectForest;
    MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    objectForest.setState(MyVoxel::VoxelCellAddress(firstIndex, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);
    objectForest.setState(MyVoxel::VoxelCellAddress(thirdIndex, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);

    if (!createRootChildMaskLeaf(objectForest, secondIndex, 2, static_cast<std::uint64_t>(0xFFFFFFFFFFFFFFFFULL)) ||
        !createRootChildMaskLeaf(toolForest, secondIndex, 2, static_cast<std::uint64_t>(0x000000000000FFFFULL)))
    {
        return false;
    }

    toolForest.setState(MyVoxel::VoxelCellAddress(firstIndex, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);
    toolForest.setState(MyVoxel::VoxelCellAddress(missingIndex, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, &changes, &statistics);

    return changed &&
           objectForest.rootTree(firstIndex) == nullptr &&
           objectForest.rootTree(secondIndex) != nullptr &&
           objectForest.rootTree(thirdIndex) != nullptr &&
           changes.modifiedRootCount() == 2 &&
           changes.containsModifiedRoot(firstIndex) &&
           changes.containsModifiedRoot(secondIndex) &&
           !changes.containsModifiedRoot(thirdIndex) &&
           statistics.rootCandidateCount == 3 &&
           statistics.existingObjectRootCount == 2 &&
           statistics.modifiedRootCount == 2;
}

// 验证局部根切削为空后能够从森林移除。
bool testEmptyResultRootErase()
{
    const MyVoxel::VoxelCellIndex index(5, 0, 0);
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0x000000000000FFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0x000000000000FFFFULL);

    MyVoxel::VoxelPackedForest objectForest;
    MyVoxel::VoxelPackedForest toolForest;
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics statistics;

    if (!createRootChildMaskLeaf(objectForest, index, 1, objectMask) ||
        !createRootChildMaskLeaf(toolForest, index, 1, toolMask))
    {
        return false;
    }

    const bool changed =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelForest(objectForest, toolForest, nullptr, &statistics);

    return changed &&
           objectForest.rootTree(index) == nullptr &&
           statistics.emptiedRootEraseCount == 1 &&
           statistics.modifiedRootCount == 1;
}

}

int main()
{
    check(testEmptyToolForest(), "Empty tool forest");
    check(testMissingObjectRoot(), "Missing object root");
    check(testDirectRootErase(), "Direct root erase");
    check(testMaskLeafCut(), "Mask leaf forest cut");
    check(testDisjointMaskPreservesSharing(), "Disjoint mask preserves sharing");
    check(testModifiedRootCopyOnWrite(), "Modified root copy-on-write");
    check(testMultipleRoots(), "Multiple roots");
    check(testEmptyResultRootErase(), "Empty result root erase");

    std::cout << std::endl;
    std::cout << "Voxel mask forest cut tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}