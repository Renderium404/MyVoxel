#include <cstdint>
#include <iostream>

#include "MyVoxel/Core/Storage/VoxelLeafBlock.h"
#include "MyVoxel/Core/Tree/VoxelPackedRootTree.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeEditor.h"
#include "MyVoxel/Operation/Algorithm/VoxelMaskCutAlgorithm.h"

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

// 在指定根树直接子节点中创建掩码叶块。
void createRootChildMaskLeaf(MyVoxel::VoxelPackedRootTree& tree, unsigned int rootCorner, std::uint64_t materialMask)
{
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);

    if (rootEditor.state() != MyVoxel::VoxelState::Subdivided)
    {
        rootEditor.split();
    }

    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(rootCorner));

    if (!leafEditor.isMaskLeaf())
    {
        const bool created = leafEditor.makeMaskLeaf();

        if (!created)
        {
            leafEditor.setState(MyVoxel::VoxelState::Empty);
            const bool recreated = leafEditor.makeMaskLeaf();

            if (!recreated)
            {
                return;
            }
        }
    }

    leafEditor.maskLeaf().materialMask = materialMask;
}

// 返回指定根树直接子节点中的掩码。
std::uint64_t rootChildMask(const MyVoxel::VoxelPackedRootTree& tree, unsigned int rootCorner)
{
    MyVoxel::VoxelPackedTreeConstCursor rootCursor(tree);
    const MyVoxel::VoxelPackedTreeConstCursor childCursor = rootCursor.child(corner(rootCorner));

    return childCursor.isMaskLeaf() ? childCursor.maskLeaf().materialMask : 0;
}

// 验证空刀具不会修改对象根树。
bool testEmptyTool()
{
    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Material);
    const MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);
    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;

    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    return !result.changed &&
           result.state == MyVoxel::VoxelState::Material &&
           objectTree.rootState == MyVoxel::VoxelState::Material &&
           statistics.visitedCellCount == 1 &&
           statistics.emptyToolSkipCount == 1 &&
           statistics.maskOperationCount == 0;
}

// 验证完整材料刀具直接删除整个对象根树。
bool testMaterialTool()
{
    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Material);
    const MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Material);
    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;

    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    return result.changed &&
           result.state == MyVoxel::VoxelState::Empty &&
           objectTree.rootState == MyVoxel::VoxelState::Empty &&
           statistics.materialToolRemoveCount == 1 &&
           statistics.maskOperationCount == 0;
}

// 验证材料对象能够跟随刀具创建掩码叶块并直接执行64位差集。
bool testCreateObjectMaskLeaf()
{
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0x00FF00FF00FF00FFULL);

    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);

    createRootChildMaskLeaf(toolTree, 3, toolMask);

    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;
    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    MyVoxel::VoxelPackedTreeConstCursor objectCursor(objectTree);
    const MyVoxel::VoxelPackedTreeConstCursor resultLeaf = objectCursor.child(corner(3));

    return result.changed &&
           result.state == MyVoxel::VoxelState::Subdivided &&
           resultLeaf.isMaskLeaf() &&
           resultLeaf.maskLeaf().materialMask == ~toolMask &&
           statistics.createdBranchCount == 1 &&
           statistics.createdMaskLeafCount == 1 &&
           statistics.maskOperationCount == 1 &&
           statistics.maskChangedCount == 1;
}

// 验证两个掩码没有交集时对象数据和逻辑状态保持不变。
bool testDisjointMasks()
{
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0x00000000FFFFFFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0xFFFFFFFF00000000ULL);

    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Empty);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);

    createRootChildMaskLeaf(objectTree, 2, objectMask);
    createRootChildMaskLeaf(toolTree, 2, toolMask);

    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;
    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    return !result.changed &&
           result.state == MyVoxel::VoxelState::Subdivided &&
           rootChildMask(objectTree, 2) == objectMask &&
           statistics.maskOperationCount == 1 &&
           statistics.maskChangedCount == 0;
}

// 验证掩码中的全部对象材料被删除后整棵根树能够折叠为空。
bool testCompleteMaskRemoval()
{
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0x0000FFFF0000FFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0xFFFFFFFFFFFFFFFFULL);

    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Empty);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);

    createRootChildMaskLeaf(objectTree, 5, objectMask);
    createRootChildMaskLeaf(toolTree, 5, toolMask);

    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;
    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    return result.changed &&
           result.state == MyVoxel::VoxelState::Empty &&
           objectTree.rootState == MyVoxel::VoxelState::Empty &&
           objectTree.blockPool.allocatedGroupCount() == 0 &&
           statistics.maskOperationCount == 1 &&
           statistics.mergeSuccessCount >= 1;
}

// 验证一个根树中的多个掩码叶块能够分别执行一次64位差集。
bool testMultipleMaskLeaves()
{
    const std::uint64_t firstObjectMask = static_cast<std::uint64_t>(0xFFFFFFFFFFFFFFFFULL);
    const std::uint64_t secondObjectMask = static_cast<std::uint64_t>(0x0F0F0F0F0F0F0F0FULL);
    const std::uint64_t firstToolMask = static_cast<std::uint64_t>(0x00FF00FF00FF00FFULL);
    const std::uint64_t secondToolMask = static_cast<std::uint64_t>(0x00000000000000FFULL);

    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Empty);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);

    createRootChildMaskLeaf(objectTree, 1, firstObjectMask);
    createRootChildMaskLeaf(objectTree, 6, secondObjectMask);
    createRootChildMaskLeaf(toolTree, 1, firstToolMask);
    createRootChildMaskLeaf(toolTree, 6, secondToolMask);

    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;
    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    return result.changed &&
           rootChildMask(objectTree, 1) == (firstObjectMask & ~firstToolMask) &&
           rootChildMask(objectTree, 6) == (secondObjectMask & ~secondToolMask) &&
           statistics.maskOperationCount == 2 &&
           statistics.maskChangedCount == 2;
}

// 验证同步递归能够穿过普通分支后在更深层执行掩码差集。
bool testBranchToMaskTraversal()
{
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0xAAAAAAAAAAAAAAAAULL);

    MyVoxel::VoxelPackedRootTree objectTree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);
    MyVoxel::VoxelPackedTreeEditor toolRootEditor(toolTree);

    toolRootEditor.split();

    MyVoxel::VoxelPackedTreeEditor branchEditor = toolRootEditor.child(corner(0));

    branchEditor.split();

    MyVoxel::VoxelPackedTreeEditor leafEditor = branchEditor.child(corner(4));

    leafEditor.makeMaskLeaf();
    leafEditor.maskLeaf().materialMask = toolMask;

    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics statistics;
    const MyVoxel::Operation::Algorithm::VoxelMaskCutResult result =
        MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(objectTree, toolTree, &statistics);

    MyVoxel::VoxelPackedTreeConstCursor objectCursor(objectTree);
    const MyVoxel::VoxelPackedTreeConstCursor resultLeaf =
        objectCursor.child(corner(0)).child(corner(4));

    return result.changed &&
           resultLeaf.isMaskLeaf() &&
           resultLeaf.maskLeaf().materialMask == ~toolMask &&
           statistics.createdBranchCount == 2 &&
           statistics.createdMaskLeafCount == 1 &&
           statistics.maskOperationCount == 1;
}

// 验证对象根树深复制后切削结果不会修改原始根树。
bool testObjectCopyPreservation()
{
    const std::uint64_t objectMask = static_cast<std::uint64_t>(0xFFFFFFFF0000FFFFULL);
    const std::uint64_t toolMask = static_cast<std::uint64_t>(0x00000000000000FFULL);

    MyVoxel::VoxelPackedRootTree sourceTree(MyVoxel::VoxelState::Empty);
    MyVoxel::VoxelPackedRootTree toolTree(MyVoxel::VoxelState::Empty);

    createRootChildMaskLeaf(sourceTree, 7, objectMask);
    createRootChildMaskLeaf(toolTree, 7, toolMask);

    MyVoxel::VoxelPackedRootTree resultTree = sourceTree;

    MyVoxel::Operation::Algorithm::cutAlignedVoxelTree(resultTree, toolTree);

    return rootChildMask(sourceTree, 7) == objectMask &&
           rootChildMask(resultTree, 7) == (objectMask & ~toolMask);
}

}

int main()
{
    check(testEmptyTool(), "Empty tool");
    check(testMaterialTool(), "Material tool");
    check(testCreateObjectMaskLeaf(), "Create object mask leaf");
    check(testDisjointMasks(), "Disjoint masks");
    check(testCompleteMaskRemoval(), "Complete mask removal");
    check(testMultipleMaskLeaves(), "Multiple mask leaves");
    check(testBranchToMaskTraversal(), "Branch to mask traversal");
    check(testObjectCopyPreservation(), "Object copy preservation");

    std::cout << std::endl;
    std::cout << "Voxel mask cut tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}