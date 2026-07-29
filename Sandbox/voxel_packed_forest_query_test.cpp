#include <cstdint>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Query/VoxelPackedForestQuery.h"
#include "MyVoxel/Core/Tree/VoxelPackedForest.h"
#include "MyVoxel/Core/VoxelGrid.h"

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

// 返回查询测试使用的体素网格。
MyVoxel::VoxelGrid makeGrid()
{
    return MyVoxel::VoxelGrid(
        MyMath::Vector3(0.0, 0.0, 0.0),
        1.0,
        static_cast<MyVoxel::VoxelLevel>(4));
}

// 返回指定根索引对应的第0层地址。
MyVoxel::VoxelCellAddress rootAddress(int x, int y = 0, int z = 0)
{
    return MyVoxel::VoxelCellAddress(
        MyVoxel::VoxelCellIndex(x, y, z),
        MyVoxel::BaseVoxelLevel);
}

// 创建轴对齐查询包围盒。
MyVoxel::Bounds3 bounds(
    double minimumX,
    double minimumY,
    double minimumZ,
    double maximumX,
    double maximumY,
    double maximumZ)
{
    return MyVoxel::Bounds3(
        MyMath::Vector3(minimumX, minimumY, minimumZ),
        MyMath::Vector3(maximumX, maximumY, maximumZ));
}

// 验证空森林始终返回Outside。
bool testEmptyForest()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    const MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelPackedForestQuery query(forest, grid);

    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation =
        query.classify(
            bounds(0.1, 0.1, 0.1, 0.9, 0.9, 0.9),
            statistics);

    return relation == MyVoxel::VoxelRegionRelation::Outside &&
           statistics.rootCandidateCount == 1 &&
           statistics.existingRootCount == 0 &&
           statistics.visitedNodeCount == 0;
}

// 验证完全位于材料根节点内的区域返回Inside。
bool testMaterialRootInside()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;

    forest.setState(rootAddress(0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation =
        query.classify(
            bounds(0.2, 0.2, 0.2, 0.8, 0.8, 0.8),
            statistics);

    return relation == MyVoxel::VoxelRegionRelation::Inside &&
           statistics.rootCandidateCount == 1 &&
           statistics.existingRootCount == 1 &&
           statistics.nodeBoundsTestCount == 1 &&
           statistics.visitedNodeCount == 1 &&
           statistics.materialNodeCount == 1 &&
           statistics.emptyNodeCount == 0 &&
           statistics.subdividedNodeCount == 0;
}

// 验证跨越材料根边界的区域返回Intersecting。
bool testMaterialRootIntersecting()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;

    forest.setState(rootAddress(0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);

    return query.classify(
               bounds(0.75, 0.25, 0.25, 1.25, 0.75, 0.75)) ==
           MyVoxel::VoxelRegionRelation::Intersecting;
}

// 验证细分根节点中材料子区域的Inside、Intersecting和Outside关系。
bool testSubdividedRootRelations()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress(0);
    const MyVoxel::VoxelCellAddress materialChild = MyVoxel::childCellAddress(root, corner(0));

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);

    for (unsigned int cornerIndex = 1; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        forest.setStateDeferred(
            MyVoxel::childCellAddress(root, corner(cornerIndex)),
            MyVoxel::VoxelState::Empty);
    }

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);

    const MyVoxel::VoxelRegionRelation insideRelation =
        query.classify(bounds(0.1, 0.1, 0.1, 0.4, 0.4, 0.4));

    const MyVoxel::VoxelRegionRelation intersectingRelation =
        query.classify(bounds(0.4, 0.1, 0.1, 0.6, 0.4, 0.4));

    const MyVoxel::VoxelRegionRelation outsideRelation =
        query.classify(bounds(0.6, 0.1, 0.1, 0.9, 0.4, 0.4));

    return forest.state(materialChild) == MyVoxel::VoxelState::Material &&
           insideRelation == MyVoxel::VoxelRegionRelation::Inside &&
           intersectingRelation == MyVoxel::VoxelRegionRelation::Intersecting &&
           outsideRelation == MyVoxel::VoxelRegionRelation::Outside;
}

// 验证跨越两个完整材料根节点的区域保守返回Intersecting。
bool testMultipleMaterialRoots()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;

    forest.setState(rootAddress(0), MyVoxel::VoxelState::Material);
    forest.setState(rootAddress(1), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation =
        query.classify(
            bounds(0.25, 0.25, 0.25, 1.75, 0.75, 0.75),
            statistics);

    return relation == MyVoxel::VoxelRegionRelation::Intersecting &&
           statistics.rootCandidateCount == 2 &&
           statistics.existingRootCount == 2 &&
           statistics.materialNodeCount == 2;
}

// 验证查询统计内部关系一致。
bool testStatisticsConsistency()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress(0);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(
        MyVoxel::childCellAddress(root, corner(7)),
        MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    query.classify(
        bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        statistics);

    return statistics.rootCandidateCount > 0 &&
           statistics.existingRootCount > 0 &&
           statistics.nodeBoundsTestCount >= statistics.visitedNodeCount &&
           statistics.visitedNodeCount ==
               statistics.emptyNodeCount +
               statistics.materialNodeCount +
               statistics.subdividedNodeCount &&
           statistics.subdividedNodeCount > 0 &&
           statistics.materialNodeCount > 0 &&
           statistics.emptyNodeCount > 0;
}

// 验证统计对象在每次classify调用前会自动清空。
bool testStatisticsReset()
{
    const MyVoxel::VoxelGrid grid = makeGrid();
    MyVoxel::VoxelPackedForest forest;

    forest.setState(rootAddress(0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelPackedForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    query.classify(
        bounds(0.1, 0.1, 0.1, 0.9, 0.9, 0.9),
        statistics);

    if (statistics.materialNodeCount != 1)
    {
        return false;
    }

    query.classify(
        bounds(2.1, 0.1, 0.1, 2.9, 0.9, 0.9),
        statistics);

    return statistics.rootCandidateCount == 1 &&
           statistics.existingRootCount == 0 &&
           statistics.nodeBoundsTestCount == 0 &&
           statistics.visitedNodeCount == 0 &&
           statistics.materialNodeCount == 0;
}

}

int main()
{
    check(testEmptyForest(), "Empty forest");
    check(testMaterialRootInside(), "Material root inside");
    check(testMaterialRootIntersecting(), "Material root intersecting");
    check(testSubdividedRootRelations(), "Subdivided root relations");
    check(testMultipleMaterialRoots(), "Multiple material roots");
    check(testStatisticsConsistency(), "Statistics consistency");
    check(testStatisticsReset(), "Statistics reset");

    std::cout << std::endl;
    std::cout << "Voxel packed forest query tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}