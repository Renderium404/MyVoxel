#include <cstddef>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Query/VoxelForestQuery.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 创建指定索引和层级的体素地址。
MyVoxel::VoxelCellAddress makeAddress(MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z, MyVoxel::VoxelLevel level)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), level);
}

// 创建指定最小和最大坐标的查询包围盒。
MyVoxel::Bounds3 makeBounds(double minimumX, double minimumY, double minimumZ, double maximumX, double maximumY, double maximumZ)
{
    return MyVoxel::Bounds3(MyMath::Vector3(minimumX, minimumY, minimumZ), MyMath::Vector3(maximumX, maximumY, maximumZ));
}

// 测试空森林查询。
bool testEmptyForest()
{
    const MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);
    const MyVoxel::VoxelForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation = query.classify(makeBounds(0.1, 0.1, 0.1, 0.9, 0.9, 0.9), statistics);

    const bool passed =
        relation == MyVoxel::VoxelRegionRelation::Outside &&
        statistics.rootCandidateCount == 1 &&
        statistics.existingRootCount == 0 &&
        statistics.visitedNodeCount == 0;

    return check(passed, "VoxelForestQuery empty forest");
}

// 测试完全位于材料根节点中的包围盒。
bool testMaterialRootInside()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    forest.setState(root, MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation = query.classify(makeBounds(0.2, 0.2, 0.2, 0.8, 0.8, 0.8), statistics);

    const bool passed =
        relation == MyVoxel::VoxelRegionRelation::Inside &&
        statistics.rootCandidateCount == 1 &&
        statistics.existingRootCount == 1 &&
        statistics.nodeBoundsTestCount == 1 &&
        statistics.visitedNodeCount == 1 &&
        statistics.materialNodeCount == 1;

    return check(passed, "VoxelForestQuery material root inside");
}

// 测试跨越材料根节点边界的包围盒。
bool testPartialRootIntersection()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);

    forest.setState(makeAddress(0, 0, 0, 0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation = query.classify(makeBounds(0.75, 0.25, 0.25, 1.25, 0.75, 0.75), statistics);

    const bool passed =
        relation == MyVoxel::VoxelRegionRelation::Intersecting &&
        statistics.rootCandidateCount == 2 &&
        statistics.existingRootCount == 1 &&
        statistics.materialNodeCount == 1;

    return check(passed, "VoxelForestQuery partial root intersection");
}

// 测试半开体素区间的根边界规则。
bool testHalfOpenRootBoundary()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);

    forest.setState(makeAddress(0, 0, 0, 0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelForestQuery query(forest, grid);

    const MyVoxel::VoxelRegionRelation first = query.classify(makeBounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));
    const MyVoxel::VoxelRegionRelation second = query.classify(makeBounds(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));

    const bool passed =
        first == MyVoxel::VoxelRegionRelation::Inside &&
        second == MyVoxel::VoxelRegionRelation::Outside;

    return check(passed, "VoxelForestQuery half-open root boundary");
}

// 测试细分节点内部和空子节点查询。
bool testSubdividedInsideAndOutside()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress emptyChild = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(emptyChild, MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelForestQuery query(forest, grid);

    const MyVoxel::VoxelRegionRelation materialRelation = query.classify(makeBounds(0.6, 0.6, 0.6, 0.9, 0.9, 0.9));
    const MyVoxel::VoxelRegionRelation emptyRelation = query.classify(makeBounds(0.1, 0.1, 0.1, 0.4, 0.4, 0.4));

    const bool passed =
        materialRelation == MyVoxel::VoxelRegionRelation::Inside &&
        emptyRelation == MyVoxel::VoxelRegionRelation::Outside;

    return check(passed, "VoxelForestQuery subdivided inside and outside");
}

// 测试同时覆盖空子节点和材料子节点的包围盒。
bool testSubdividedIntersection()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress emptyChild = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(emptyChild, MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelForestQuery query(forest, grid);
    const MyVoxel::VoxelRegionRelation relation = query.classify(makeBounds(0.25, 0.25, 0.25, 0.75, 0.75, 0.75));

    return check(relation == MyVoxel::VoxelRegionRelation::Intersecting, "VoxelForestQuery subdivided intersection");
}

// 测试多个材料子节点共同覆盖时保守返回Intersecting。
bool testConservativeMaterialCoverage()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);

    const MyVoxel::VoxelForestQuery query(forest, grid);
    const MyVoxel::VoxelRegionRelation relation = query.classify(makeBounds(0.25, 0.25, 0.25, 0.75, 0.75, 0.75));

    return check(relation == MyVoxel::VoxelRegionRelation::Intersecting, "VoxelForestQuery conservative material coverage");
}

// 测试平移体素网格原点后的空间查询。
bool testTranslatedGrid()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(10.0, 20.0, 30.0), 2.0, 3);

    forest.setState(makeAddress(0, 0, 0, 0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelForestQuery query(forest, grid);

    const MyVoxel::VoxelRegionRelation inside = query.classify(makeBounds(10.25, 20.25, 30.25, 11.75, 21.75, 31.75));
    const MyVoxel::VoxelRegionRelation outside = query.classify(makeBounds(8.25, 18.25, 28.25, 9.75, 19.75, 29.75));

    const bool passed =
        inside == MyVoxel::VoxelRegionRelation::Inside &&
        outside == MyVoxel::VoxelRegionRelation::Outside;

    return check(passed, "VoxelForestQuery translated grid");
}

// 测试查询统计数据累加和重置。
bool testStatisticsAccumulate()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelGrid grid(1.0, 3);

    forest.setState(makeAddress(0, 0, 0, 0), MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics first;
    MyVoxel::VoxelForestQueryStatistics second;

    query.classify(makeBounds(0.1, 0.1, 0.1, 0.9, 0.9, 0.9), first);
    query.classify(makeBounds(2.1, 2.1, 2.1, 2.9, 2.9, 2.9), second);

    first.accumulate(second);

    const bool accumulatedPassed =
        first.rootCandidateCount == 2 &&
        first.existingRootCount == 1 &&
        first.visitedNodeCount == 1 &&
        first.materialNodeCount == 1;

    first.reset();

    const bool resetPassed =
        first.rootCandidateCount == 0 &&
        first.existingRootCount == 0 &&
        first.nodeBoundsTestCount == 0 &&
        first.visitedNodeCount == 0 &&
        first.emptyNodeCount == 0 &&
        first.materialNodeCount == 0 &&
        first.subdividedNodeCount == 0;

    return check(accumulatedPassed && resetPassed, "VoxelForestQuery statistics");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testEmptyForest(),
        testMaterialRootInside(),
        testPartialRootIntersection(),
        testHalfOpenRootBoundary(),
        testSubdividedInsideAndOutside(),
        testSubdividedIntersection(),
        testConservativeMaterialCoverage(),
        testTranslatedGrid(),
        testStatisticsAccumulate()
    };

    const std::size_t testCount = sizeof(results) / sizeof(results[0]);

    for (std::size_t testIndex = 0; testIndex < testCount; ++testIndex)
    {
        if (results[testIndex])
        {
            ++passedCount;
        }
        else
        {
            ++failedCount;
        }
    }

    std::cout << std::endl;
    std::cout << "Voxel forest query tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}