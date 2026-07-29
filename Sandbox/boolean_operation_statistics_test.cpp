#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include "BooleanOperationTestCommon.h"
#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Operation/BooleanOperation.h"

#if !defined(MYVOXEL_TEST)
#error boolean_operation_statistics_test requires MYVOXEL_TEST.
#endif

#if !MYVOXEL_BOOLEAN_STATISTICS_ENABLED
#error Boolean operation statistics must be enabled in the test build.
#endif

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 测试连续刀具无统计和统计路径结果完全一致。
bool testContinuousResultConsistency()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    MyVoxel::Operation::BooleanOperationStatistics statistics;

    const MyVoxel::VoxelShape normalResult = MyVoxel::Operation::cut(workpiece, tool);
    const MyVoxel::VoxelShape statisticsResult = MyVoxel::Operation::cut(workpiece, tool, statistics);

    return check(
        BooleanOperationTest::occupancySignature(normalResult).isEqualTo(BooleanOperationTest::occupancySignature(statisticsResult)),
        "Statistics continuous result consistency");
}

// 测试体素刀具无统计和统计路径结果完全一致。
bool testVoxelResultConsistency()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::VoxelShape tool = BooleanOperationTest::makeVoxelTool(workpiece);
    MyVoxel::Operation::BooleanOperationStatistics statistics;

    const MyVoxel::VoxelShape normalResult = MyVoxel::Operation::cut(workpiece, tool);
    const MyVoxel::VoxelShape statisticsResult = MyVoxel::Operation::cut(workpiece, tool, statistics);

    return check(
        BooleanOperationTest::occupancySignature(normalResult).isEqualTo(BooleanOperationTest::occupancySignature(statisticsResult)),
        "Statistics voxel result consistency");
}

// 测试连续刀具统计内部关系。
bool testContinuousStatistics()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::BooleanOperationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes, statistics);

    const bool passed =
        !result.isEmpty() &&
        changes.hasChanges() &&
        statistics.totalMilliseconds >= 0.0 &&
        statistics.planningMilliseconds >= 0.0 &&
        statistics.rootPreparationMilliseconds >= 0.0 &&
        statistics.planApplicationMilliseconds >= 0.0 &&
        statistics.planningWorkerCount > 0 &&
        statistics.cutPlanNodeCount > 0 &&
        statistics.cutPlanGroupCount > 0 &&
        statistics.preparedRootTreeCount > 0 &&
        statistics.broadPhaseRootCandidateCount >= statistics.broadPhaseExistingRootCount &&
        statistics.inputMaterialCellCount == statistics.broadPhaseExistingRootCount &&
        statistics.classifiedCellCount == statistics.outsideCellCount + statistics.insideCellCount + statistics.intersectingCellCount &&
        statistics.pointQueryCount == statistics.centerSampleCount &&
        statistics.queryVisitedNodeCount == 0 &&
        statistics.accessorNodeVisitCount == 0;

    return check(passed, "Statistics continuous counters");
}

// 测试体素刀具统计内部关系。// 测试体素刀具统计内部关系。
bool testVoxelStatistics()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const BooleanOperationTest::OccupancySignature sourceSignature = BooleanOperationTest::occupancySignature(workpiece);
    const MyVoxel::VoxelShape tool = BooleanOperationTest::makeVoxelTool(workpiece);
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::BooleanOperationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes, statistics);
    const BooleanOperationTest::OccupancySignature resultSignature = BooleanOperationTest::occupancySignature(result);

    const bool removedMaterial = resultSignature.voxelCount < sourceSignature.voxelCount;
    const bool retainedMaterial = resultSignature.voxelCount > 0;
    const bool changesRecorded = changes.hasChanges();
    const bool timingValid = statistics.totalMilliseconds >= 0.0;
    const bool recursionEntered = statistics.inputMaterialCellCount > 0 && statistics.visitedCellCount > 0;
    const bool classificationConsistent =
        statistics.classifiedCellCount ==
        statistics.outsideCellCount +
        statistics.insideCellCount +
        statistics.intersectingCellCount;
    const bool centerQueryConsistent = statistics.pointQueryCount == statistics.centerSampleCount;
    const bool queryStatisticsConsistent =
        statistics.classifiedCellCount == 0 ||
        statistics.queryRootCandidateCount > 0;
    const bool accessorStatisticsConsistent =
        statistics.pointQueryCount == 0
            ? statistics.accessorNodeVisitCount == 0
            : statistics.accessorNodeVisitCount > 0 &&
              statistics.pointQueryCount ==
                  statistics.accessorRootCacheHitCount +
                  statistics.accessorRootCacheMissCount;

    std::cout << "  Source voxels      : " << sourceSignature.voxelCount << std::endl;
    std::cout << "  Result voxels      : " << resultSignature.voxelCount << std::endl;
    std::cout << "  Modified roots     : " << changes.modifiedRootCount() << std::endl;
    std::cout << "  Input cells        : " << statistics.inputMaterialCellCount << std::endl;
    std::cout << "  Visited cells      : " << statistics.visitedCellCount << std::endl;
    std::cout << "  Classified cells   : " << statistics.classifiedCellCount << std::endl;
    std::cout << "  Outside cells      : " << statistics.outsideCellCount << std::endl;
    std::cout << "  Inside cells       : " << statistics.insideCellCount << std::endl;
    std::cout << "  Intersecting cells : " << statistics.intersectingCellCount << std::endl;
    std::cout << "  Center samples     : " << statistics.centerSampleCount << std::endl;
    std::cout << "  Point queries      : " << statistics.pointQueryCount << std::endl;
    std::cout << "  Query candidates   : " << statistics.queryRootCandidateCount << std::endl;
    std::cout << "  Query roots        : " << statistics.queryExistingRootCount << std::endl;
    std::cout << "  Query node visits  : " << statistics.queryVisitedNodeCount << std::endl;
    std::cout << "  Accessor hits      : " << statistics.accessorRootCacheHitCount << std::endl;
    std::cout << "  Accessor misses    : " << statistics.accessorRootCacheMissCount << std::endl;
    std::cout << "  Accessor visits    : " << statistics.accessorNodeVisitCount << std::endl;

    return check(
        removedMaterial &&
        retainedMaterial &&
        changesRecorded &&
        timingValid &&
        recursionEntered &&
        classificationConsistent &&
        centerQueryConsistent &&
        queryStatisticsConsistent &&
        accessorStatisticsConsistent,
        "Statistics voxel counters");
}

// 测试指定线程数量实际生效且结果一致。
bool testParallelWorkerCounts()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    const std::array<unsigned int, 3> workerCounts = {{1, 2, 4}};
    BooleanOperationTest::OccupancySignature referenceSignature;
    bool hasReference = false;
    bool passed = true;

    for (std::size_t workerIndex = 0; workerIndex < workerCounts.size(); ++workerIndex)
    {
        MyVoxel::Operation::BooleanOperationOptions options;
        options.workerCount = workerCounts[workerIndex];
        options.minimumParallelRootCount = 1;

        MyVoxel::Operation::BooleanOperationStatistics statistics;
        const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, options, statistics);
        const BooleanOperationTest::OccupancySignature signature = BooleanOperationTest::occupancySignature(result);

        if (!hasReference)
        {
            referenceSignature = signature;
            hasReference = true;
        }
        else if (!signature.isEqualTo(referenceSignature))
        {
            passed = false;
        }

        if (statistics.planningWorkerCount != workerCounts[workerIndex])
        {
            passed = false;
        }

        if (statistics.planApplicationWorkerCount == 0 || statistics.planApplicationWorkerCount > workerCounts[workerIndex])
        {
            passed = false;
        }
    }

    return check(passed, "Statistics parallel worker counts");
}

// 测试变更集、写时复制和输入保持不变。
bool testChangeSetAndCopyOnWrite()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const BooleanOperationTest::OccupancySignature sourceSignature = BooleanOperationTest::occupancySignature(workpiece);
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::BooleanOperationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes, statistics);

    const bool passed =
        changes.hasChanges() &&
        changes.modifiedRootCount() > 0 &&
        !result.sharesDataWith(workpiece) &&
        BooleanOperationTest::occupancySignature(workpiece).isEqualTo(sourceSignature) &&
        result.transform().isEqualTo(workpiece.transform());

    return check(passed, "Statistics change set and copy-on-write");
}

// 测试统计累加和重置。
bool testStatisticsAccumulateAndReset()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    MyVoxel::Operation::BooleanOperationStatistics first;
    MyVoxel::Operation::BooleanOperationStatistics second;

    MyVoxel::Operation::cut(workpiece, tool, first);
    MyVoxel::Operation::cut(workpiece, tool, second);

    const std::uint64_t firstVisitedCount = first.visitedCellCount;
    const std::uint64_t secondVisitedCount = second.visitedCellCount;
    const double firstTotalMilliseconds = first.totalMilliseconds;
    const double secondTotalMilliseconds = second.totalMilliseconds;

    first.accumulate(second);

    const bool accumulatePassed =
        first.visitedCellCount == firstVisitedCount + secondVisitedCount &&
        first.totalMilliseconds == firstTotalMilliseconds + secondTotalMilliseconds &&
        first.planningWorkerCount > 0;

    first.reset();

    const bool resetPassed =
        first.totalMilliseconds == 0.0 &&
        first.planningMilliseconds == 0.0 &&
        first.visitedCellCount == 0 &&
        first.classifiedCellCount == 0 &&
        first.cutPlanNodeCount == 0 &&
        first.queryVisitedNodeCount == 0 &&
        first.accessorNodeVisitCount == 0;

    return check(accumulatePassed && resetPassed, "Statistics accumulate and reset");
}

}

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testContinuousResultConsistency(),
        testVoxelResultConsistency(),
        testContinuousStatistics(),
        testVoxelStatistics(),
        testParallelWorkerCounts(),
        testChangeSetAndCopyOnWrite(),
        testStatisticsAccumulateAndReset()
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
    std::cout << "Boolean operation statistics tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}