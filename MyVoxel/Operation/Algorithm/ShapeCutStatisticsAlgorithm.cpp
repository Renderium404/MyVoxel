#include "BooleanAlgorithmConfig.h"

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

#include "BooleanCutAlgorithm.h"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "BooleanAlgorithmCommon.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "ShapeCutContext.h"
#include "ShapeCutPlan.h"
#include "ShapeCutPlanStatistics.h"

namespace
{

using Clock = std::chrono::steady_clock;

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 返回包含两端点的索引范围长度。
std::uint64_t inclusiveRangeLength(MyVoxel::VoxelIndex minimumValue, MyVoxel::VoxelIndex maximumValue)
{
    assert(minimumValue <= maximumValue);
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(maximumValue) - static_cast<std::int64_t>(minimumValue)) + 1;
}

// 返回连续刀具宽相第0层候选根数量。
std::uint64_t rootCandidateCount(const MyVoxel::Operation::Algorithm::ShapeToolContext& context)
{
    const std::uint64_t xCount = inclusiveRangeLength(context.minimumRootIndex.x, context.maximumRootIndex.x);
    const std::uint64_t yCount = inclusiveRangeLength(context.minimumRootIndex.y, context.maximumRootIndex.y);
    const std::uint64_t zCount = inclusiveRangeLength(context.minimumRootIndex.z, context.maximumRootIndex.z);

    return MyVoxel::Operation::Algorithm::saturatedMultiply(MyVoxel::Operation::Algorithm::saturatedMultiply(xCount, yCount), zCount);
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

VoxelShape cutWithShapeToolStatistics(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    statistics.reset();

    if (changes)
    {
        changes->clear();
    }

    const Clock::time_point totalStart = Clock::now();

    assert(object.grid().isValid());
    assert(object.transform().isRigidTransform());
    assert(tool.isValid());
    assert(tool.localToWorld().isRigidTransform());

    const Clock::time_point toolIndexStart = Clock::now();
    const ShapeToolContext context = createShapeToolContext(object, tool);
    const Clock::time_point toolIndexEnd = Clock::now();

    statistics.toolIndexMilliseconds = elapsedMilliseconds(toolIndexStart, toolIndexEnd);
    statistics.broadPhaseRootCandidateCount = rootCandidateCount(context);

    const Clock::time_point materialCollectionStart = Clock::now();
    const std::vector<VoxelCellAddress> rootCells = collectRootCellsInRange(object, context.minimumRootIndex, context.maximumRootIndex);
    const Clock::time_point materialCollectionEnd = Clock::now();

    statistics.materialCollectionMilliseconds = elapsedMilliseconds(materialCollectionStart, materialCollectionEnd);
    statistics.broadPhaseExistingRootCount = static_cast<std::uint64_t>(rootCells.size());
    statistics.inputMaterialCellCount = static_cast<std::uint64_t>(rootCells.size());

    if (rootCells.empty())
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    const unsigned int planningWorkerCount = resolveParallelWorkerCount(options, rootCells.size());
    std::vector<RootCutPlan> plans(rootCells.size());
    std::vector<BooleanOperationStatistics> localStatistics(rootCells.size());

    const Clock::time_point planningStart = Clock::now();

    buildShapeCutPlansStatistics(object, tool, context, rootCells, planningWorkerCount, plans, localStatistics);

    const Clock::time_point planningEnd = Clock::now();

    statistics.planningMilliseconds = elapsedMilliseconds(planningStart, planningEnd);
    statistics.planningWorkerCount = planningWorkerCount;

    for (std::size_t planIndex = 0; planIndex < localStatistics.size(); ++planIndex)
    {
        statistics.accumulate(localStatistics[planIndex]);
    }

    if (!hasShapeCutPlanModification(plans))
    {
        statistics.cuttingMilliseconds = statistics.planningMilliseconds;
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    VoxelShape result = object;

    const Clock::time_point detachStart = Clock::now();
    VoxelForest& resultForest = result.editForest();
    const Clock::time_point detachEnd = Clock::now();

    statistics.detachMilliseconds = elapsedMilliseconds(detachStart, detachEnd);

    std::vector<RootPlanApplication> applications;
    applications.reserve(rootCells.size());

    std::uint64_t removedRootTreeCount = 0;

    const Clock::time_point rootPreparationStart = Clock::now();

    for (std::size_t rootIndex = 0; rootIndex < rootCells.size(); ++rootIndex)
    {
        const VoxelCellAddress& rootAddress = rootCells[rootIndex];
        const CutPlanNode& rootPlan = plans[rootIndex].root;

        if (rootPlan.action == CutPlanAction::Keep)
        {
            continue;
        }

        if (rootPlan.action == CutPlanAction::Remove)
        {
            const bool removed = resultForest.setState(rootAddress, VoxelState::Empty);

            assert(removed);

            if (removed)
            {
                ++removedRootTreeCount;

                if (changes)
                {
                    changes->addModifiedRoot(rootAddress.index);
                }
            }

            continue;
        }

        assert(rootPlan.action == CutPlanAction::Subdivide);

        if (changes)
        {
            changes->addModifiedRoot(rootAddress.index);
        }

        applications.push_back(RootPlanApplication(resultForest, rootAddress, plans[rootIndex]));
    }

    const Clock::time_point rootPreparationEnd = Clock::now();
    const unsigned int applicationWorkerCount = resolveParallelWorkerCount(options, applications.size());
    const Clock::time_point applicationStart = Clock::now();

    applyShapeCutPlans(applications, applicationWorkerCount);

    const Clock::time_point applicationEnd = Clock::now();

    statistics.rootPreparationMilliseconds = elapsedMilliseconds(rootPreparationStart, rootPreparationEnd);
    statistics.planApplicationMilliseconds = elapsedMilliseconds(applicationStart, applicationEnd);
    statistics.planApplicationWorkerCount = applicationWorkerCount;
    statistics.preparedRootTreeCount = static_cast<std::uint64_t>(applications.size());
    statistics.removedRootTreeCount = removedRootTreeCount;
    statistics.cuttingMilliseconds = statistics.planningMilliseconds + statistics.rootPreparationMilliseconds + statistics.planApplicationMilliseconds;
    statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());

    assert(statistics.broadPhaseExistingRootCount == statistics.inputMaterialCellCount);
    assert(statistics.classifiedCellCount == statistics.outsideCellCount + statistics.insideCellCount + statistics.intersectingCellCount);
    assert(statistics.pointQueryCount == statistics.centerSampleCount);
    assert(statistics.queryRootCandidateCount == 0);
    assert(statistics.queryExistingRootCount == 0);
    assert(statistics.queryNodeBoundsTestCount == 0);
    assert(statistics.queryVisitedNodeCount == 0);
    assert(statistics.accessorRootCacheHitCount == 0);
    assert(statistics.accessorRootCacheMissCount == 0);
    assert(statistics.accessorPathReuseCount == 0);
    assert(statistics.accessorReusedPathLevelCount == 0);
    assert(statistics.accessorNodeVisitCount == 0);

    return result;
}

}
}
}

#endif