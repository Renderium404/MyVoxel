#include "BooleanAlgorithmConfig.h"

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

#include "MyVoxel/Operation/BooleanOperation.h"

namespace MyVoxel
{
namespace Operation
{

void BooleanOperationStatistics::reset()
{
    *this = BooleanOperationStatistics();
    alignedMaskCutCount = 0;
    alignedMaskRootCandidateCount = 0;
    alignedMaskIntersectingRootCount = 0;
    alignedMaskOperationCount = 0;
    alignedMaskChangedCount = 0;
}

void BooleanOperationStatistics::accumulate(const BooleanOperationStatistics& other)
{
    totalMilliseconds += other.totalMilliseconds;
    toolIndexMilliseconds += other.toolIndexMilliseconds;
    materialCollectionMilliseconds += other.materialCollectionMilliseconds;
    detachMilliseconds += other.detachMilliseconds;
    planningMilliseconds += other.planningMilliseconds;
    rootPreparationMilliseconds += other.rootPreparationMilliseconds;
    planApplicationMilliseconds += other.planApplicationMilliseconds;
    cuttingMilliseconds += other.cuttingMilliseconds;

    planningWorkerCount += other.planningWorkerCount;
    planApplicationWorkerCount += other.planApplicationWorkerCount;
    cutPlanNodeCount += other.cutPlanNodeCount;
    cutPlanGroupCount += other.cutPlanGroupCount;
    preparedRootTreeCount += other.preparedRootTreeCount;
    removedRootTreeCount += other.removedRootTreeCount;

    broadPhaseRootCandidateCount += other.broadPhaseRootCandidateCount;
    broadPhaseExistingRootCount += other.broadPhaseExistingRootCount;

    toolMaterialCellCount += other.toolMaterialCellCount;
    inputMaterialCellCount += other.inputMaterialCellCount;
    visitedCellCount += other.visitedCellCount;
    emptySkippedCellCount += other.emptySkippedCellCount;
    classifiedCellCount += other.classifiedCellCount;
    outsideCellCount += other.outsideCellCount;
    insideCellCount += other.insideCellCount;
    intersectingCellCount += other.intersectingCellCount;
    centerSampleCount += other.centerSampleCount;
    pointQueryCount += other.pointQueryCount;
    centerRemovedCellCount += other.centerRemovedCellCount;
    removedBranchCount += other.removedBranchCount;
    mergeAttemptCount += other.mergeAttemptCount;
    mergeSuccessCount += other.mergeSuccessCount;

    rootBucketVisitCount += other.rootBucketVisitCount;
    materialBoxTestCount += other.materialBoxTestCount;
    queryRootCandidateCount += other.queryRootCandidateCount;
    queryExistingRootCount += other.queryExistingRootCount;
    queryNodeBoundsTestCount += other.queryNodeBoundsTestCount;
    queryVisitedNodeCount += other.queryVisitedNodeCount;

    accessorRootCacheHitCount += other.accessorRootCacheHitCount;
    accessorRootCacheMissCount += other.accessorRootCacheMissCount;
    accessorPathReuseCount += other.accessorPathReuseCount;
    accessorReusedPathLevelCount += other.accessorReusedPathLevelCount;
    accessorNodeVisitCount += other.accessorNodeVisitCount;

    alignedMaskCutCount += other.alignedMaskCutCount;
    alignedMaskRootCandidateCount += other.alignedMaskRootCandidateCount;
    alignedMaskIntersectingRootCount += other.alignedMaskIntersectingRootCount;
    alignedMaskOperationCount += other.alignedMaskOperationCount;
    alignedMaskChangedCount += other.alignedMaskChangedCount;



}

}
}

#endif