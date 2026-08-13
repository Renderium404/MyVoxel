#include "ShapeCutAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Foundation/Stopwatch.h"
#endif
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const unsigned int MaskLeafCoveredLevelCount = 2; // 默认变化跟踪层级与MaskLeaf入口保持一致，便于后续局部显示更新。

// 保存一个已经通过只读预扫描确认需要修改的最高层TSDF样本。
struct SampleUpdate
{
    SampleUpdate(const MyVoxel::VoxelCellAddress& addressValue, float oldDistanceValue, float newDistanceValue)
        : address(addressValue), oldDistance(oldDistanceValue), newDistance(newDistanceValue)
    {
    }

    MyVoxel::VoxelCellAddress address; // 当前最高采样层级地址。
    float oldDistance; // 当前操作开始前的工件TSDF距离。
    float newDistance; // 标准Difference公式计算后的目标TSDF距离。
};

class DisabledShapeCutRecorder
{
public:
    void broadPhaseCandidate(){}
    void processedRoot(){}
    void changedRoot(){}
    void removedRoot(){}
    void visitSample(){}
    void skipBackground(){}
    void toolSample(double, float){}
    void centerRemoved(){}
    void mergeAttempt(){}
    void mergeSuccess(){}
    void addRootClassificationMilliseconds(double){}
    void addRootPreparationMilliseconds(double){}
    void addExecutionMilliseconds(double){}
    void addRootCpuMilliseconds(double){}
    void addCommitMilliseconds(double){}
};

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

class EnabledShapeCutRecorder
{
public:
    explicit EnabledShapeCutRecorder(MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics) : m_statistics(statistics){}

    void broadPhaseCandidate(){++m_statistics.broadPhaseRootCandidateCount;}
    void processedRoot(){++m_statistics.processedRootCount;}
    void changedRoot(){++m_statistics.changedRootCount;}
    void removedRoot(){++m_statistics.removedRootCount;}
    void visitSample(){++m_statistics.visitedCellCount;}
    void skipBackground(){++m_statistics.emptySkippedCellCount;}
    void toolSample(double distance, float backgroundDistance)
    {
        ++m_statistics.centerSampleCount;
        ++m_statistics.scalarFastClassificationCount;
        ++m_statistics.classifiedCellCount;
        if (distance >= static_cast<double>(backgroundDistance)) ++m_statistics.outsideCellCount;
        else if (distance <= -static_cast<double>(backgroundDistance)) ++m_statistics.insideCellCount;
        else ++m_statistics.intersectingCellCount;
    }
    void centerRemoved(){++m_statistics.centerRemovedCellCount;}
    void mergeAttempt(){++m_statistics.mergeAttemptCount;}
    void mergeSuccess(){++m_statistics.mergeSuccessCount;}
    void addRootClassificationMilliseconds(double value){m_statistics.rootClassificationMilliseconds += value;}
    void addRootPreparationMilliseconds(double value){m_statistics.rootPreparationMilliseconds += value;}
    void addExecutionMilliseconds(double value){m_statistics.intersectingRootExecutionMilliseconds += value;}
    void addRootCpuMilliseconds(double value){m_statistics.intersectingRootCpuMilliseconds += value;}
    void addCommitMilliseconds(double value){m_statistics.commitMilliseconds += value;}

private:
    MyVoxel::Operation::Algorithm::ShapeCutStatistics& m_statistics;
};

using ShapeCutRecorder = EnabledShapeCutRecorder;

#else

using ShapeCutRecorder = DisabledShapeCutRecorder;

#endif

// 返回默认变化跟踪使用的MaskLeaf入口层级。
MyVoxel::VoxelLevel defaultTrackingLevel(MyVoxel::VoxelLevel maximumLevel)
{
    const unsigned int maximum = static_cast<unsigned int>(maximumLevel);
    return maximum >= MaskLeafCoveredLevelCount ? static_cast<MyVoxel::VoxelLevel>(maximum - MaskLeafCoveredLevelCount) : MyVoxel::BaseVoxelLevel;
}

// 将工具signed distance转换为Difference补集距离clamp(-dTool,-B,+B)。
float toolComplementDistance(double toolDistance, float backgroundDistance)
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(toolDistance), "Shape cut signed-distance sample must be finite.");
    const double complement = -toolDistance;
    if (complement >= static_cast<double>(backgroundDistance)) return backgroundDistance;
    if (complement <= -static_cast<double>(backgroundDistance)) return -backgroundDistance;
    const float value = static_cast<float>(complement);
    return (std::max)(-backgroundDistance, (std::min)(backgroundDistance, value));
}

// 返回标准截断SDF差集单样本结果max(dObject,-dTool)。
float subtractDistance(float objectDistance, double toolDistance, float backgroundDistance)
{
    return (std::max)(objectDistance, toolComplementDistance(toolDistance, backgroundDistance));
}

// 使用空会话刷新调用者提供的旧变化记录，同时保持Grid和backgroundDistance语义。
void resetChanges(const MyVoxel::VoxelShape& object, MyVoxel::VoxelChangeSet* changes)
{
    if (!changes) return;
    MyVoxel::VoxelShape emptyResult(object.grid(), object.backgroundDistance());
    MyVoxel::VoxelShapeSession session(emptyResult, changes->trackingLevel());
    *changes = session.takeChanges();
}

// 将会话产生的变化记录交给调用者。
void storeChanges(MyVoxel::VoxelShapeSession& session, MyVoxel::VoxelChangeSet* changes)
{
    if (changes) *changes = session.takeChanges();
}

// 统计工具扩展B范围内实际存在的工件Root；Difference不会访问其它Root。
void recordCandidateRoots(const MyVoxel::VoxelShape& object, const MyVoxel::ShapeQuery& query, ShapeCutRecorder& recorder)
{
    const MyVoxel::Bounds3 bounds = query.queryBounds().expanded(static_cast<double>(object.backgroundDistance()));
    const MyVoxel::VoxelCellRange rootRange = object.grid().cellRange(bounds, MyVoxel::BaseVoxelLevel);
    object.forest().forEachRootCellInRange(rootRange.minimum, rootRange.maximum,
        [&](const MyVoxel::VoxelCellAddress&)
        {
            recorder.broadPhaseCandidate();
        });
}

// 在不修改VoxelShape的前提下逐个最高层采样地址计算标准TSDF Difference结果。
void collectSampleUpdates(const MyVoxel::VoxelShape& object, const MyVoxel::ShapeQuery& query,
                          std::vector<SampleUpdate>& updates, std::set<MyVoxel::VoxelCellIndex>& changedRoots,
                          ShapeCutRecorder& recorder)
{
    updates.clear();
    changedRoots.clear();
    const float backgroundDistance = object.backgroundDistance();
    const MyVoxel::VoxelGrid& grid = object.grid();
    const MyVoxel::VoxelLevel sampleLevel = grid.maximumLevel();
    const MyVoxel::Bounds3 candidateBounds = query.queryBounds().expanded(static_cast<double>(backgroundDistance));
    const MyVoxel::VoxelCellRange sampleRange = grid.cellRange(candidateBounds, sampleLevel);

    for (std::int64_t z = static_cast<std::int64_t>(sampleRange.minimum.z); z <= static_cast<std::int64_t>(sampleRange.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(sampleRange.minimum.y); y <= static_cast<std::int64_t>(sampleRange.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(sampleRange.minimum.x); x <= static_cast<std::int64_t>(sampleRange.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellAddress address(
                    MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y), static_cast<MyVoxel::VoxelIndex>(z)),
                    sampleLevel);
                recorder.visitSample();
                const float oldDistance = object.distance(address);

                // +B已经是截断场最大值，max(+B,anything)严格保持不变；同时覆盖全部缺失Root。
                if (oldDistance == backgroundDistance)
                {
                    recorder.skipBackground();
                    continue;
                }

                const double toolDistance = query.signedDistanceToPoint(grid.cellCenter(address));
                MYVOXEL_ASSERT_MESSAGE(std::isfinite(toolDistance), "Shape cut ShapeQuery must return finite signed distance.");
                recorder.toolSample(toolDistance, backgroundDistance);
                const float newDistance = subtractDistance(oldDistance, toolDistance, backgroundDistance);
                if (newDistance == oldDistance) continue;

                if (oldDistance <= 0.0f && newDistance > 0.0f) recorder.centerRemoved();
                updates.push_back(SampleUpdate(address, oldDistance, newDistance));
                changedRoots.insert(MyVoxel::rootCellAddress(address).index);
            }
        }
    }
}

// 将预扫描得到的确定修改列表写入VoxelShape并按实际变化Root执行Value-aware prune。
bool commitSampleUpdates(MyVoxel::VoxelShape& object, const std::vector<SampleUpdate>& updates,
                         const std::set<MyVoxel::VoxelCellIndex>& changedRoots, MyVoxel::VoxelChangeSet* changes,
                         ShapeCutRecorder& recorder)
{
    if (updates.empty()) return false;

    const MyVoxel::VoxelLevel trackingLevel = changes ? changes->trackingLevel() : defaultTrackingLevel(object.grid().maximumLevel());
    MyVoxel::VoxelShapeSession session(object, trackingLevel);

    for (std::size_t updateIndex = 0; updateIndex < updates.size(); ++updateIndex)
    {
        const SampleUpdate& update = updates[updateIndex];
        const bool changed = session.setDistance(update.address, update.newDistance);

        // 前序写入可能使当前Leaf无损折叠并提前得到同一目标Value，因此这里只要求最终Value一致。
        MYVOXEL_ASSERT_MESSAGE(changed || session.distance(update.address) == update.newDistance,
                               "Shape cut sample commit must reach the precomputed target distance.");
    }

    for (std::set<MyVoxel::VoxelCellIndex>::const_iterator iterator = changedRoots.begin(); iterator != changedRoots.end(); ++iterator)
    {
        recorder.mergeAttempt();
        const bool hadTreeBeforePrune = session.tree(*iterator) != nullptr;
        const bool pruned = session.pruneTree(*iterator);
        if (pruned) recorder.mergeSuccess();
        const bool hasTreeAfterPrune = session.tree(*iterator) != nullptr;
        if (hadTreeBeforePrune && !hasTreeAfterPrune) recorder.removedRoot();
        recorder.processedRoot();
        recorder.changedRoot();
    }

    storeChanges(session, changes);
    return true;
}

// 执行当前Instance::Shape连续体直接TSDF差集。
bool applyShapeCut(MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool,
                   MyVoxel::VoxelChangeSet* changes, ShapeCutRecorder& recorder)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape cut requires a valid Instance::Shape.");

    resetChanges(object, changes);
    if (object.isEmpty()) return false;

    const MyVoxel::ShapeQuery query(tool, object.transform());
    MYVOXEL_REQUIRE_MESSAGE(query.isValid(), "Shape cut requires a valid ShapeQuery.");
    MYVOXEL_REQUIRE_MESSAGE(query.supportsSignedDistance(),
                            "TSDF Shape cut requires exact signed-distance support in object voxel space.");

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Foundation::Stopwatch classificationTimer;
#endif

    recordCandidateRoots(object, query, recorder);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addRootClassificationMilliseconds(classificationTimer.elapsedMilliseconds());
    MyVoxel::Foundation::Stopwatch executionTimer;
#endif

    // 正确性基线先只读计算全部目标Value；此阶段绝不修改object，因此异常和no-op都保持原共享状态。
    std::vector<SampleUpdate> updates;
    std::set<MyVoxel::VoxelCellIndex> changedRoots;
    collectSampleUpdates(object, query, updates, changedRoots, recorder);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    const double executionMilliseconds = executionTimer.elapsedMilliseconds();
    recorder.addExecutionMilliseconds(executionMilliseconds);
    recorder.addRootCpuMilliseconds(executionMilliseconds);
#endif

    if (updates.empty()) return false;

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Foundation::Stopwatch commitTimer;
#endif

    const bool changed = commitSampleUpdates(object, updates, changedRoots, changes, recorder);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addCommitMilliseconds(commitTimer.elapsedMilliseconds());
#endif

    MYVOXEL_ASSERT_MESSAGE(changed, "A non-empty Shape cut update list must modify the VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(object.isValid(), "Shape cut result VoxelShape must remain valid.");
    return true;
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

ShapeCutStatistics::ShapeCutStatistics(){reset();}

void ShapeCutStatistics::reset()
{
    broadPhaseRootCandidateCount = 0; processedRootCount = 0; changedRootCount = 0; removedRootCount = 0;
    rootBoundsConstructionCount = 0; scalarFastClassificationCount = 0; octantBatchClassificationCount = 0; derivedChildBoundsCount = 0;
    visitedCellCount = 0; emptySkippedCellCount = 0; classifiedCellCount = 0; outsideCellCount = 0; insideCellCount = 0; intersectingCellCount = 0;
    centerSampleCount = 0; centerRemovedCellCount = 0; removedBranchCount = 0;
    splitCount = 0; maskLeafBuildCount = 0; maskLeafOperationCount = 0; maskLeafRemovedCellCount = 0;
    mergeAttemptCount = 0; mergeSuccessCount = 0;
    rootClassificationMilliseconds = 0.0; rootPreparationMilliseconds = 0.0; intersectingRootExecutionMilliseconds = 0.0;
    intersectingRootCpuMilliseconds = 0.0; commitMilliseconds = 0.0;
}

void ShapeCutStatistics::accumulate(const ShapeCutStatistics& other)
{
    broadPhaseRootCandidateCount += other.broadPhaseRootCandidateCount; processedRootCount += other.processedRootCount;
    changedRootCount += other.changedRootCount; removedRootCount += other.removedRootCount;
    rootBoundsConstructionCount += other.rootBoundsConstructionCount; scalarFastClassificationCount += other.scalarFastClassificationCount;
    octantBatchClassificationCount += other.octantBatchClassificationCount; derivedChildBoundsCount += other.derivedChildBoundsCount;
    visitedCellCount += other.visitedCellCount; emptySkippedCellCount += other.emptySkippedCellCount; classifiedCellCount += other.classifiedCellCount;
    outsideCellCount += other.outsideCellCount; insideCellCount += other.insideCellCount; intersectingCellCount += other.intersectingCellCount;
    centerSampleCount += other.centerSampleCount; centerRemovedCellCount += other.centerRemovedCellCount; removedBranchCount += other.removedBranchCount;
    splitCount += other.splitCount; maskLeafBuildCount += other.maskLeafBuildCount; maskLeafOperationCount += other.maskLeafOperationCount;
    maskLeafRemovedCellCount += other.maskLeafRemovedCellCount; mergeAttemptCount += other.mergeAttemptCount; mergeSuccessCount += other.mergeSuccessCount;
    rootClassificationMilliseconds += other.rootClassificationMilliseconds; rootPreparationMilliseconds += other.rootPreparationMilliseconds;
    intersectingRootExecutionMilliseconds += other.intersectingRootExecutionMilliseconds;
    intersectingRootCpuMilliseconds += other.intersectingRootCpuMilliseconds; commitMilliseconds += other.commitMilliseconds;
}

#endif

bool ShapeCutAlgorithm::apply(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    ShapeCutStatistics ignoredStatistics;
    return apply(object, tool, changes, ignoredStatistics);
#else
    ShapeCutRecorder recorder;
    return applyShapeCut(object, tool, changes, recorder);
#endif
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

bool ShapeCutAlgorithm::apply(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes, ShapeCutStatistics& statistics)
{
    statistics.reset();
    ShapeCutRecorder recorder(statistics);
    return applyShapeCut(object, tool, changes, recorder);
}

#endif

}
}
}