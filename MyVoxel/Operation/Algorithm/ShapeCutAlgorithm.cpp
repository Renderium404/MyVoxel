#include "ShapeCutAlgorithm.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"
#include "MyVoxel/Foundation/Stopwatch.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"
#include "MyVoxel/Geometry/Shape/ShapeRelation.h"

namespace
{

const unsigned int MaskLeafCoveredLevelCount = 2; // 一个掩码叶块覆盖当前逻辑体素下面两级。
const double CellCenterScale = 0.5; // 体素中心和半尺寸递归二分使用的固定比例。
const unsigned int XChildMask = 1; // VoxelCorner第0位控制子包围盒的X方向。
const unsigned int YChildMask = 2; // VoxelCorner第1位控制子包围盒的Y方向。
const unsigned int ZChildMask = 4; // VoxelCorner第2位控制子包围盒的Z方向。
const unsigned int MinimumOctantBatchActiveChildCount = 4; // 至少四个活动子体素时批量分类全部八个子包围盒。
const std::size_t MinimumParallelIntersectingRootCount = 4; // 四个及以上相交根进入并行切削。

static_assert(
    static_cast<unsigned int>(MyVoxel::ShapeQuery::OctantCount) ==
    static_cast<unsigned int>(MyVoxel::VoxelCornerCount),
    "ShapeQuery octant order must match VoxelCorner order.");

// 保存体素在查询空间中的中心和半尺寸，递归过程不再重复构造Bounds3。
struct CellBounds
{
    CellBounds(const MyMath::Vector3& centerValue, const MyMath::Vector3& extentValue)
        : center(centerValue)
        , extent(extentValue)
    {
        MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "CellBounds center must be finite.");
        MYVOXEL_ASSERT_MESSAGE(
            extent.isFinite() &&
            extent.x() >= 0.0 &&
            extent.y() >= 0.0 &&
            extent.z() >= 0.0,
            "CellBounds extent must be finite and non-negative.");
    }

    MyMath::Vector3 center; // 当前体素中心。
    MyMath::Vector3 extent; // 当前体素三个方向的半尺寸。
};

struct CutCellResult
{
    CutCellResult(MyVoxel::VoxelState stateValue, bool changedValue)
        : state(stateValue)
        , changed(changedValue)
    {
    }

    MyVoxel::VoxelState state;
    bool changed;
};

class DisabledShapeCutRecorder
{
public:
    void broadPhaseCandidate() {}
    void processedRoot() {}
    void changedRoot() {}
    void removedRoot() {}
    void rootBoundsConstruction() {}
    void scalarFastClassification() {}
    void octantBatchClassification() {}
    void derivedChildBounds() {}
    void visitCell() {}
    void skipEmpty() {}
    void classify(MyVoxel::ShapeRelation) {}
    void centerSample() {}
    void centerRemoved() {}
    void removedBranch() {}
    void split() {}
    void buildMaskLeaf() {}
    void maskLeafOperation(std::uint64_t) {}
    void mergeAttempt() {}
    void mergeSuccess() {}
    void addRootClassificationMilliseconds(double) {}
    void addRootPreparationMilliseconds(double) {}
    void addIntersectingRootExecutionMilliseconds(double) {}
    void addCommitMilliseconds(double) {}
};

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

class EnabledShapeCutRecorder
{
public:
    explicit EnabledShapeCutRecorder(MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
        : m_statistics(statistics)
    {
    }

    void broadPhaseCandidate()
    {
        ++m_statistics.broadPhaseRootCandidateCount;
    }

    void processedRoot()
    {
        ++m_statistics.processedRootCount;
    }

    void changedRoot()
    {
        ++m_statistics.changedRootCount;
    }

    void removedRoot()
    {
        ++m_statistics.removedRootCount;
    }

    void rootBoundsConstruction()
    {
        ++m_statistics.rootBoundsConstructionCount;
    }

    void scalarFastClassification()
    {
        ++m_statistics.scalarFastClassificationCount;
    }

    void octantBatchClassification()
    {
        ++m_statistics.octantBatchClassificationCount;
    }

    void derivedChildBounds()
    {
        ++m_statistics.derivedChildBoundsCount;
    }

    void visitCell()
    {
        ++m_statistics.visitedCellCount;
    }

    void skipEmpty()
    {
        ++m_statistics.emptySkippedCellCount;
    }

    void classify(MyVoxel::ShapeRelation relation)
    {
        ++m_statistics.classifiedCellCount;

        switch (relation)
        {
        case MyVoxel::ShapeRelation::Outside:
            ++m_statistics.outsideCellCount;
            return;

        case MyVoxel::ShapeRelation::Inside:
            ++m_statistics.insideCellCount;
            return;

        case MyVoxel::ShapeRelation::Intersecting:
            ++m_statistics.intersectingCellCount;
            return;
        }

        MYVOXEL_ASSERT_MESSAGE(false, "Unknown ShapeRelation.");
    }

    void centerSample()
    {
        ++m_statistics.centerSampleCount;
    }

    void centerRemoved()
    {
        ++m_statistics.centerRemovedCellCount;
    }

    void removedBranch()
    {
        ++m_statistics.removedBranchCount;
    }

    void split()
    {
        ++m_statistics.splitCount;
    }

    void buildMaskLeaf()
    {
        ++m_statistics.maskLeafBuildCount;
    }

    void maskLeafOperation(std::uint64_t removedMask)
    {
        ++m_statistics.maskLeafOperationCount;
        m_statistics.maskLeafRemovedCellCount += MyVoxel::leafMaskBitCount(removedMask);
    }

    void mergeAttempt()
    {
        ++m_statistics.mergeAttemptCount;
    }

    void mergeSuccess()
    {
        ++m_statistics.mergeSuccessCount;
    }

    void addRootClassificationMilliseconds(double milliseconds)
    {
        m_statistics.rootClassificationMilliseconds += milliseconds;
    }

    void addRootPreparationMilliseconds(double milliseconds)
    {
        m_statistics.rootPreparationMilliseconds += milliseconds;
    }

    void addIntersectingRootExecutionMilliseconds(double milliseconds)
    {
        m_statistics.intersectingRootExecutionMilliseconds += milliseconds;
    }

    void addCommitMilliseconds(double milliseconds)
    {
        m_statistics.commitMilliseconds += milliseconds;
    }

    void accumulate(const MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
    {
        m_statistics.accumulate(statistics);
    }

private:
    MyVoxel::Operation::Algorithm::ShapeCutStatistics& m_statistics;
};

using ShapeCutRecorder = EnabledShapeCutRecorder;

#else

using ShapeCutRecorder = DisabledShapeCutRecorder;

#endif

// 保存一个已经完成根级快速分类的边界相交根。
struct IntersectingRootPreparation
{
    IntersectingRootPreparation(const MyVoxel::VoxelCellIndex& rootIndexValue,
                                const CellBounds& rootBoundsValue)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
    {
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前相交根的第0层索引。
    CellBounds rootBounds; // 当前根的中心和半尺寸。
};

// 保存一个相交根的独立计算结果和提交状态。
struct RootCutItem
{
    RootCutItem(
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        const CellBounds& rootBoundsValue,
        const MyVoxel::VoxelTree& sourceTree,
        const MyVoxel::VoxelChangeSet::DirtyCellRegion& dirtyRegionValue)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
        , resultTree(sourceTree)
        , dirtyRegion(dirtyRegionValue)
        , changed(false)
        , removed(false)
    {
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前相交根的第0层索引。
    CellBounds rootBounds; // 当前根的中心和半尺寸。
    MyVoxel::VoxelTree resultTree; // 当前任务独立计算的根树结果。
    MyVoxel::VoxelChangeSet::DirtyCellRegion dirtyRegion; // 当前根实际发生材料变化的固定层级脏区。
    bool changed; // 当前根是否发生实际材料变化。
    bool removed; // 当前根结果是否完全为空并应从森林删除。

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Operation::Algorithm::ShapeCutStatistics statistics; // 当前相交根独立产生的递归切削统计。
#endif
};

// 将普通轴对齐包围盒转换为中心和半尺寸表示。
CellBounds makeCellBounds(const MyVoxel::Bounds3& bounds)
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Cannot create CellBounds from invalid Bounds3.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    return CellBounds(
        MyMath::Vector3(
            (minimum.x() + maximum.x()) * CellCenterScale,
            (minimum.y() + maximum.y()) * CellCenterScale,
            (minimum.z() + maximum.z()) * CellCenterScale),
        MyMath::Vector3(
            (maximum.x() - minimum.x()) * CellCenterScale,
            (maximum.y() - minimum.y()) * CellCenterScale,
            (maximum.z() - minimum.z()) * CellCenterScale));
}

// 根据父中心、子半尺寸和角点方向生成子体素中心和半尺寸。
CellBounds makeChildBounds(const MyMath::Vector3& parentCenter,
                           const MyMath::Vector3& childExtent,
                           MyVoxel::VoxelCorner corner)
{
    const unsigned int cornerValue = static_cast<unsigned int>(corner);

    MYVOXEL_ASSERT_MESSAGE(
        cornerValue < static_cast<unsigned int>(MyVoxel::VoxelCornerCount),
        "Voxel child corner must be in range [0, 7].");

    return CellBounds(
        MyMath::Vector3(
            parentCenter.x() + ((cornerValue & XChildMask) != 0 ? childExtent.x() : -childExtent.x()),
            parentCenter.y() + ((cornerValue & YChildMask) != 0 ? childExtent.y() : -childExtent.y()),
            parentCenter.z() + ((cornerValue & ZChildMask) != 0 ? childExtent.z() : -childExtent.z())),
        childExtent);
}

// 返回用于增量表面更新统计的掩码叶块层级。
MyVoxel::VoxelLevel dirtyLeafBlockLevel(MyVoxel::VoxelLevel maximumLevel)
{
    const unsigned int maximumLevelValue =
        static_cast<unsigned int>(maximumLevel);

    return maximumLevelValue >= MaskLeafCoveredLevelCount
        ? static_cast<MyVoxel::VoxelLevel>(
            maximumLevelValue - MaskLeafCoveredLevelCount)
        : MyVoxel::BaseVoxelLevel;
}

// 判断当前逻辑体素是否恰好剩余两层。
bool hasTwoRemainingLevels(const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel maximumLevel)
{
    const unsigned int currentLevel = static_cast<unsigned int>(address.level);
    const unsigned int finalLevel = static_cast<unsigned int>(maximumLevel);

    return currentLevel <= finalLevel && finalLevel - currentLevel == MaskLeafCoveredLevelCount;
}

// 使用中心和半尺寸快速分类一个体素包围盒并记录统计。
MyVoxel::ShapeRelation classifyCellBoundsFast(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& bounds,
    ShapeCutRecorder& recorder)
{
    recorder.scalarFastClassification();

    const MyVoxel::ShapeRelation relation =
        query.classifyBoundsFast(bounds.center, bounds.extent);

    recorder.classify(relation);
    return relation;
}

// 批量分类当前体素的八个等尺寸子体素，并返回子体素半尺寸。
MyMath::Vector3 classifyAllChildBoundsFast(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& parentBounds,
    std::array<
        MyVoxel::ShapeRelation,
        MyVoxel::ShapeQuery::OctantCount>& relations,
    ShapeCutRecorder& recorder)
{
    const MyMath::Vector3 childExtent =
        parentBounds.extent * CellCenterScale;

    query.classifyOctantBoundsFast(
        parentBounds.center,
        childExtent,
        relations);

    recorder.octantBatchClassification();

    for (unsigned int cornerIndex = 0;
         cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++cornerIndex)
    {
        recorder.classify(relations[cornerIndex]);
    }

    return childExtent;
}

// 根据活动子体素数量选择批量或逐个快速分类，并返回子体素半尺寸。
MyMath::Vector3 classifyActiveChildBoundsFast(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& parentBounds,
    std::uint8_t activeMask,
    std::array<
        MyVoxel::ShapeRelation,
        MyVoxel::ShapeQuery::OctantCount>& relations,
    ShapeCutRecorder& recorder)
{
    MYVOXEL_ASSERT_MESSAGE(
        activeMask != MyVoxel::EmptyVoxelNodeMask,
        "Active child classification requires at least one active child.");

    if (MyVoxel::nodeMaskBitCount(activeMask) >=
        MinimumOctantBatchActiveChildCount)
    {
        return classifyAllChildBoundsFast(
            query,
            parentBounds,
            relations,
            recorder);
    }

    relations.fill(MyVoxel::ShapeRelation::Outside);

    const MyMath::Vector3 childExtent =
        parentBounds.extent * CellCenterScale;
    std::uint8_t remainingMask = activeMask;

    while (remainingMask != MyVoxel::EmptyVoxelNodeMask)
    {
        const MyVoxel::VoxelCorner corner =
            MyVoxel::takeFirstNodeCorner(remainingMask);
        const CellBounds childBounds =
            makeChildBounds(
                parentBounds.center,
                childExtent,
                corner);

        recorder.derivedChildBounds();

        relations[static_cast<unsigned int>(corner)] =
            classifyCellBoundsFast(
                query,
                childBounds,
                recorder);
    }

    return childExtent;
}

// 对最高层相交体素执行中心采样。
bool toolContainsCellCenter(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& bounds,
    ShapeCutRecorder& recorder)
{
    recorder.centerSample();
    return query.containsPoint(bounds.center);
}

// 返回最高层体素是否被连续几何体覆盖。
bool toolContainsMaximumCell(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& bounds,
    MyVoxel::ShapeRelation relation,
    ShapeCutRecorder& recorder)
{
    switch (relation)
    {
    case MyVoxel::ShapeRelation::Outside:
        return false;

    case MyVoxel::ShapeRelation::Inside:
        return true;

    case MyVoxel::ShapeRelation::Intersecting:
        return toolContainsCellCenter(
            query,
            bounds,
            recorder);
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Unknown ShapeRelation.");
    return false;
}

// 生成当前逻辑体素下面两级对应的64位工具材料掩码。
std::uint64_t buildToolLeafMask(
    const MyVoxel::ShapeQuery& query,
    const CellBounds& cellBounds,
    const MyVoxel::VoxelCellAddress& address,
    MyVoxel::VoxelLevel maximumLevel,
    ShapeCutRecorder& recorder)
{
    MYVOXEL_ASSERT_MESSAGE(
        hasTwoRemainingLevels(address, maximumLevel),
        "Tool leaf mask requires exactly two remaining voxel levels.");

    recorder.buildMaskLeaf();

    std::uint64_t toolMask = MyVoxel::EmptyVoxelLeafMask;

    std::array<
        MyVoxel::ShapeRelation,
        MyVoxel::ShapeQuery::OctantCount> coarseRelations;

    const MyMath::Vector3 coarseExtent =
        classifyAllChildBoundsFast(
            query,
            cellBounds,
            coarseRelations,
            recorder);

    for (unsigned int coarseIndex = 0;
         coarseIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++coarseIndex)
    {
        const MyVoxel::VoxelCorner coarseCorner =
            static_cast<MyVoxel::VoxelCorner>(coarseIndex);
        const MyVoxel::ShapeRelation coarseRelation =
            coarseRelations[coarseIndex];

        if (coarseRelation == MyVoxel::ShapeRelation::Outside)
        {
            continue;
        }

        if (coarseRelation == MyVoxel::ShapeRelation::Inside)
        {
            toolMask |= MyVoxel::leafGroupMask(coarseCorner);
            continue;
        }

        const CellBounds coarseBounds =
            makeChildBounds(
                cellBounds.center,
                coarseExtent,
                coarseCorner);

        recorder.derivedChildBounds();

        std::array<
            MyVoxel::ShapeRelation,
            MyVoxel::ShapeQuery::OctantCount> fineRelations;

        const MyMath::Vector3 fineExtent =
            classifyAllChildBoundsFast(
                query,
                coarseBounds,
                fineRelations,
                recorder);

        std::uint8_t groupMask = 0;

        for (unsigned int fineIndex = 0;
             fineIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
             ++fineIndex)
        {
            const MyVoxel::VoxelCorner fineCorner =
                static_cast<MyVoxel::VoxelCorner>(fineIndex);
            const MyVoxel::ShapeRelation fineRelation =
                fineRelations[fineIndex];

            if (fineRelation == MyVoxel::ShapeRelation::Outside)
            {
                continue;
            }

            if (fineRelation == MyVoxel::ShapeRelation::Inside)
            {
                groupMask = static_cast<std::uint8_t>(
                    groupMask |
                    MyVoxel::nodeCornerMask(fineCorner));
                continue;
            }

            const CellBounds fineBounds =
                makeChildBounds(
                    coarseBounds.center,
                    fineExtent,
                    fineCorner);

            recorder.derivedChildBounds();

            if (toolContainsMaximumCell(
                    query,
                    fineBounds,
                    fineRelation,
                    recorder))
            {
                groupMask = static_cast<std::uint8_t>(
                    groupMask |
                    MyVoxel::nodeCornerMask(fineCorner));
            }
        }

        toolMask |=
            static_cast<std::uint64_t>(groupMask) <<
            MyVoxel::leafMaskOffset(coarseCorner);
    }

    return toolMask;
}

// 将当前逻辑体素转换为覆盖下面两级的64位工件材料掩码。
std::uint64_t currentObjectLeafMask(MyVoxel::VoxelTreeEditor& editor)
{
    if (editor.isEmpty())
    {
        return MyVoxel::EmptyVoxelLeafMask;
    }

    if (editor.isMaterial())
    {
        return MyVoxel::FullVoxelLeafMask;
    }

    const MyVoxel::VoxelChildStateMasks coarseStates = editor.childStateMasks();
    std::uint64_t materialMask = MyVoxel::EmptyVoxelLeafMask;

    for (unsigned int coarseIndex = 0; coarseIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++coarseIndex)
    {
        const MyVoxel::VoxelCorner coarseCorner = static_cast<MyVoxel::VoxelCorner>(coarseIndex);
        const std::uint8_t coarseBit = MyVoxel::nodeCornerMask(coarseCorner);

        if ((coarseStates.material & coarseBit) != 0)
        {
            materialMask |= MyVoxel::leafGroupMask(coarseCorner);
            continue;
        }

        if ((coarseStates.subdivided & coarseBit) == 0)
        {
            continue;
        }

        MyVoxel::VoxelTreeEditor coarseEditor = editor.child(coarseCorner);
        const MyVoxel::VoxelChildStateMasks fineStates = coarseEditor.childStateMasks();

        MYVOXEL_ASSERT_MESSAGE(fineStates.subdivided == MyVoxel::EmptyVoxelNodeMask,
                               "A voxel with one remaining level cannot contain subdivided children.");

        materialMask |= static_cast<std::uint64_t>(fineStates.material) << MyVoxel::leafMaskOffset(coarseCorner);
    }

    return materialMask;
}

// 尝试将当前细分体素折叠为空或材料。
MyVoxel::VoxelState collapseCell(MyVoxel::VoxelTreeEditor& editor, ShapeCutRecorder& recorder)
{
    if (!editor.isSubdivided())
    {
        return editor.state();
    }

    recorder.mergeAttempt();

    const MyVoxel::VoxelChildStateMasks states = editor.childStateMasks();

    if (states.empty == MyVoxel::FullVoxelNodeMask)
    {
        editor.setEmpty();
        recorder.mergeSuccess();
        return MyVoxel::VoxelState::Empty;
    }

    if (states.material == MyVoxel::FullVoxelNodeMask)
    {
        editor.setMaterial();
        recorder.mergeSuccess();
        return MyVoxel::VoxelState::Material;
    }

    return MyVoxel::VoxelState::Subdivided;
}

// 在调用者要求变化结果时，将当前材料变化记录到Root独立脏区。
void recordMaterialChange(
    const MyVoxel::VoxelShapeSession& session,
    MyVoxel::VoxelChangeSet::DirtyCellRegion* dirtyRegion,
    const MyVoxel::VoxelCellAddress& address)
{
    if (!dirtyRegion)
    {
        return;
    }

    session.recordMaterialChange(
        *dirtyRegion,
        address);
}

// 递归从当前工件逻辑体素中减去连续几何体。
CutCellResult cutCell(
    MyVoxel::VoxelTreeEditor editor,
    const MyVoxel::ShapeQuery& query,
    const CellBounds& cellBounds,
    const MyVoxel::VoxelCellAddress& address,
    MyVoxel::VoxelLevel maximumLevel,
    MyVoxel::ShapeRelation relation,
    const MyVoxel::VoxelShapeSession& session,
    MyVoxel::VoxelChangeSet::DirtyCellRegion* dirtyRegion,
    ShapeCutRecorder& recorder)
{
    recorder.visitCell();

    if (editor.isEmpty())
    {
        recorder.skipEmpty();
        return CutCellResult(MyVoxel::VoxelState::Empty, false);
    }

    if (relation == MyVoxel::ShapeRelation::Outside)
    {
        return CutCellResult(editor.state(), false);
    }

    if (relation == MyVoxel::ShapeRelation::Inside)
    {
        if (editor.isSubdivided())
        {
            recorder.removedBranch();
        }

        recordMaterialChange(
            session,
            dirtyRegion,
            address);
        editor.setEmpty();
        return CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    MYVOXEL_ASSERT_MESSAGE(
        relation == MyVoxel::ShapeRelation::Intersecting,
        "Shape cut requires a valid ShapeRelation.");

    if (address.level == maximumLevel)
    {
        if (!toolContainsCellCenter(
                query,
                cellBounds,
                recorder))
        {
            return CutCellResult(editor.state(), false);
        }

        recordMaterialChange(
            session,
            dirtyRegion,
            address);
        editor.setEmpty();
        recorder.centerRemoved();
        return CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    MYVOXEL_ASSERT_MESSAGE(
        address.level < maximumLevel,
        "Shape cut address exceeds the VoxelGrid maximum level.");

    if (hasTwoRemainingLevels(address, maximumLevel) &&
        editor.canSetMaterialMask())
    {
        const std::uint64_t objectMask =
            currentObjectLeafMask(editor);
        const std::uint64_t toolMask =
            buildToolLeafMask(
                query,
                cellBounds,
                address,
                maximumLevel,
                recorder);
        const std::uint64_t removedMask =
            objectMask & toolMask;

        if (removedMask == MyVoxel::EmptyVoxelLeafMask)
        {
            return CutCellResult(editor.state(), false);
        }

        const std::uint64_t resultMask =
            objectMask & ~toolMask;

        recordMaterialChange(
            session,
            dirtyRegion,
            address);
        recorder.maskLeafOperation(removedMask);
        MYVOXEL_ASSERT_MESSAGE(
            editor.canSetMaterialMask(),
            "Shape cut MaskLeaf path requires a material-mask editor.");
        editor.setMaterialMask(resultMask);
        return CutCellResult(
            MyVoxel::leafMaskState(resultMask),
            true);
    }

    const MyVoxel::VoxelChildStateMasks childStates =
        editor.childStateMasks();
    const std::uint8_t activeMask =
        static_cast<std::uint8_t>(
            childStates.material |
            childStates.subdivided);

    MYVOXEL_ASSERT_MESSAGE(
        activeMask != MyVoxel::EmptyVoxelNodeMask,
        "A non-empty cut cell must contain at least one active child.");

    std::array<
        MyVoxel::ShapeRelation,
        MyVoxel::ShapeQuery::OctantCount> childRelations;

    const MyMath::Vector3 childExtent =
        classifyActiveChildBoundsFast(
            query,
            cellBounds,
            activeMask,
            childRelations,
            recorder);

    std::uint8_t candidateMask = activeMask;
    std::uint8_t remainingMask = activeMask;

    while (remainingMask != MyVoxel::EmptyVoxelNodeMask)
    {
        const MyVoxel::VoxelCorner corner =
            MyVoxel::takeFirstNodeCorner(remainingMask);

        if (childRelations[static_cast<unsigned int>(corner)] ==
            MyVoxel::ShapeRelation::Outside)
        {
            candidateMask = static_cast<std::uint8_t>(
                candidateMask &
                static_cast<std::uint8_t>(
                    ~MyVoxel::nodeCornerMask(corner)));
        }
    }

    if (candidateMask == MyVoxel::EmptyVoxelNodeMask)
    {
        return CutCellResult(editor.state(), false);
    }

    if (!editor.isSubdivided())
    {
        editor.subdivide();
        recorder.split();
    }

    bool changed = false;
    remainingMask = candidateMask;

    while (remainingMask != MyVoxel::EmptyVoxelNodeMask)
    {
        const MyVoxel::VoxelCorner corner =
            MyVoxel::takeFirstNodeCorner(remainingMask);
        const MyVoxel::VoxelCellAddress childAddress =
            MyVoxel::childCellAddress(address, corner);
        const CellBounds childBounds =
            makeChildBounds(
                cellBounds.center,
                childExtent,
                corner);

        recorder.derivedChildBounds();

        MyVoxel::VoxelTreeEditor childEditor =
            editor.child(corner);
        const CutCellResult childResult =
            cutCell(
                childEditor,
                query,
                childBounds,
                childAddress,
                maximumLevel,
                childRelations[static_cast<unsigned int>(corner)],
                session,
                dirtyRegion,
                recorder);

        changed = childResult.changed || changed;
    }

    if (!changed)
    {
        return CutCellResult(editor.state(), false);
    }

    return CutCellResult(
        collapseCell(editor, recorder),
        true);
}

// 串行分类几何包围盒范围内的已有根。
void classifyCandidateRoots(
    const MyVoxel::VoxelShape& object,
    const MyVoxel::ShapeQuery& query,
    const MyVoxel::VoxelGrid& grid,
    std::vector<MyVoxel::VoxelCellIndex>& removedRootIndices,
    std::vector<IntersectingRootPreparation>& intersectingRoots,
    ShapeCutRecorder& recorder)
{
    removedRootIndices.clear();
    intersectingRoots.clear();

    const MyVoxel::VoxelCellRange rootRange =
        grid.cellRange(
            query.queryBounds(),
            MyVoxel::BaseVoxelLevel);

    object.forest().forEachRootCellInRange(
        rootRange.minimum,
        rootRange.maximum,
        [&](const MyVoxel::VoxelCellAddress& rootAddress)
        {
            recorder.broadPhaseCandidate();

            const CellBounds rootBounds =
                makeCellBounds(grid.cellBounds(rootAddress));

            recorder.rootBoundsConstruction();

            const MyVoxel::ShapeRelation relation =
                classifyCellBoundsFast(
                    query,
                    rootBounds,
                    recorder);

            recorder.processedRoot();

            switch (relation)
            {
            case MyVoxel::ShapeRelation::Outside:
                return;

            case MyVoxel::ShapeRelation::Inside:
                removedRootIndices.push_back(rootAddress.index);
                return;

            case MyVoxel::ShapeRelation::Intersecting:
                intersectingRoots.push_back(
                    IntersectingRootPreparation(
                        rootAddress.index,
                        rootBounds));
                return;
            }

            MYVOXEL_ASSERT_MESSAGE(
                false,
                "Unknown root ShapeRelation.");
        });
}

// 为每个相交根建立独立结果树。
void prepareRootCutItems(
    const MyVoxel::VoxelShapeSession& session,
    const std::vector<IntersectingRootPreparation>& roots,
    std::vector<RootCutItem>& items)
{
    items.clear();
    items.reserve(roots.size());

    for (std::size_t rootPosition = 0;
         rootPosition < roots.size();
         ++rootPosition)
    {
        const IntersectingRootPreparation& root =
            roots[rootPosition];
        const MyVoxel::VoxelTree* sourceTree =
            session.tree(root.rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            sourceTree,
            "A classified intersecting shape cut root must exist in the session.");

        items.push_back(
            RootCutItem(
                root.rootIndex,
                root.rootBounds,
                *sourceTree,
                session.createDirtyCellRegion(
                    root.rootIndex)));
    }
}

// 在独立根树副本上执行一次相交根递归切削。
void processIntersectingRootCut(
    RootCutItem& item,
    const MyVoxel::ShapeQuery& query,
    MyVoxel::VoxelLevel maximumLevel,
    const MyVoxel::VoxelShapeSession& session,
    bool trackMaterialChanges)
{
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    ShapeCutRecorder recorder(item.statistics);
    MyVoxel::Foundation::Stopwatch executionTimer;
#else
    ShapeCutRecorder recorder;
#endif

    const MyVoxel::VoxelCellAddress rootAddress(
        item.rootIndex,
        MyVoxel::BaseVoxelLevel);
    MyVoxel::VoxelTreeEditor editor =
        item.resultTree.editor();

    const CutCellResult rootResult =
        cutCell(
            editor,
            query,
            item.rootBounds,
            rootAddress,
            maximumLevel,
            MyVoxel::ShapeRelation::Intersecting,
            session,
            trackMaterialChanges
                ? &item.dirtyRegion
                : nullptr,
            recorder);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    item.statistics.intersectingRootCpuMilliseconds +=
        executionTimer.elapsedMilliseconds();
#endif

    if (!rootResult.changed)
    {
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(
        !trackMaterialChanges ||
            item.dirtyRegion.changedCellCount() > 0,
        "A changed shape cut Root must contain a material dirty region.");

    item.changed = true;
    item.removed =
        rootResult.state == MyVoxel::VoxelState::Empty;

    if (item.removed)
    {
        recorder.removedRoot();
    }

    recorder.changedRoot();

    MYVOXEL_ASSERT_MESSAGE(
        item.resultTree.isValid(),
        "Shape cut produced an invalid root tree.");
}

// 将全部成功计算的相交根结果串行提交到目标会话。
bool commitIntersectingRootCutItems(
    MyVoxel::VoxelShapeSession& session,
    std::vector<RootCutItem>& items,
    bool trackMaterialChanges,
    ShapeCutRecorder& recorder)
{
    static_cast<void>(recorder);

    bool changed = false;

    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        RootCutItem& item = items[itemIndex];

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
        recorder.accumulate(item.statistics);
#endif

        if (!item.changed)
        {
            continue;
        }

        if (item.removed)
        {
            const bool erased =
                trackMaterialChanges
                    ? session.eraseTree(
                        item.rootIndex,
                        item.dirtyRegion)
                    : session.eraseTree(
                        item.rootIndex);

            MYVOXEL_ASSERT_MESSAGE(
                erased,
                "An empty shape cut result root must be removable.");
        }
        else
        {
            MYVOXEL_ASSERT_MESSAGE(
                item.resultTree.isValid(),
                "Shape cut result root tree must be valid.");

            if (trackMaterialChanges)
            {
                session.setTree(
                    item.rootIndex,
                    std::move(item.resultTree),
                    item.dirtyRegion);
            }
            else
            {
                session.setTree(
                    item.rootIndex,
                    std::move(item.resultTree));
            }
        }

        changed = true;
    }

    return changed;
}

// 将被几何体完整覆盖的根串行删除。
bool commitRemovedRoots(
    MyVoxel::VoxelShapeSession& session,
    const std::vector<MyVoxel::VoxelCellIndex>& rootIndices,
    ShapeCutRecorder& recorder)
{
    bool changed = false;

    for (std::size_t rootPosition = 0;
         rootPosition < rootIndices.size();
         ++rootPosition)
    {
        const MyVoxel::VoxelCellIndex& rootIndex =
            rootIndices[rootPosition];
        const bool erased = session.eraseTree(rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            erased,
            "A fully covered shape cut root must be removable.");

        recorder.removedRoot();
        recorder.changedRoot();
        changed = true;
    }

    return changed;
}

// 使用一个空会话结果覆盖调用者提供的旧变化记录。
void resetChanges(const MyVoxel::VoxelShape& object, MyVoxel::VoxelChangeSet* changes)
{
    if (!changes)
    {
        return;
    }

    MyVoxel::VoxelShape emptyResult(object.grid());
    MyVoxel::VoxelShapeSession session(
        emptyResult,
        changes->trackingLevel());

    *changes = session.takeChanges();
}

// 将会话产生的变化记录交给调用者。
void storeChanges(MyVoxel::VoxelShapeSession& session, MyVoxel::VoxelChangeSet* changes)
{
    if (changes)
    {
        *changes = session.takeChanges();
    }
}

// 执行连续几何直接差集。
bool applyShapeCut(MyVoxel::VoxelShape& object,
                   const MyVoxel::Shape& tool,
                   MyVoxel::VoxelChangeSet* changes,
                   ShapeCutRecorder& recorder)
{
    MYVOXEL_ASSERT_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(tool.isValid(), "Shape cut requires a valid Shape.");

    resetChanges(object, changes);

    if (object.isEmpty())
    {
        return false;
    }

    const MyMath::Matrix4 objectTransform = object.transform();
    const MyVoxel::VoxelGrid grid = object.grid();
    const MyVoxel::Shape toolValue = tool;
    const MyVoxel::ShapeQuery preparationQuery(
        toolValue,
        objectTransform);

    std::vector<MyVoxel::VoxelCellIndex> removedRootIndices;
    std::vector<IntersectingRootPreparation> intersectingRoots;

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Foundation::Stopwatch rootClassificationTimer;
#endif

    classifyCandidateRoots(
        object,
        preparationQuery,
        grid,
        removedRootIndices,
        intersectingRoots,
        recorder);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addRootClassificationMilliseconds(
        rootClassificationTimer.elapsedMilliseconds());
#endif

    if (removedRootIndices.empty() && intersectingRoots.empty())
    {
        return false;
    }

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Foundation::Stopwatch rootPreparationTimer;
#endif

    const MyVoxel::VoxelLevel maximumLevel =
        grid.maximumLevel();
    const MyVoxel::VoxelLevel blockLevel =
        dirtyLeafBlockLevel(maximumLevel);
    const bool trackMaterialChanges =
        changes != nullptr;
    const MyVoxel::VoxelLevel trackingLevel =
        trackMaterialChanges
            ? changes->trackingLevel()
            : blockLevel;
    MyVoxel::VoxelShapeSession session(
        object,
        trackingLevel);
    std::vector<RootCutItem> items;

    prepareRootCutItems(
        session,
        intersectingRoots,
        items);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addRootPreparationMilliseconds(
        rootPreparationTimer.elapsedMilliseconds());
    MyVoxel::Foundation::Stopwatch executionTimer;
#endif

    if (!items.empty())
    {
        MyVoxel::Foundation::ParallelOptions options;
        options.minimumParallelTaskCount =
            MinimumParallelIntersectingRootCount;

        MyVoxel::Foundation::ParallelExecutor::global().execute(
            0,
            items.size(),
            [&](std::size_t blockBegin, std::size_t blockEnd)
            {
                // 每个批次使用独立ShapeQuery，避免未来查询器增加内部缓存后形成跨线程共享状态。
                const MyVoxel::ShapeQuery query(
                    toolValue,
                    objectTransform);

                for (std::size_t itemIndex = blockBegin;
                     itemIndex < blockEnd;
                     ++itemIndex)
                {
                    processIntersectingRootCut(
                        items[itemIndex],
                        query,
                        maximumLevel,
                        session,
                        trackMaterialChanges);
                }
            },
            options);
    }

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addIntersectingRootExecutionMilliseconds(
        executionTimer.elapsedMilliseconds());
    MyVoxel::Foundation::Stopwatch commitTimer;
#endif

    // execute抛出异常时不会到达提交阶段，目标森林保持原始根内容不变。
    bool changed =
        commitIntersectingRootCutItems(
            session,
            items,
            trackMaterialChanges,
            recorder);
    changed =
        commitRemovedRoots(
            session,
            removedRootIndices,
            recorder) ||
        changed;

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    recorder.addCommitMilliseconds(
        commitTimer.elapsedMilliseconds());
#endif

    storeChanges(session, changes);
    return changed;
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

ShapeCutStatistics::ShapeCutStatistics()
{
    reset();
}

void ShapeCutStatistics::reset()
{
    broadPhaseRootCandidateCount = 0;
    processedRootCount = 0;
    changedRootCount = 0;
    removedRootCount = 0;

    rootBoundsConstructionCount = 0;
    scalarFastClassificationCount = 0;
    octantBatchClassificationCount = 0;
    derivedChildBoundsCount = 0;

    visitedCellCount = 0;
    emptySkippedCellCount = 0;
    classifiedCellCount = 0;
    outsideCellCount = 0;
    insideCellCount = 0;
    intersectingCellCount = 0;

    centerSampleCount = 0;
    centerRemovedCellCount = 0;
    removedBranchCount = 0;

    splitCount = 0;
    maskLeafBuildCount = 0;
    maskLeafOperationCount = 0;
    maskLeafRemovedCellCount = 0;

    mergeAttemptCount = 0;
    mergeSuccessCount = 0;

    rootClassificationMilliseconds = 0.0;
    rootPreparationMilliseconds = 0.0;
    intersectingRootExecutionMilliseconds = 0.0;
    intersectingRootCpuMilliseconds = 0.0;
    commitMilliseconds = 0.0;
}

void ShapeCutStatistics::accumulate(const ShapeCutStatistics& other)
{
    broadPhaseRootCandidateCount += other.broadPhaseRootCandidateCount;
    processedRootCount += other.processedRootCount;
    changedRootCount += other.changedRootCount;
    removedRootCount += other.removedRootCount;

    rootBoundsConstructionCount += other.rootBoundsConstructionCount;
    scalarFastClassificationCount += other.scalarFastClassificationCount;
    octantBatchClassificationCount += other.octantBatchClassificationCount;
    derivedChildBoundsCount += other.derivedChildBoundsCount;

    visitedCellCount += other.visitedCellCount;
    emptySkippedCellCount += other.emptySkippedCellCount;
    classifiedCellCount += other.classifiedCellCount;
    outsideCellCount += other.outsideCellCount;
    insideCellCount += other.insideCellCount;
    intersectingCellCount += other.intersectingCellCount;

    centerSampleCount += other.centerSampleCount;
    centerRemovedCellCount += other.centerRemovedCellCount;
    removedBranchCount += other.removedBranchCount;

    splitCount += other.splitCount;
    maskLeafBuildCount += other.maskLeafBuildCount;
    maskLeafOperationCount += other.maskLeafOperationCount;
    maskLeafRemovedCellCount += other.maskLeafRemovedCellCount;

    mergeAttemptCount += other.mergeAttemptCount;
    mergeSuccessCount += other.mergeSuccessCount;

    rootClassificationMilliseconds += other.rootClassificationMilliseconds;
    rootPreparationMilliseconds += other.rootPreparationMilliseconds;
    intersectingRootExecutionMilliseconds += other.intersectingRootExecutionMilliseconds;
    intersectingRootCpuMilliseconds += other.intersectingRootCpuMilliseconds;
    commitMilliseconds += other.commitMilliseconds;
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

bool ShapeCutAlgorithm::apply(VoxelShape& object, const Shape& tool,
                              VoxelChangeSet* changes, ShapeCutStatistics& statistics)
{
    statistics.reset();

    ShapeCutRecorder recorder(statistics);
    return applyShapeCut(object, tool, changes, recorder);
}

#endif

}
}
}