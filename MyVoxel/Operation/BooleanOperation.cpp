#include "BooleanOperation.h"

#include <cassert>

#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"
#include "MyVoxel/Operation/Algorithm/VoxelForestBooleanAlgorithm.h"

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Foundation/Stopwatch.h"
#endif

namespace
{

const double ExactAlignmentEpsilon = 0.0; // 快速布尔运算要求两个体素地址空间逐元素完全一致。

enum class DirectApplyResult
{
    NotHandled,
    Unchanged,
    Changed
};

// 返回本次操作使用的变化跟踪层级；不需要输出变化时使用第0层。
MyVoxel::VoxelLevel changeTrackingLevel(const MyVoxel::VoxelChangeSet* changes)
{
    return changes ? changes->trackingLevel() : MyVoxel::BaseVoxelLevel;
}

// 使用一个空会话结果覆盖调用者提供的旧变化记录。
void resetChanges(const MyVoxel::VoxelShape& shape, MyVoxel::VoxelChangeSet* changes)
{
    if (!changes)
    {
        return;
    }

    MyVoxel::VoxelShape emptyResult(shape.grid());
    MyVoxel::VoxelShapeSession session(emptyResult, changes->trackingLevel());
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

// 将右侧全部非空根树复制到当前空会话结果。
void copyAllRoots(MyVoxel::VoxelShapeSession& left, const MyVoxel::VoxelForest& right)
{
    right.forEachRootCell(
        [&left, &right](const MyVoxel::VoxelCellAddress& rootAddress)
        {
            const MyVoxel::VoxelTree* tree = right.getTree(rootAddress.index);

            assert(tree);
            left.setTree(rootAddress.index, *tree);
        });
}

// 清空非空体素体并通过会话产生完整根变化。
bool clearShape(MyVoxel::VoxelShape& shape,
                MyVoxel::VoxelLevel trackingLevel,
                MyVoxel::VoxelChangeSet* changes)
{
    if (shape.isEmpty())
    {
        return false;
    }

    MyVoxel::VoxelShapeSession session(shape, trackingLevel);
    const bool changed = session.clear();
    storeChanges(session, changes);
    return changed;
}

// 处理同数据和空操作数产生的确定结果。
DirectApplyResult applyDirect(MyVoxel::VoxelShape& left,
                              const MyVoxel::VoxelShape& right,
                              MyVoxel::Operation::VoxelBooleanType type,
                              MyVoxel::VoxelLevel trackingLevel,
                              MyVoxel::VoxelChangeSet* changes)
{
    if (left.sharesDataWith(right))
    {
        switch (type)
        {
        case MyVoxel::Operation::VoxelBooleanType::Union:
        case MyVoxel::Operation::VoxelBooleanType::Intersection:
            return DirectApplyResult::Unchanged;

        case MyVoxel::Operation::VoxelBooleanType::Difference:
        case MyVoxel::Operation::VoxelBooleanType::ExclusiveOr:
            return clearShape(left, trackingLevel, changes)
                       ? DirectApplyResult::Changed
                       : DirectApplyResult::Unchanged;
        }

        assert(false);
    }

    if (left.isEmpty())
    {
        switch (type)
        {
        case MyVoxel::Operation::VoxelBooleanType::Union:
        case MyVoxel::Operation::VoxelBooleanType::ExclusiveOr:
        {
            if (right.isEmpty())
            {
                return DirectApplyResult::Unchanged;
            }

            MyVoxel::VoxelShapeSession session(left, trackingLevel);
            copyAllRoots(session, right.forest());
            storeChanges(session, changes);
            return DirectApplyResult::Changed;
        }

        case MyVoxel::Operation::VoxelBooleanType::Intersection:
        case MyVoxel::Operation::VoxelBooleanType::Difference:
            return DirectApplyResult::Unchanged;
        }

        assert(false);
    }

    if (right.isEmpty())
    {
        switch (type)
        {
        case MyVoxel::Operation::VoxelBooleanType::Union:
        case MyVoxel::Operation::VoxelBooleanType::Difference:
        case MyVoxel::Operation::VoxelBooleanType::ExclusiveOr:
            return DirectApplyResult::Unchanged;

        case MyVoxel::Operation::VoxelBooleanType::Intersection:
            return clearShape(left, trackingLevel, changes)
                       ? DirectApplyResult::Changed
                       : DirectApplyResult::Unchanged;
        }

        assert(false);
    }

    return DirectApplyResult::NotHandled;
}

#ifndef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 返回指定布尔运算结果。
MyVoxel::VoxelShape applyReturnedShape(const MyVoxel::VoxelShape& left,
                                       const MyVoxel::VoxelShape& right,
                                       MyVoxel::Operation::VoxelBooleanType type,
                                       MyVoxel::VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(left.isValid(), "Boolean operation requires a valid left VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(right.isValid(), "Boolean operation requires a valid right VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(MyVoxel::Operation::BooleanOperation::isAligned(left, right),
                            "Fast voxel boolean operation requires exactly aligned grids and transforms.");

    const MyVoxel::VoxelLevel trackingLevel = changeTrackingLevel(changes);
    resetChanges(left, changes);

    MyVoxel::VoxelShape result = left;
    const DirectApplyResult directResult =
        applyDirect(result, right, type, trackingLevel, changes);

    if (directResult != DirectApplyResult::NotHandled)
    {
        return result;
    }

    MyVoxel::VoxelShapeSession session(result, trackingLevel);
    const bool changed =
        MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
            session,
            right.forest(),
            type);

    storeChanges(session, changes);
    return changed ? result : left;
}

// 将指定布尔运算结果写回left。
bool applyInPlaceShape(MyVoxel::VoxelShape& left,
                       const MyVoxel::VoxelShape& right,
                       MyVoxel::Operation::VoxelBooleanType type,
                       MyVoxel::VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(left.isValid(), "Boolean operation requires a valid left VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(right.isValid(), "Boolean operation requires a valid right VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(MyVoxel::Operation::BooleanOperation::isAligned(left, right),
                            "Fast voxel boolean operation requires exactly aligned grids and transforms.");

    const MyVoxel::VoxelLevel trackingLevel = changeTrackingLevel(changes);
    resetChanges(left, changes);

    const DirectApplyResult directResult =
        applyDirect(left, right, type, trackingLevel, changes);

    if (directResult != DirectApplyResult::NotHandled)
    {
        return directResult == DirectApplyResult::Changed;
    }

    if (!left.isDataShared())
    {
        MyVoxel::VoxelShapeSession session(left, trackingLevel);
        const bool changed =
            MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
                session,
                right.forest(),
                type);

        storeChanges(session, changes);
        return changed;
    }

    // 数据共享时先在临时副本中执行，避免无变化运算使left失去原共享关系。
    MyVoxel::VoxelShape result = left;
    MyVoxel::VoxelShapeSession session(result, trackingLevel);

    const bool changed =
        MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
            session,
            right.forest(),
            type);

    storeChanges(session, changes);

    if (changed)
    {
        left = result;
    }

    return changed;
}

#else

// 累加树级布尔统计。
void accumulateTreeStatistics(MyVoxel::Operation::Algorithm::VoxelTreeBooleanStatistics& target,
                              const MyVoxel::Operation::Algorithm::VoxelTreeBooleanStatistics& source)
{
    target.visitedVoxelCount += source.visitedVoxelCount;
    target.leafMaskOperationCount += source.leafMaskOperationCount;
    target.directlyClearedChildCount += source.directlyClearedChildCount;
    target.directlyFilledChildCount += source.directlyFilledChildCount;
    target.copiedSubtreeCount += source.copiedSubtreeCount;
    target.invertedSubtreeCount += source.invertedSubtreeCount;
    target.recursiveChildCount += source.recursiveChildCount;
    target.collapsedVoxelCount += source.collapsedVoxelCount;
}

// 累加森林级布尔统计。
void accumulateForestStatistics(MyVoxel::Operation::Algorithm::VoxelForestBooleanStatistics& target,
                                const MyVoxel::Operation::Algorithm::VoxelForestBooleanStatistics& source)
{
    target.candidateRootCount += source.candidateRootCount;
    target.pairedRootCount += source.pairedRootCount;
    target.copiedRootCount += source.copiedRootCount;
    target.erasedRootCount += source.erasedRootCount;
    target.modifiedRootCount += source.modifiedRootCount;
    target.skippedRootCount += source.skippedRootCount;

    accumulateTreeStatistics(target.treeStatistics, source.treeStatistics);
}

// 初始化一次布尔运算统计。
void initializeStatistics(MyVoxel::Operation::BooleanOperationStatistics& statistics,
                          const MyVoxel::VoxelShape& left,
                          const MyVoxel::VoxelShape& right,
                          bool inPlace)
{
    statistics.reset();
    statistics.operationCount = 1;
    statistics.returnedShapeOperationCount = inPlace ? 0 : 1;
    statistics.inPlaceOperationCount = inPlace ? 1 : 0;
    statistics.inputLeftRootCount = left.rootCount();
    statistics.inputRightRootCount = right.rootCount();
}

// 返回指定布尔运算结果并记录统计。
MyVoxel::VoxelShape applyReturnedShapeStatistics(
    const MyVoxel::VoxelShape& left,
    const MyVoxel::VoxelShape& right,
    MyVoxel::Operation::VoxelBooleanType type,
    MyVoxel::VoxelChangeSet* changes,
    MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(left.isValid(), "Boolean operation requires a valid left VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(right.isValid(), "Boolean operation requires a valid right VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(MyVoxel::Operation::BooleanOperation::isAligned(left, right),
                            "Fast voxel boolean operation requires exactly aligned grids and transforms.");

    initializeStatistics(statistics, left, right, false);
    MyVoxel::Foundation::Stopwatch totalStopwatch;

    const MyVoxel::VoxelLevel trackingLevel = changeTrackingLevel(changes);
    resetChanges(left, changes);

    MyVoxel::VoxelShape result = left;
    const DirectApplyResult directResult =
        applyDirect(result, right, type, trackingLevel, changes);

    if (directResult != DirectApplyResult::NotHandled)
    {
        statistics.directResultCount = 1;
        statistics.outputRootCount = result.rootCount();
        statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
        return result;
    }

    MyVoxel::Foundation::Stopwatch detachStopwatch;
    MyVoxel::VoxelShapeSession session(result, trackingLevel);
    statistics.detachMilliseconds = detachStopwatch.elapsedMilliseconds();

    MyVoxel::Operation::Algorithm::VoxelForestBooleanStatistics forestStatistics;
    MyVoxel::Foundation::Stopwatch forestStopwatch;

    const bool changed =
        MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
            session,
            right.forest(),
            type,
            forestStatistics);

    statistics.forestMilliseconds = forestStopwatch.elapsedMilliseconds();
    statistics.forestStatistics = forestStatistics;
    storeChanges(session, changes);

    if (!changed)
    {
        result = left;
    }

    statistics.outputRootCount = result.rootCount();
    statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
    return result;
}

// 将指定布尔运算结果写回left并记录统计。
bool applyInPlaceShapeStatistics(
    MyVoxel::VoxelShape& left,
    const MyVoxel::VoxelShape& right,
    MyVoxel::Operation::VoxelBooleanType type,
    MyVoxel::VoxelChangeSet* changes,
    MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(left.isValid(), "Boolean operation requires a valid left VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(right.isValid(), "Boolean operation requires a valid right VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(MyVoxel::Operation::BooleanOperation::isAligned(left, right),
                            "Fast voxel boolean operation requires exactly aligned grids and transforms.");

    initializeStatistics(statistics, left, right, true);
    MyVoxel::Foundation::Stopwatch totalStopwatch;

    const MyVoxel::VoxelLevel trackingLevel = changeTrackingLevel(changes);
    resetChanges(left, changes);

    const DirectApplyResult directResult =
        applyDirect(left, right, type, trackingLevel, changes);

    if (directResult != DirectApplyResult::NotHandled)
    {
        statistics.directResultCount = 1;
        statistics.outputRootCount = left.rootCount();
        statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
        return directResult == DirectApplyResult::Changed;
    }

    MyVoxel::Operation::Algorithm::VoxelForestBooleanStatistics forestStatistics;
    bool changed = false;

    if (!left.isDataShared())
    {
        MyVoxel::Foundation::Stopwatch detachStopwatch;
        MyVoxel::VoxelShapeSession session(left, trackingLevel);
        statistics.detachMilliseconds = detachStopwatch.elapsedMilliseconds();

        MyVoxel::Foundation::Stopwatch forestStopwatch;
        changed = MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
            session,
            right.forest(),
            type,
            forestStatistics);
        statistics.forestMilliseconds = forestStopwatch.elapsedMilliseconds();
        storeChanges(session, changes);
    }
    else
    {
        ++statistics.sharedTargetCopyCount;

        MyVoxel::VoxelShape result = left;
        MyVoxel::Foundation::Stopwatch detachStopwatch;
        MyVoxel::VoxelShapeSession session(result, trackingLevel);
        statistics.detachMilliseconds = detachStopwatch.elapsedMilliseconds();

        MyVoxel::Foundation::Stopwatch forestStopwatch;
        changed = MyVoxel::Operation::Algorithm::VoxelForestBooleanAlgorithm::apply(
            session,
            right.forest(),
            type,
            forestStatistics);
        statistics.forestMilliseconds = forestStopwatch.elapsedMilliseconds();
        storeChanges(session, changes);

        if (changed)
        {
            left = result;
        }
    }

    statistics.forestStatistics = forestStatistics;
    statistics.outputRootCount = left.rootCount();
    statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
    return changed;
}

// 返回指定布尔运算结果，统计版本中保留普通接口并丢弃内部统计。
MyVoxel::VoxelShape applyReturnedShape(const MyVoxel::VoxelShape& left,
                                       const MyVoxel::VoxelShape& right,
                                       MyVoxel::Operation::VoxelBooleanType type,
                                       MyVoxel::VoxelChangeSet* changes)
{
    MyVoxel::Operation::BooleanOperationStatistics ignoredStatistics;
    return applyReturnedShapeStatistics(left, right, type, changes, ignoredStatistics);
}

// 将指定布尔运算结果写回left，统计版本中保留普通接口并丢弃内部统计。
bool applyInPlaceShape(MyVoxel::VoxelShape& left,
                       const MyVoxel::VoxelShape& right,
                       MyVoxel::Operation::VoxelBooleanType type,
                       MyVoxel::VoxelChangeSet* changes)
{
    MyVoxel::Operation::BooleanOperationStatistics ignoredStatistics;
    return applyInPlaceShapeStatistics(left, right, type, changes, ignoredStatistics);
}

#endif

}

namespace MyVoxel
{
namespace Operation
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

void BooleanOperationStatistics::reset()
{
    totalMilliseconds = 0.0;
    detachMilliseconds = 0.0;
    forestMilliseconds = 0.0;

    operationCount = 0;
    returnedShapeOperationCount = 0;
    inPlaceOperationCount = 0;
    directResultCount = 0;
    sharedTargetCopyCount = 0;

    inputLeftRootCount = 0;
    inputRightRootCount = 0;
    outputRootCount = 0;

    forestStatistics.reset();
}

void BooleanOperationStatistics::accumulate(const BooleanOperationStatistics& other)
{
    totalMilliseconds += other.totalMilliseconds;
    detachMilliseconds += other.detachMilliseconds;
    forestMilliseconds += other.forestMilliseconds;

    operationCount += other.operationCount;
    returnedShapeOperationCount += other.returnedShapeOperationCount;
    inPlaceOperationCount += other.inPlaceOperationCount;
    directResultCount += other.directResultCount;
    sharedTargetCopyCount += other.sharedTargetCopyCount;

    inputLeftRootCount += other.inputLeftRootCount;
    inputRightRootCount += other.inputRightRootCount;
    outputRootCount += other.outputRootCount;

    accumulateForestStatistics(forestStatistics, other.forestStatistics);
}

#endif

/// 空间兼容性

bool BooleanOperation::isAligned(const VoxelShape& left, const VoxelShape& right)
{
    if (!left.isValid() || !right.isValid())
    {
        return false;
    }

    return left.grid().isEqualTo(right.grid(), ExactAlignmentEpsilon) &&
           left.transform().isEqualTo(right.transform(), ExactAlignmentEpsilon);
}

/// 通用布尔运算

VoxelShape BooleanOperation::apply(const VoxelShape& left, const VoxelShape& right, VoxelBooleanType type, VoxelChangeSet* changes)
{
    return applyReturnedShape(left, right, type, changes);
}

bool BooleanOperation::applyInPlace(VoxelShape& left, const VoxelShape& right, VoxelBooleanType type, VoxelChangeSet* changes)
{
    return applyInPlaceShape(left, right, type, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::apply(const VoxelShape& left, const VoxelShape& right, VoxelBooleanType type,
                                   VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyReturnedShapeStatistics(left, right, type, changes, statistics);
}

bool BooleanOperation::applyInPlace(VoxelShape& left, const VoxelShape& right, VoxelBooleanType type,
                                    VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyInPlaceShapeStatistics(left, right, type, changes, statistics);
}

#endif

/// 并集

VoxelShape BooleanOperation::unite(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return apply(left, right, VoxelBooleanType::Union, changes);
}

bool BooleanOperation::uniteInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return applyInPlace(left, right, VoxelBooleanType::Union, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::unite(const VoxelShape& left, const VoxelShape& right,
                                   VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return apply(left, right, VoxelBooleanType::Union, changes, statistics);
}

bool BooleanOperation::uniteInPlace(VoxelShape& left, const VoxelShape& right,
                                    VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyInPlace(left, right, VoxelBooleanType::Union, changes, statistics);
}

#endif

/// 交集

VoxelShape BooleanOperation::intersect(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return apply(left, right, VoxelBooleanType::Intersection, changes);
}

bool BooleanOperation::intersectInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return applyInPlace(left, right, VoxelBooleanType::Intersection, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::intersect(const VoxelShape& left, const VoxelShape& right,
                                       VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return apply(left, right, VoxelBooleanType::Intersection, changes, statistics);
}

bool BooleanOperation::intersectInPlace(VoxelShape& left, const VoxelShape& right,
                                        VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyInPlace(left, right, VoxelBooleanType::Intersection, changes, statistics);
}

#endif

/// 差集

VoxelShape BooleanOperation::subtract(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return apply(left, right, VoxelBooleanType::Difference, changes);
}

bool BooleanOperation::subtractInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return applyInPlace(left, right, VoxelBooleanType::Difference, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::subtract(const VoxelShape& left, const VoxelShape& right,
                                      VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return apply(left, right, VoxelBooleanType::Difference, changes, statistics);
}

bool BooleanOperation::subtractInPlace(VoxelShape& left, const VoxelShape& right,
                                       VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyInPlace(left, right, VoxelBooleanType::Difference, changes, statistics);
}

#endif

/// 异或

VoxelShape BooleanOperation::exclusiveOr(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return apply(left, right, VoxelBooleanType::ExclusiveOr, changes);
}

bool BooleanOperation::exclusiveOrInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes)
{
    return applyInPlace(left, right, VoxelBooleanType::ExclusiveOr, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::exclusiveOr(const VoxelShape& left, const VoxelShape& right,
                                         VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return apply(left, right, VoxelBooleanType::ExclusiveOr, changes, statistics);
}

bool BooleanOperation::exclusiveOrInPlace(VoxelShape& left, const VoxelShape& right,
                                          VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    return applyInPlace(left, right, VoxelBooleanType::ExclusiveOr, changes, statistics);
}

#endif

/// 连续几何差集

VoxelShape BooleanOperation::subtract(const VoxelShape& object, const Geometry::ShapeInstance& tool,
                                      VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid ShapeInstance.");

    VoxelShape result = object;

    if (!Algorithm::ShapeCutAlgorithm::apply(result, tool, changes))
    {
        return object;
    }

    return result;
}

bool BooleanOperation::subtractInPlace(VoxelShape& object, const Geometry::ShapeInstance& tool,
                                       VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid ShapeInstance.");

    if (!object.isDataShared())
    {
        return Algorithm::ShapeCutAlgorithm::apply(object, tool, changes);
    }

    VoxelShape result = object;
    const bool changed = Algorithm::ShapeCutAlgorithm::apply(result, tool, changes);

    if (changed)
    {
        object = result;
    }

    return changed;
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape BooleanOperation::subtract(const VoxelShape& object, const Geometry::ShapeInstance& tool,
                                      VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid ShapeInstance.");

    VoxelShape result = object;

    if (!Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics))
    {
        return object;
    }

    return result;
}

bool BooleanOperation::subtractInPlace(VoxelShape& object, const Geometry::ShapeInstance& tool,
                                       VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid ShapeInstance.");

    if (!object.isDataShared())
    {
        return Algorithm::ShapeCutAlgorithm::apply(object, tool, changes, statistics);
    }

    VoxelShape result = object;
    const bool changed = Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics);

    if (changed)
    {
        object = result;
    }

    return changed;
}

#endif

}
}