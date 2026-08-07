#include "ShapeBooleanOperation.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"
#include "MyVoxel/Operation/BooleanOperation.h"

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Foundation/Stopwatch.h"
#endif

namespace
{

// 返回连续Shape直接差集结果。
MyVoxel::VoxelShape subtractReturned(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool,
                                     MyVoxel::VoxelChangeSet* changes)
{
    MyVoxel::VoxelShape result = object;
    return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes) ? result : object;
}

// 原地执行连续Shape直接差集，并保持共享目标在无变化时的共享关系。
bool subtractInPlaceShape(MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, MyVoxel::VoxelChangeSet* changes)
{
    if (!object.isDataShared())
    {
        return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(object, tool, changes);
    }

    MyVoxel::VoxelShape result = object;
    const bool changed = MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes);

    if (changed)
    {
        object = result;
    }

    return changed;
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 返回连续Shape直接差集结果并记录切削统计。
MyVoxel::VoxelShape subtractReturnedStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool,
                                               MyVoxel::VoxelChangeSet* changes,
                                               MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
{
    MyVoxel::VoxelShape result = object;
    return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics) ? result : object;
}

// 原地执行连续Shape直接差集并记录切削统计。
bool subtractInPlaceStatistics(MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool,
                               MyVoxel::VoxelChangeSet* changes,
                               MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
{
    if (!object.isDataShared())
    {
        return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(object, tool, changes, statistics);
    }

    MyVoxel::VoxelShape result = object;
    const bool changed = MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics);

    if (changed)
    {
        object = result;
    }

    return changed;
}

#endif

}

namespace MyVoxel
{
namespace Operation
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

ShapeBooleanStatistics::ShapeBooleanStatistics()
{
    reset();
}

void ShapeBooleanStatistics::reset()
{
    type = VoxelBooleanType::Union;
    usedDirectShapeCut = false;
    totalMilliseconds = 0.0;
    voxelizationMilliseconds = 0.0;
    booleanMilliseconds = 0.0;
    shapeCutMilliseconds = 0.0;
    voxelizationStatistics.reset();
    voxelBooleanStatistics.reset();
    shapeCutStatistics.reset();
}

#endif

/// 通用布尔运算

VoxelShape ShapeBooleanOperation::apply(const VoxelShape& object, const Shape& tool, VoxelBooleanType type, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape boolean operation requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape boolean operation requires a valid Shape tool.");

    if (type == VoxelBooleanType::Difference)
    {
        return subtractReturned(object, tool, changes);
    }

    const VoxelShape voxelTool = Modeling::voxelizeAligned(tool, object);
    return BooleanOperation::apply(object, voxelTool, type, changes);
}

bool ShapeBooleanOperation::applyInPlace(VoxelShape& object, const Shape& tool, VoxelBooleanType type, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape boolean operation requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape boolean operation requires a valid Shape tool.");

    if (type == VoxelBooleanType::Difference)
    {
        return subtractInPlaceShape(object, tool, changes);
    }

    const VoxelShape voxelTool = Modeling::voxelizeAligned(tool, object);
    return BooleanOperation::applyInPlace(object, voxelTool, type, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape ShapeBooleanOperation::apply(const VoxelShape& object, const Shape& tool, VoxelBooleanType type,
                                        VoxelChangeSet* changes, ShapeBooleanStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape boolean operation requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape boolean operation requires a valid Shape tool.");

    statistics.reset();
    statistics.type = type;
    Foundation::Stopwatch totalStopwatch;

    if (type == VoxelBooleanType::Difference)
    {
        statistics.usedDirectShapeCut = true;
        Foundation::Stopwatch shapeCutStopwatch;
        const VoxelShape result = subtractReturnedStatistics(object, tool, changes, statistics.shapeCutStatistics);
        statistics.shapeCutMilliseconds = shapeCutStopwatch.elapsedMilliseconds();
        statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
        return result;
    }

    Foundation::Stopwatch voxelizationStopwatch;
    const VoxelShape voxelTool = Modeling::voxelizeAligned(tool, object, statistics.voxelizationStatistics);
    statistics.voxelizationMilliseconds = voxelizationStopwatch.elapsedMilliseconds();

    Foundation::Stopwatch booleanStopwatch;
    const VoxelShape result = BooleanOperation::apply(object, voxelTool, type, changes, statistics.voxelBooleanStatistics);
    statistics.booleanMilliseconds = booleanStopwatch.elapsedMilliseconds();
    statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
    return result;
}

bool ShapeBooleanOperation::applyInPlace(VoxelShape& object, const Shape& tool, VoxelBooleanType type,
                                         VoxelChangeSet* changes, ShapeBooleanStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape boolean operation requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape boolean operation requires a valid Shape tool.");

    statistics.reset();
    statistics.type = type;
    Foundation::Stopwatch totalStopwatch;

    if (type == VoxelBooleanType::Difference)
    {
        statistics.usedDirectShapeCut = true;
        Foundation::Stopwatch shapeCutStopwatch;
        const bool changed = subtractInPlaceStatistics(object, tool, changes, statistics.shapeCutStatistics);
        statistics.shapeCutMilliseconds = shapeCutStopwatch.elapsedMilliseconds();
        statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
        return changed;
    }

    Foundation::Stopwatch voxelizationStopwatch;
    const VoxelShape voxelTool = Modeling::voxelizeAligned(tool, object, statistics.voxelizationStatistics);
    statistics.voxelizationMilliseconds = voxelizationStopwatch.elapsedMilliseconds();

    Foundation::Stopwatch booleanStopwatch;
    const bool changed = BooleanOperation::applyInPlace(object, voxelTool, type, changes, statistics.voxelBooleanStatistics);
    statistics.booleanMilliseconds = booleanStopwatch.elapsedMilliseconds();
    statistics.totalMilliseconds = totalStopwatch.elapsedMilliseconds();
    return changed;
}

#endif

/// 并集

VoxelShape ShapeBooleanOperation::unite(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return apply(object, tool, VoxelBooleanType::Union, changes);
}

bool ShapeBooleanOperation::uniteInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return applyInPlace(object, tool, VoxelBooleanType::Union, changes);
}

/// 交集

VoxelShape ShapeBooleanOperation::intersect(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return apply(object, tool, VoxelBooleanType::Intersection, changes);
}

bool ShapeBooleanOperation::intersectInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return applyInPlace(object, tool, VoxelBooleanType::Intersection, changes);
}

/// 差集

VoxelShape ShapeBooleanOperation::subtract(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid Shape tool.");
    return subtractReturned(object, tool, changes);
}

bool ShapeBooleanOperation::subtractInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid Shape tool.");
    return subtractInPlaceShape(object, tool, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape ShapeBooleanOperation::subtract(const VoxelShape& object, const Shape& tool,
                                           VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid Shape tool.");
    return subtractReturnedStatistics(object, tool, changes, statistics);
}

bool ShapeBooleanOperation::subtractInPlace(VoxelShape& object, const Shape& tool,
                                            VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape subtraction requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape subtraction requires a valid Shape tool.");
    return subtractInPlaceStatistics(object, tool, changes, statistics);
}

#endif

/// 异或

VoxelShape ShapeBooleanOperation::exclusiveOr(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return apply(object, tool, VoxelBooleanType::ExclusiveOr, changes);
}

bool ShapeBooleanOperation::exclusiveOrInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    return applyInPlace(object, tool, VoxelBooleanType::ExclusiveOr, changes);
}

}
}
