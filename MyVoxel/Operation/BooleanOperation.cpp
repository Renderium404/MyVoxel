#include "BooleanOperation.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"

namespace
{

// 返回连续Shape直接差集结果。
MyVoxel::VoxelShape subtractReturned(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, MyVoxel::VoxelChangeSet* changes)
{
    MyVoxel::VoxelShape result = object;
    return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes) ? result : object;
}

// 原地执行连续Shape直接差集；共享目标无变化时保持原共享关系。
bool subtractInPlaceShape(MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, MyVoxel::VoxelChangeSet* changes)
{
    if (!object.isDataShared()) return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(object, tool, changes);

    MyVoxel::VoxelShape result = object;
    const bool changed = MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes);
    if (changed) object = result;
    return changed;
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 返回连续Shape直接差集结果并输出统计。
MyVoxel::VoxelShape subtractReturnedStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool,
                                               MyVoxel::VoxelChangeSet* changes,
                                               MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
{
    MyVoxel::VoxelShape result = object;
    return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics) ? result : object;
}

// 原地执行连续Shape直接差集并输出统计。
bool subtractInPlaceStatistics(MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, MyVoxel::VoxelChangeSet* changes,
                               MyVoxel::Operation::Algorithm::ShapeCutStatistics& statistics)
{
    if (!object.isDataShared()) return MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(object, tool, changes, statistics);

    MyVoxel::VoxelShape result = object;
    const bool changed = MyVoxel::Operation::Algorithm::ShapeCutAlgorithm::apply(result, tool, changes, statistics);
    if (changed) object = result;
    return changed;
}

#endif

}

namespace MyVoxel
{
namespace Operation
{

VoxelShape ShapeCutOperation::subtract(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape cut requires a valid Instance::Shape tool.");
    return subtractReturned(object, tool, changes);
}

bool ShapeCutOperation::subtractInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape cut requires a valid Instance::Shape tool.");
    return subtractInPlaceShape(object, tool, changes);
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

VoxelShape ShapeCutOperation::subtract(const VoxelShape& object, const Shape& tool,
                                       VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape cut requires a valid Instance::Shape tool.");
    return subtractReturnedStatistics(object, tool, changes, statistics);
}

bool ShapeCutOperation::subtractInPlace(VoxelShape& object, const Shape& tool,
                                        VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics)
{
    MYVOXEL_REQUIRE_MESSAGE(object.isValid(), "Shape cut requires a valid VoxelShape object.");
    MYVOXEL_REQUIRE_MESSAGE(tool.isValid(), "Shape cut requires a valid Instance::Shape tool.");
    return subtractInPlaceStatistics(object, tool, changes, statistics);
}

#endif

}
}