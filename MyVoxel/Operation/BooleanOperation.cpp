#include "BooleanOperation.h"

#include "Algorithm/BooleanCutAlgorithm.h"

namespace MyVoxel
{
namespace Operation
{

/// 体素刀具布尔减

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool)
{
    return Algorithm::cutWithVoxelTool(object, tool, nullptr);
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes)
{
    return Algorithm::cutWithVoxelTool(object, tool, &changes);
}

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithVoxelToolStatistics(object, tool, nullptr, statistics);
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithVoxelToolStatistics(object, tool, &changes, statistics);
}

#endif

/// 连续刀具布尔减

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool)
{
    return Algorithm::cutWithShapeTool(object, tool, BooleanOperationOptions(), nullptr);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, VoxelChangeSet& changes)
{
    return Algorithm::cutWithShapeTool(object, tool, BooleanOperationOptions(), &changes);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options)
{
    return Algorithm::cutWithShapeTool(object, tool, options, nullptr);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes)
{
    return Algorithm::cutWithShapeTool(object, tool, options, &changes);
}

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithShapeToolStatistics(object, tool, BooleanOperationOptions(), nullptr, statistics);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithShapeToolStatistics(object, tool, BooleanOperationOptions(), &changes, statistics);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithShapeToolStatistics(object, tool, options, nullptr, statistics);
}

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return Algorithm::cutWithShapeToolStatistics(object, tool, options, &changes, statistics);
}

#endif

}
}