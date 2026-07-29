#ifndef MYVOXEL_OPERATION_ALGORITHM_BOOLEANCUTALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_BOOLEANCUTALGORITHM_H

#include "BooleanAlgorithmConfig.h"

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"

namespace MyVoxel
{
namespace Operation
{

struct BooleanOperationOptions;

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

struct BooleanOperationStatistics;

#endif

namespace Algorithm
{

/// 体素刀具

// 使用体素刀具执行不带性能统计的布尔减。
VoxelShape cutWithVoxelTool(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet* changes);

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

// 使用体素刀具执行带性能统计的布尔减。
VoxelShape cutWithVoxelToolStatistics(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

/// 连续刀具

// 使用连续Shape实例执行不带性能统计的并行布尔减。
VoxelShape cutWithShapeTool(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet* changes);

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

// 使用连续Shape实例执行带性能统计的并行布尔减。
VoxelShape cutWithShapeToolStatistics(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_BOOLEANCUTALGORITHM_H