#ifndef MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLANSTATISTICS_H
#define MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLANSTATISTICS_H

#include "BooleanAlgorithmConfig.h"

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

#include <vector>

#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "ShapeCutContext.h"
#include "ShapeCutPlan.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 为全部候选根节点并行生成带线程局部统计的切削计划。
void buildShapeCutPlansStatistics(const VoxelShape& object, const Geometry::ShapeInstance& tool, const ShapeToolContext& context, const std::vector<VoxelCellAddress>& rootCells, unsigned int workerCount, std::vector<RootCutPlan>& plans, std::vector<BooleanOperationStatistics>& localStatistics);

}
}
}

#endif

#endif // MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLANSTATISTICS_H