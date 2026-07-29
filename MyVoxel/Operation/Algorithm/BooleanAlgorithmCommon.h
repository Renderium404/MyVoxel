#ifndef MYVOXEL_OPERATION_ALGORITHM_BOOLEANALGORITHMCOMMON_H
#define MYVOXEL_OPERATION_ALGORITHM_BOOLEANALGORITHMCOMMON_H

#include <cstdint>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 表示工件体素与刀具材料区域之间的保守关系。
enum class CellRelation
{
    Outside,
    Intersecting,
    Inside
};

// 保存单个体素节点切削后的状态和材料变化结果。
struct CutCellResult
{
    CutCellResult(VoxelState stateValue, bool changedValue);

    VoxelState state; // 当前节点切削后的最终状态。
    bool changed; // 当前节点覆盖区域内是否实际删除了材料。
};

// 返回指定体素地址所属的第0层根节点索引。
VoxelCellIndex rootCellIndex(VoxelCellAddress address);

// 返回指定包围盒的中心点。
MyMath::Vector3 boundsCenter(const Bounds3& bounds);

// 返回指定体素在VoxelShape局部坐标中的中心点。
MyMath::Vector3 cellCenter(const VoxelShape& shape, const VoxelCellAddress& address);

// 返回VoxelGrid指定层级的体素边长。
double cellEdgeLength(const VoxelGrid& grid, VoxelLevel level);

// 收集VoxelShape当前全部材料叶节点地址。
std::vector<VoxelCellAddress> collectMaterialCells(const VoxelShape& shape);

// 收集指定第0层索引范围内实际存在的根节点。
std::vector<VoxelCellAddress> collectRootCellsInRange(const VoxelShape& shape, const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex);

// 执行无符号64位饱和乘法。
std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second);

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_BOOLEANALGORITHMCOMMON_H