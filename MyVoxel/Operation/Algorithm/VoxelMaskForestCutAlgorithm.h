#ifndef MYVOXEL_OPERATION_ALGORITHM_VOXELMASKFORESTCUTALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_VOXELMASKFORESTCUTALGORITHM_H

#include <cstdint>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/Tree/VoxelPackedForest.h"
#include "VoxelMaskCutAlgorithm.h"
#include "MyVoxel/Core/VoxelShape.h"
namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 记录一次完全对齐体素森林掩码切削的根级和树级统计。
struct VoxelMaskForestCutStatistics
{
    // 清空全部统计。
    void reset();

    std::uint64_t rootCandidateCount = 0; // 工具森林实际存在的根节点数量。
    std::uint64_t existingObjectRootCount = 0; // 同索引下实际存在对象根树的数量。
    std::uint64_t intersectionVisitedCellCount = 0; // 根树相交预检访问的逻辑节点数量。
    std::uint64_t intersectionMaskTestCount = 0; // 根树相交预检执行64位掩码交集的次数。
    std::uint64_t intersectingRootCount = 0; // 实际存在材料交集的根树数量。
    std::uint64_t detachedRootCount = 0; // 因局部切削而执行写时复制分离的根树数量。
    std::uint64_t directRootEraseCount = 0; // 被完整材料工具根直接删除的对象根树数量。
    std::uint64_t emptiedRootEraseCount = 0; // 局部切削后变为空并从森林删除的根树数量。
    std::uint64_t modifiedRootCount = 0; // 实际发生材料变化的对象根树数量。
    VoxelMaskCutStatistics treeStatistics; // 全部局部根树同步切削统计。
};

// 对两个完全对齐的Packed森林执行原地体素布尔减，返回是否实际删除材料。
bool cutAlignedVoxelForest(
    VoxelPackedForest& objectForest,
    const VoxelPackedForest& toolForest,
    VoxelChangeSet* changes = nullptr,
    VoxelMaskForestCutStatistics* statistics = nullptr);
// 检查两个VoxelShape是否使用完全相同的局部体素网格和空间变换。
bool canUseAlignedVoxelMaskCut(const VoxelShape& object, const VoxelShape& tool);

// 对两个完全对齐的VoxelShape执行掩码布尔减。
VoxelShape cutAlignedVoxelShapes(
    const VoxelShape& object,
    const VoxelShape& tool,
    VoxelChangeSet* changes = nullptr,
    VoxelMaskForestCutStatistics* statistics = nullptr);
}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_VOXELMASKFORESTCUTALGORITHM_H