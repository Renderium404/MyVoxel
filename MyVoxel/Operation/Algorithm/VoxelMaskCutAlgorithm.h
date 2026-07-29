#ifndef MYVOXEL_OPERATION_ALGORITHM_VOXELMASKCUTALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_VOXELMASKCUTALGORITHM_H

#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelPackedRootTree.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 记录一次同坐标Packed根树掩码切削的内部操作数量。
struct VoxelMaskCutStatistics
{
    // 清空全部统计。
    void reset();

    // 累加另一次根树切削统计。
    void accumulate(const VoxelMaskCutStatistics& other);

    std::uint64_t visitedCellCount = 0; // 同步遍历访问的逻辑节点数量。
    std::uint64_t emptyObjectSkipCount = 0; // 因对象节点为空而跳过的节点数量。
    std::uint64_t emptyToolSkipCount = 0; // 因刀具节点为空而跳过的节点数量。
    std::uint64_t materialToolRemoveCount = 0; // 刀具完整材料节点直接删除对象分支的次数。
    std::uint64_t createdBranchCount = 0; // 为跟随刀具普通分支而创建对象普通分支的次数。
    std::uint64_t createdMaskLeafCount = 0; // 为跟随刀具掩码叶块而创建对象掩码叶块的次数。
    std::uint64_t recursiveNodeCount = 0; // 进入八子节点同步递归的节点数量。
    std::uint64_t maskOperationCount = 0; // 直接执行64位掩码差集的次数。
    std::uint64_t maskChangedCount = 0; // 64位掩码差集实际删除材料的次数。
    std::uint64_t mergeAttemptCount = 0; // 发生材料变化后尝试向上合并的次数。
    std::uint64_t mergeSuccessCount = 0; // 成功折叠为空或材料节点的次数。
};

// 保存一个根树切削后的状态及材料变化结果。
struct VoxelMaskCutResult
{
    VoxelMaskCutResult(VoxelState stateValue, bool changedValue);

    VoxelState state; // 对象根树切削后的最终状态。
    bool changed; // 对象根树中是否实际删除材料。
};

// 对两个完全对齐的Packed根树执行同步体素布尔减。
VoxelMaskCutResult cutAlignedVoxelTree(VoxelPackedRootTree& objectTree, const VoxelPackedRootTree& toolTree, VoxelMaskCutStatistics* statistics = nullptr);

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_VOXELMASKCUTALGORITHM_H