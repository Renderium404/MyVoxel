#ifndef MYVOXEL_OPERATION_ALGORITHM_VOXELTREEBOOLEANALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_VOXELTREEBOOLEANALGORITHM_H

#include <cstddef>

#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Operation/VoxelBooleanMask.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 保存一次树级布尔运算的内部执行统计。
struct VoxelTreeBooleanStatistics
{
    std::size_t visitedVoxelCount = 0;          // 实际处理的逻辑体素对数量。
    std::size_t leafMaskOperationCount = 0;     // 直接执行64位叶块运算的次数。
    std::size_t directlyClearedChildCount = 0;  // 当前层直接设置为空的子体素数量。
    std::size_t directlyFilledChildCount = 0;   // 当前层直接设置为材料的子体素数量。
    std::size_t copiedSubtreeCount = 0;         // 执行子树复制的逻辑节点数量。
    std::size_t invertedSubtreeCount = 0;       // 执行子树反转的逻辑节点数量。
    std::size_t recursiveChildCount = 0;        // 实际进入递归的子体素数量。
    std::size_t collapsedVoxelCount = 0;        // 运算后折叠为空或材料的逻辑体素数量。
};

// 对两个结构兼容的VoxelTree执行原地布尔运算。
//
// 结果写入left，right保持不变。
// 两棵树必须对应相同的根空间、最大层级和掩码叶压缩规则。
class VoxelTreeBooleanAlgorithm
{
public:
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    // 执行带内部统计的树级布尔运算，返回left是否发生实际变化。
    static bool apply(VoxelTree& left, const VoxelTree& right, VoxelBooleanType type, VoxelTreeBooleanStatistics& statistics);
#else
    // 执行不带统计开销的树级布尔运算，返回left是否发生实际变化。
    static bool apply(VoxelTree& left, const VoxelTree& right, VoxelBooleanType type);
#endif

private:
    VoxelTreeBooleanAlgorithm() = delete;
};

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_VOXELTREEBOOLEANALGORITHM_H