#ifndef MYVOXEL_OPERATION_ALGORITHM_VOXELFORESTBOOLEANALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_VOXELFORESTBOOLEANALGORITHM_H

#include <cstddef>

#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Operation/Algorithm/VoxelTreeBooleanAlgorithm.h"
#include "MyVoxel/Operation/VoxelBooleanMask.h"

namespace MyVoxel
{

class VoxelShapeSession;

namespace Operation
{
namespace Algorithm
{

// 保存一次森林级布尔运算的内部执行统计。
struct VoxelForestBooleanStatistics
{
    std::size_t candidateRootCount = 0; // 根据运算类型实际检查的候选根数量。
    std::size_t pairedRootCount = 0; // 左右森林中同时存在并进入树级运算的根数量。
    std::size_t copiedRootCount = 0; // 直接从右侧共享复制到左侧的根数量。
    std::size_t erasedRootCount = 0; // 运算后从左侧森林删除的根数量。
    std::size_t modifiedRootCount = 0; // 实际发生内容变化的左侧根数量。
    std::size_t skippedRootCount = 0; // 候选中不需要修改的根数量。
    VoxelTreeBooleanStatistics treeStatistics; // 全部配对根的累计树级统计。

    // 清空全部森林级和树级统计。
    void reset();
};

// 通过VoxelShapeSession对两个地址完全对齐的体素森林执行原地布尔运算。
//
// 结果写入left，right保持不变。
// 两个森林必须使用相同的体素网格、最高层级和掩码叶布局。
// 当前类只负责根索引和根树运算，不检查VoxelShape空间变换。
class VoxelForestBooleanAlgorithm
{
public:
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    // 执行带统计的森林级布尔运算，返回left是否发生实际变化。
    static bool apply(VoxelShapeSession& left, const VoxelForest& right, VoxelBooleanType type,
                      VoxelForestBooleanStatistics& statistics);
#else
    // 执行不带统计开销的森林级布尔运算，返回left是否发生实际变化。
    static bool apply(VoxelShapeSession& left, const VoxelForest& right, VoxelBooleanType type);
#endif

private:
    VoxelForestBooleanAlgorithm() = delete;
};

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_VOXELFORESTBOOLEANALGORITHM_H