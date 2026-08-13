#ifndef MYVOXEL_OPERATION_SHAPECUTOPERATION_H
#define MYVOXEL_OPERATION_SHAPECUTOPERATION_H

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Instance/Shape.h"
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"
#endif

namespace MyVoxel
{
namespace Operation
{

// 提供当前正式的VoxelShape-Instance::Shape连续体切削入口。
//
// 类只表达Difference切削，不包含Union、Intersection或ExclusiveOr。
// 返回式接口利用VoxelShape写时复制；原地接口在共享目标无变化时保持原共享关系。
class ShapeCutOperation
{
public:
    // 返回从object中减去连续tool后的VoxelShape。
    static VoxelShape subtract(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);
    // 从object中原地减去连续tool，返回TSDF是否发生实际变化。
    static bool subtractInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回连续Shape切削结果并输出算法内部统计。
    static VoxelShape subtract(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);
    // 原地执行连续Shape切削并输出算法内部统计。
    static bool subtractInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);

#endif

private:
    ShapeCutOperation() = delete;
};

}
}

#endif // MYVOXEL_OPERATION_SHAPECUTOPERATION_H