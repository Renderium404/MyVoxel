#ifndef MYVOXEL_OPERATION_ALGORITHM_SHAPECUTCONTEXT_H
#define MYVOXEL_OPERATION_ALGORITHM_SHAPECUTCONTEXT_H

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 保存连续Shape在指定体素层级使用的固定递归数据。
struct ShapeToolLevelContext
{
    ShapeToolLevelContext();

    double halfExtentX; // 当前层级体素映射到Shape局部空间后的X方向包围盒半宽。
    double halfExtentY; // 当前层级体素映射到Shape局部空间后的Y方向包围盒半宽。
    double halfExtentZ; // 当前层级体素映射到Shape局部空间后的Z方向包围盒半宽。
    MyMath::Vector3 childCenterOffsets[VoxelCornerCount]; // 八个子节点中心相对父节点中心的Shape局部偏移。
};

// 保存连续Shape切削使用的宽相范围和全部层级递归数据。
struct ShapeToolContext
{
    MyMath::Matrix4 objectToTool; // 工件局部坐标到连续Shape局部坐标的变换。
    std::vector<ShapeToolLevelContext> levels; // 各体素层级预计算数据。
    VoxelCellIndex minimumRootIndex; // 连续Shape包围盒覆盖的最小第0层索引。
    VoxelCellIndex maximumRootIndex; // 连续Shape包围盒覆盖的最大第0层索引。
};

// 创建连续Shape切削使用的宽相范围和全部层级递归数据。
ShapeToolContext createShapeToolContext(const VoxelShape& object, const Geometry::ShapeInstance& tool);

// 返回连续Shape局部空间中当前体素中心和预计算半宽对应的包围盒。
Bounds3 shapeToolCellBounds(const ShapeToolContext& context, VoxelLevel level, const MyMath::Vector3& centerToolSpace);

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_SHAPECUTCONTEXT_H