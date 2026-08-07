#ifndef MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H
#define MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H

#include "MyMath/Matrix4.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"
#include "MyVoxel/Topology/Topology_Wire.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

// 将局部XY平面中的闭合有面积Topology_Wire绕局部Z轴完整旋转并创建Topology_Shape。
Topology_Shape createRevolved(const Topology_Wire& profile);

/// 空间Shape实例创建

// 将局部轮廓绕局部Z轴完整旋转并使用单位变换创建Shape实例。
Shape makeRevolved(const Topology_Wire& profile);
// 将局部轮廓绕局部Z轴完整旋转并使用指定可逆仿射变换创建Shape实例。
Shape makeRevolved(const Topology_Wire& profile, const MyMath::Matrix4& localToWorld);

}
}

#endif // MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H
