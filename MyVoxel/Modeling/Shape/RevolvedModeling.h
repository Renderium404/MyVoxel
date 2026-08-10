#ifndef MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H
#define MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H

#include "MyMath/Matrix4.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"
#include "MyVoxel/Instance/Wire.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

// 使用局部XY平面闭合Topology_Wire作为母线创建绕局部Z轴完整旋转的Topology_Shape。
Topology_Shape createRevolved(const Topology_Wire& profile, double profileTolerance);

/// 空间Shape实例创建

// 使用单位变换创建局部回转Shape实例。
Shape makeRevolved(const Topology_Wire& profile, double profileTolerance);
// 使用指定可逆仿射变换创建回转Shape实例。
Shape makeRevolved(const Topology_Wire& profile, double profileTolerance, const MyMath::Matrix4& localToWorld);
// 使用Wire自身空间放置创建回转Shape实例，Wire局部母线绕自身局部Z轴旋转后沿用相同localToWorld。
Shape makeRevolved(const Wire& profile, double profileTolerance);

}
}

#endif // MYVOXEL_MODELING_SHAPE_REVOLVEDMODELING_H
