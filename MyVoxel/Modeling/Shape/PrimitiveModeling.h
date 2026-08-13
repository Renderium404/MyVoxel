#ifndef MYVOXEL_MODELING_SHAPE_PRIMITIVEMODELING_H
#define MYVOXEL_MODELING_SHAPE_PRIMITIVEMODELING_H

#include "MyMath/Matrix4.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

// 创建以局部原点为中心并与局部坐标轴平行的标准长方体Topology_Shape。
Topology_Shape createBox(double sizeX, double sizeY, double sizeZ);
// 创建球心位于局部原点的标准球体Topology_Shape。
Topology_Shape createSphere(double radius);
// 创建轴线沿局部Z轴并且中心位于局部原点的标准圆柱体Topology_Shape。
Topology_Shape createCylinder(double radius, double height);
// 创建底面位于局部Z负方向、顶点位于局部Z正方向的标准圆锥体Topology_Shape。
Topology_Shape createCone(double bottomRadius, double height);
// 创建底面位于局部Z负方向、顶面位于局部Z正方向的标准圆锥台Topology_Shape。
Topology_Shape createConeFrustum(double bottomRadius, double topRadius, double height);

/// 空间Shape实例创建

// 使用单位变换创建标准长方体Shape实例。
Shape makeBox(double sizeX, double sizeY, double sizeZ);
// 使用指定可逆仿射变换创建标准长方体Shape实例。
Shape makeBox(double sizeX, double sizeY, double sizeZ, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准球体Shape实例。
Shape makeSphere(double radius);
// 使用指定可逆仿射变换创建标准球体Shape实例。
Shape makeSphere(double radius, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准圆柱体Shape实例。
Shape makeCylinder(double radius, double height);
// 使用指定可逆仿射变换创建标准圆柱体Shape实例。
Shape makeCylinder(double radius, double height, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准圆锥体Shape实例。
Shape makeCone(double bottomRadius, double height);
// 使用指定可逆仿射变换创建标准圆锥体Shape实例。
Shape makeCone(double bottomRadius, double height, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准圆锥台Shape实例。
Shape makeConeFrustum(double bottomRadius, double topRadius, double height);
// 使用指定可逆仿射变换创建标准圆锥台Shape实例。
Shape makeConeFrustum(double bottomRadius, double topRadius, double height, const MyMath::Matrix4& localToWorld);

}
}

#endif // MYVOXEL_MODELING_SHAPE_PRIMITIVEMODELING_H
