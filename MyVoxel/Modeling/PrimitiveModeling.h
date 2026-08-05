#ifndef MYVOXEL_MODELING_PRIMITIVEMODELING_H
#define MYVOXEL_MODELING_PRIMITIVEMODELING_H

#include "MyVoxel/Geometry/Shape.h"

namespace MyVoxel
{
namespace Modeling
{

/// 标准基础体创建
// 创建以局部原点为中心并与局部坐标轴平行的标准长方体。
Geometry::Shape makeBox(double sizeX, double sizeY, double sizeZ);
// 创建球心位于局部原点的标准球体。
Geometry::Shape makeSphere(double radius);
// 创建轴线沿局部Z轴并且中心位于局部原点的标准圆柱体。
Geometry::Shape makeCylinder(double radius, double height);
// 创建底面位于局部Z负方向、顶点位于局部Z正方向的标准圆锥体。
Geometry::Shape makeCone(double bottomRadius, double height);
// 创建底面位于局部Z负方向、顶面位于局部Z正方向的标准圆锥台。
Geometry::Shape makeConeFrustum(double bottomRadius, double topRadius, double height);

}
}

#endif // MYVOXEL_MODELING_PRIMITIVEMODELING_H