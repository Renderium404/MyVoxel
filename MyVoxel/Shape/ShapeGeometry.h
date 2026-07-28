#ifndef MYVOXEL_SHAPEGEOMETRY_H
#define MYVOXEL_SHAPEGEOMETRY_H

#include "MyMath/Vector3.h"

#include "ShapeBounds.h"
#include "ShapeRegionRelation.h"

namespace MyVoxel
{

// 定义连续封闭实体的局部几何查询接口。
class ShapeGeometry
{
public:
    virtual ~ShapeGeometry() = default;

    // 返回连续实体在自身局部坐标系中的轴对齐包围盒。
    virtual ShapeBounds localBounds() const = 0;

    // 判断指定局部坐标点是否位于连续实体材料中。
    virtual bool containsLocalPoint(const MyMath::Vector3& point) const = 0;

    // 保守判断指定局部包围盒与连续实体材料之间的关系。
    virtual ShapeRegionRelation classifyLocalBounds(const ShapeBounds& bounds) const = 0;
};

}

#endif // MYVOXEL_SHAPEGEOMETRY_H