#ifndef MYVOXEL_GEOMETRY_SHAPEGEOMETRY_H
#define MYVOXEL_GEOMETRY_SHAPEGEOMETRY_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "ShapeKind.h"
#include "ShapeRelation.h"

namespace MyVoxel
{
namespace Geometry
{

// 定义局部坐标系中有限、封闭并且可执行空间查询的连续几何体。
class ShapeGeometry : public Foundation::ReferenceCounted
{
public:
    /// 几何属性

    // 返回当前连续几何体的标准类型。
    virtual ShapeKind kind() const = 0;

    // 返回当前连续几何体在局部坐标系中的轴对齐包围盒。
    virtual Bounds3 localBounds() const = 0;

    /// 空间查询

    // 判断指定局部坐标点是否位于几何体内部或边界上。
    virtual bool containsLocalPoint(const MyMath::Vector3& point) const = 0;

    // 返回指定局部轴对齐包围盒与几何体之间的保守空间关系。
    virtual ShapeRelation classifyLocalBounds(const Bounds3& bounds) const = 0;

protected:
    // 通过侵入式引用计数管理连续几何体生命周期。
    ~ShapeGeometry() override = default;
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEGEOMETRY_H