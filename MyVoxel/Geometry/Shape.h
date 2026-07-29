#ifndef MYVOXEL_GEOMETRY_SHAPE_H
#define MYVOXEL_GEOMETRY_SHAPE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "ShapeGeometry.h"
#include "ShapeKind.h"
#include "ShapeRelation.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示一个共享不可变几何数据的连续几何值对象。
class Shape
{
public:
    // 构造不包含几何数据的无效Shape。
    Shape();

    // 使用不可变几何数据构造Shape。
    explicit Shape(const Foundation::RefPtr<const ShapeGeometry>& geometry);

    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;

    /// 状态判断

    // 判断当前Shape是否持有有效几何数据。
    bool isValid() const;

    // 判断当前Shape是否与另一个Shape共享同一份几何数据。
    bool sharesGeometryWith(const Shape& other) const;

    /// 几何访问

    // 返回当前Shape持有的不可变几何数据。
    const ShapeGeometry& geometry() const;

    // 返回当前Shape持有的不可变几何数据普通指针，无效Shape返回空指针。
    const ShapeGeometry* geometryPointer() const;

    // 返回当前Shape的标准类型。
    ShapeKind kind() const;

    // 返回当前Shape的局部轴对齐包围盒。
    Bounds3 localBounds() const;

    /// 空间查询

    // 判断指定局部坐标点是否位于Shape内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const;

    // 返回指定局部轴对齐包围盒与Shape之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const;

private:
    Foundation::RefPtr<const ShapeGeometry> m_geometry; // 当前Shape共享的不可变几何数据。
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPE_H