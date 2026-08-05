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

    /// 标准空间查询
    // 判断指定局部坐标点是否位于几何体内部或边界上。
    virtual bool containsLocalPoint(const MyMath::Vector3& point) const = 0;
    // 返回指定局部轴对齐包围盒与几何体之间的保守空间关系。
    // 标准路径接收完整Bounds3，具体几何应保持实现直接、线性并便于验证。
    virtual ShapeRelation classifyLocalBounds(const Bounds3& bounds) const = 0;

    /// 快速空间查询
    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    // 默认实现构造Bounds3并回退到标准路径，因此现有及自定义几何不必提供快速实现。
    // 调用者必须保证center有限，extent有限且各分量非负。
    virtual ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
    {
        return classifyLocalBounds(Bounds3(center - extent, center + extent));
    }

protected:
    // 通过侵入式引用计数管理连续几何体生命周期。
    ~ShapeGeometry() override = default;
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEGEOMETRY_H