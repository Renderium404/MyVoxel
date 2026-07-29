#ifndef MYVOXEL_GEOMETRY_SHAPEINSTANCE_H
#define MYVOXEL_GEOMETRY_SHAPEINSTANCE_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "Shape.h"
#include "ShapeRelation.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示一个连续Shape在世界空间中的不可变放置实例。
class ShapeInstance
{
public:
    // 构造不包含Shape的无效实例。
    ShapeInstance();
    // 使用单位变换构造Shape实例。
    explicit ShapeInstance(const Shape& shape);

    // 使用可逆仿射变换构造Shape实例。
    ShapeInstance(const Shape& shape, const MyMath::Matrix4& localToWorld);
    ShapeInstance(const ShapeInstance&) = default;
    ShapeInstance& operator=(const ShapeInstance&) = default;

    /// 状态判断
    // 判断当前实例是否包含有效Shape和可逆仿射变换。
    bool isValid() const;

    /// 实例数据

    // 返回当前实例引用的Shape。
    const Shape& shape() const;
    // 返回局部坐标到世界坐标的变换矩阵。
    const MyMath::Matrix4& localToWorld() const;
    // 返回世界坐标到局部坐标的变换矩阵。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前实例在世界坐标系中的轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 空间查询
    // 判断指定世界坐标点是否位于实例内部或边界上。
    bool containsWorldPoint(const MyMath::Vector3& point) const;
    // 返回指定世界轴对齐包围盒与实例之间的保守空间关系。
    ShapeRelation classifyWorldBounds(const Bounds3& bounds) const;

private:
    Shape m_shape; // 当前实例引用的连续Shape。
    MyMath::Matrix4 m_localToWorld; // 局部坐标到世界坐标的变换矩阵。
    MyMath::Matrix4 m_worldToLocal; // 世界坐标到局部坐标的变换矩阵。
    Bounds3 m_worldBounds; // 当前实例在世界坐标系中的轴对齐包围盒。
    bool m_valid; // 当前实例是否包含完整有效数据。
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEINSTANCE_H