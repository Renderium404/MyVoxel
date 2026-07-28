#ifndef MYVOXEL_SHAPE_H
#define MYVOXEL_SHAPE_H

#include <memory>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "ShapeBounds.h"
#include "ShapeGeometry.h"
#include "ShapeRegionRelation.h"

namespace MyVoxel
{

// 表示由不可变连续几何和独立空间变换组成的标准形体实例。
class Shape
{
public:
    // 使用指定连续几何创建单位变换下的Shape。
    explicit Shape(const std::shared_ptr<const ShapeGeometry>& geometry);

    // 使用指定连续几何和空间变换创建Shape。
    Shape(const std::shared_ptr<const ShapeGeometry>& geometry, const MyMath::Matrix4& transform);

    /// 连续几何

    // 返回当前Shape持有的不可变连续几何。
    const ShapeGeometry& geometry() const;

    // 返回当前Shape持有的连续几何共享句柄。
    const std::shared_ptr<const ShapeGeometry>& geometryHandle() const;

    // 检查两个Shape是否共享同一份连续几何。
    bool sharesGeometryWith(const Shape& other) const;

    /// 局部空间查询

    // 返回连续几何在自身局部坐标系中的轴对齐包围盒。
    ShapeBounds localBounds() const;

    // 判断指定局部坐标点是否位于连续实体材料中。
    bool containsLocalPoint(const MyMath::Vector3& point) const;

    // 保守判断指定局部包围盒与连续实体材料之间的关系。
    ShapeRegionRelation classifyLocalBounds(const ShapeBounds& bounds) const;

    /// 空间变换

    // 返回Shape局部坐标系到世界坐标系的变换。
    const MyMath::Matrix4& transform() const;

    // 设置Shape局部坐标系到世界坐标系的仿射变换。
    void setTransform(const MyMath::Matrix4& transform);

    // 将Shape空间变换恢复为单位矩阵。
    void resetTransform();

private:
    std::shared_ptr<const ShapeGeometry> m_geometry; // 当前Shape共享的不可变连续几何。
    MyMath::Matrix4 m_transform; // Shape局部坐标系到世界坐标系的变换。
};

}

#endif // MYVOXEL_SHAPE_H