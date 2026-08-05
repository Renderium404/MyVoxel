#ifndef MYVOXEL_GEOMETRY_SHAPEQUERY_H
#define MYVOXEL_GEOMETRY_SHAPEQUERY_H

#include <array>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "Shape.h"
#include "ShapeInstance.h"
#include "ShapeRelation.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示一个连续Shape在指定查询坐标系中的不可变空间查询器。
//
// 标准包围盒分类接口接收Bounds3并通过八角点变换完成查询，逻辑直接且可独立用于验证。
// 快速接口接收已经计算好的中心和半尺寸，并利用缓存的矩阵数据及几何快速分类减少重复计算。
// 旋转或缩放后的局部轴对齐包围盒属于保守范围，可能增加Intersecting结果，但不会错误地排除相交区域。
class ShapeQuery
{
public:
    enum
    {
        OctantCount = 8
    };

public:
    // 使用Shape自身局部坐标系创建查询器。
    explicit ShapeQuery(const Shape& shape);

    // 使用指定查询空间到世界空间的变换创建ShapeInstance查询器，queryToWorld必须为可逆仿射变换。
    ShapeQuery(const ShapeInstance& instance, const MyMath::Matrix4& queryToWorld);

    /// 状态判断

    // 判断当前查询器是否包含有效几何和可逆坐标变换。
    bool isValid() const;

    // 判断查询空间是否与Shape局部空间完全相同。
    bool isIdentityQuery() const;

    /// 查询数据

    // 返回当前查询器引用的连续Shape。
    const Shape& shape() const;

    // 返回查询空间到Shape局部空间的变换。
    const MyMath::Matrix4& queryToLocal() const;

    // 返回Shape局部空间到查询空间的变换。
    const MyMath::Matrix4& localToQuery() const;

    // 返回Shape在查询空间中的轴对齐包围盒。
    const Bounds3& queryBounds() const;

    /// 标准空间查询

    // 判断查询空间中的指定点是否位于Shape内部或边界上。
    bool containsPoint(const MyMath::Vector3& point) const;

    // 使用完整Bounds3执行保守分类。
    //
    // 非单位查询通过八个角点变换生成局部轴对齐包围盒，逻辑独立且适合作为快速路径的验证基准。
    ShapeRelation classifyBounds(const Bounds3& bounds) const;

    /// 快速空间查询

    // 使用已经计算好的查询空间包围盒中心和半尺寸执行保守分类。
    //
    // 调用者必须保证center有限，extent有限且各分量非负。
    ShapeRelation classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

    // 批量分类由parentCenter和childExtent定义的八个等尺寸子包围盒。
    //
    // 子包围盒中心按照三位二进制角点编号排列，第0、1、2位分别控制X、Y、Z方向。
    void classifyOctantBoundsFast(const MyMath::Vector3& parentCenter,
                                  const MyMath::Vector3& childExtent,
                                  std::array<ShapeRelation, OctantCount>& results) const;

private:
    // 更新查询空间到局部空间线性变换的三个绝对值行。
    void updateAbsoluteQueryToLocalRows();

    // 使用快速中心—半尺寸公式转换查询空间中心和半尺寸。
    void transformBoundsToLocalFast(const MyMath::Vector3& center,
                                    const MyMath::Vector3& extent,
                                    MyMath::Vector3& localCenter,
                                    MyMath::Vector3& localExtent) const;

private:
    Shape m_shape; // 当前查询器引用的不可变连续几何。
    MyMath::Matrix4 m_queryToLocal; // 查询空间到Shape局部空间的变换。
    MyMath::Matrix4 m_localToQuery; // Shape局部空间到查询空间的变换。
    MyMath::Vector3 m_absoluteQueryToLocalRowX; // queryToLocal线性部分第0行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowY; // queryToLocal线性部分第1行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowZ; // queryToLocal线性部分第2行各分量绝对值。
    Bounds3 m_queryBounds; // Shape在查询空间中的轴对齐包围盒。
    bool m_identityQuery; // 查询空间是否与Shape局部空间完全相同。
    bool m_valid; // 当前查询器是否包含完整有效数据。
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEQUERY_H