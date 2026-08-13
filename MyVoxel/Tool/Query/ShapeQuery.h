#ifndef MYVOXEL_QUERY_SHAPEQUERY_H
#define MYVOXEL_QUERY_SHAPEQUERY_H

#include <array>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/ShapeRelation.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{

// 表示一个Shape在指定查询坐标系中的不可变空间查询器，缓存坐标变换和包围盒快速转换数据。
// 标准查询保持固定实现作为正确性基准，特殊Shape查询器可覆写受保护快速查询实现而不绕过公共参数验证。
class ShapeQuery
{
public:
    enum
    {
        OctantCount = 8
    };

public:
    // 使用Topology_Shape自身局部坐标系创建通用查询器。
    explicit ShapeQuery(const Topology_Shape& topology);
    // 使用Shape所在世界坐标系创建通用查询器。
    explicit ShapeQuery(const Shape& shape);
    // 使用指定查询坐标系创建通用查询器，queryToWorld必须为可逆仿射变换。
    ShapeQuery(const Shape& shape, const MyMath::Matrix4& queryToWorld);
    ShapeQuery(const ShapeQuery&) = delete;
    ShapeQuery& operator=(const ShapeQuery&) = delete;
    virtual ~ShapeQuery();

    /// 状态判断

    // 判断当前查询器是否包含有效Shape和可逆坐标变换。
    bool isValid() const;
    // 判断查询坐标系是否与Shape局部坐标系完全相同。
    bool isIdentityQuery() const;
    // 判断当前几何和查询坐标变换是否能够提供精确查询空间有符号距离。
    bool supportsSignedDistance() const;

    /// 查询对象与空间数据

    // 返回当前查询器引用的空间Shape实例。
    const Shape& shape() const;
    // 返回当前查询器引用的局部拓扑Shape。
    const Topology_Shape& topology() const;
    // 返回当前查询器引用的局部实体几何资源。
    const Geometry_Shape& geometry() const;
    // 返回查询坐标到Shape局部坐标的变换。
    const MyMath::Matrix4& queryToLocal() const;
    // 返回Shape局部坐标到查询坐标的变换。
    const MyMath::Matrix4& localToQuery() const;
    // 返回Shape在查询坐标系中的保守轴对齐包围盒。
    const Bounds3& queryBounds() const;

    /// 标准空间查询

    // 判断查询坐标系中的指定点是否位于Shape内部或边界上。
    bool containsPoint(const MyMath::Vector3& point) const;
    // 返回查询坐标系中指定点到Shape边界的精确有符号距离；仅支持提供局部距离且查询到局部为刚体或统一缩放变换的Shape。
    double signedDistanceToPoint(const MyMath::Vector3& point) const;
    // 使用完整Bounds3执行保守分类，非单位查询通过八角点变换后调用几何标准分类。
    ShapeRelation classifyBounds(const Bounds3& bounds) const;

    /// 快速空间查询

    // 使用已经计算好的查询空间包围盒中心和半尺寸执行保守分类。
    // 公共入口固定完成状态、参数和查询包围盒检查，再调用可覆写的快速实现。
    ShapeRelation classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;
    // 批量分类由parentCenter和childExtent定义的八个等尺寸子包围盒。
    // 公共入口固定完成状态和参数检查，再调用可覆写的八分体快速实现。
    void classifyOctantBoundsFast(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, std::array<ShapeRelation, OctantCount>& results) const;

protected:
    /// 特殊Shape快速查询扩展

    // 执行单个查询空间包围盒快速分类，派生查询器可针对特殊Shape覆写该热点实现。
    virtual ShapeRelation classifyBoundsFastImpl(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;
    // 执行八个等尺寸子包围盒批量快速分类，派生查询器可共享中间量并覆写该热点实现。
    virtual void classifyOctantBoundsFastImpl(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, std::array<ShapeRelation, OctantCount>& results) const;

    /// 通用快速查询辅助

    // 使用缓存的仿射矩阵绝对值行，将查询空间中心和半尺寸保守转换到Shape局部空间。
    void transformBoundsToLocalFast(const MyMath::Vector3& center, const MyMath::Vector3& extent, MyMath::Vector3& localCenter, MyMath::Vector3& localExtent) const;
    // 判断查询空间中心和半尺寸定义的包围盒是否与Shape查询包围盒相交。
    bool intersectsQueryBounds(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

private:
    // 使用空间Shape和查询坐标到世界坐标的变换初始化查询缓存。
    void initialize(const Shape& shape, const MyMath::Matrix4& queryToWorld);
    // 更新查询空间到局部空间线性变换的三个绝对值行。
    void updateAbsoluteQueryToLocalRows();
    // 检查查询空间到局部空间是否保持欧氏距离到统一比例，并缓存局部长度与查询长度的比例。
    void updateSignedDistanceMetric();

private:
    Shape m_shape; // 当前查询器引用的空间Shape实例。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Shape局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Shape局部坐标到查询坐标的变换。
    MyMath::Vector3 m_absoluteQueryToLocalRowX; // queryToLocal线性部分第0行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowY; // queryToLocal线性部分第1行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowZ; // queryToLocal线性部分第2行各分量绝对值。
    double m_localDistancePerQueryUnit; // 查询空间单位长度经过queryToLocal后对应的统一局部长度。
    Bounds3 m_queryBounds; // Shape在查询坐标系中的保守轴对齐包围盒。
    bool m_signedDistanceMetricValid; // queryToLocal是否保持欧氏距离到统一比例。
    bool m_identityQuery; // 查询坐标系是否与Shape局部坐标系完全相同。
    bool m_valid; // 当前查询器是否包含完整有效数据。
};

}

#endif // MYVOXEL_QUERY_SHAPEQUERY_H
