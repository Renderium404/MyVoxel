#ifndef MYVOXEL_TOPOLOGY_SHAPE_H
#define MYVOXEL_TOPOLOGY_SHAPE_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "Topology_Shape.h"

namespace MyVoxel
{

// 表示Topology_Shape在世界空间中的一次不可变放置，缓存正反变换和世界轴对齐包围盒。
class Shape
{
public:
    // 构造不包含局部拓扑实体的空Shape实例。
    Shape();
    // 使用单位变换放置局部拓扑Shape。
    explicit Shape(const Topology_Shape& topology);
    // 使用可逆仿射变换放置局部拓扑Shape。
    Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld);
    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;

    /// 状态判断

    // 判断当前Shape是否包含有效局部拓扑实体和可逆仿射变换。
    bool isValid() const;
    // 判断当前Shape是否未包含局部拓扑实体。
    bool isNull() const;
    // 判断当前Shape是否包含有效局部拓扑实体和可逆仿射变换。
    explicit operator bool() const;
    // 判断当前Shape是否与另一个Shape共享同一份局部几何资源。
    bool sharesGeometryWith(const Shape& other) const;

    /// 局部拓扑与几何资源

    // 返回当前Shape持有的局部拓扑Shape，空对象调用属于调用错误。
    const Topology_Shape& topology() const;
    // 返回当前Shape最终持有的不可变几何资源，空对象调用属于调用错误。
    const Geometry_Shape& geometry() const;
    // 返回当前Shape最终持有的不可变几何资源普通指针，空对象返回空指针。
    const Geometry_Shape* geometryPointer() const;
    // 返回当前Shape的标准类型，空对象调用属于调用错误。
    ShapeKind kind() const;

    /// 空间数据

    // 返回当前Shape在局部坐标系中的轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回局部坐标到世界坐标的变换矩阵。
    const MyMath::Matrix4& localToWorld() const;
    // 返回世界坐标到局部坐标的变换矩阵。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前Shape在世界坐标系中的轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 局部空间查询

    // 判断指定局部坐标点是否位于Shape内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const;
    // 返回指定局部轴对齐包围盒与Shape之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const;
    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

    /// 世界空间查询

    // 判断指定世界坐标点是否位于Shape内部或边界上。
    bool containsWorldPoint(const MyMath::Vector3& point) const;
    // 返回指定世界轴对齐包围盒与Shape之间的保守空间关系。
    ShapeRelation classifyWorldBounds(const Bounds3& bounds) const;

private:
    // 使用局部拓扑Shape和局部到世界变换初始化完整Shape实例数据。
    void initialize(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld);

private:
    Topology_Shape m_topology; // 当前Shape实例持有的局部拓扑实体。
    MyMath::Matrix4 m_localToWorld; // 局部坐标到世界坐标的变换矩阵。
    MyMath::Matrix4 m_worldToLocal; // 世界坐标到局部坐标的变换矩阵。
    Bounds3 m_worldBounds; // 当前Shape实例在世界坐标系中的轴对齐包围盒。
    bool m_valid; // 当前Shape实例是否包含完整有效数据。
};

}

#endif // MYVOXEL_TOPOLOGY_SHAPE_H