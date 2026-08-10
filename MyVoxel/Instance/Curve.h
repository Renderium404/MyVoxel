#ifndef MYVOXEL_INSTANCE_CURVE_H
#define MYVOXEL_INSTANCE_CURVE_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Instance/Instance_Object.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"

namespace MyVoxel
{

// 表示Topology_Edge在世界空间中的一次不可变放置，并缓存对应的世界轴对齐包围盒。
class Curve : public Instance_Object
{
public:
    // 构造不包含局部拓扑边的空Curve实例。
    Curve();
    // 使用单位变换放置局部拓扑边。
    explicit Curve(const Topology_Edge& topology);
    // 使用指定可逆仿射变换放置局部拓扑边。
    Curve(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld);
    Curve(const Curve&) = default;
    Curve& operator=(const Curve&) = default;

    /// 状态判断

    // 判断当前Curve是否包含有效局部拓扑边、有效空间放置和世界包围盒。
    bool isValid() const;
    // 判断当前Curve是否未包含局部拓扑边。
    bool isNull() const;
    // 判断当前Curve是否包含完整有效数据。
    explicit operator bool() const;
    // 判断当前Curve是否与另一个Curve最终引用同一个Geometry_Curve资源。
    bool sharesGeometryWith(const Curve& other) const;

    /// 局部拓扑与几何资源

    // 返回当前Curve持有的局部拓扑边。
    const Topology_Edge& topology() const;
    // 返回当前Curve最终持有的不可变连续曲线几何。
    const Geometry_Curve& geometry() const;
    // 返回当前Curve最终持有的不可变连续曲线几何普通指针，空对象返回空指针。
    const Geometry_Curve* geometryPointer() const;
    // 返回当前Curve的标准几何类型。
    CurveKind kind() const;

    /// 空间数据

    // 返回当前Curve在局部坐标系中的轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回当前Curve从局部空间到世界空间的放置变换。
    const MyMath::Matrix4& localToWorld() const;
    // 返回当前Curve从世界空间到局部空间的逆放置变换。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前Curve在世界坐标系中的轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 曲线属性

    // 返回当前拓扑使用方向中的局部起点。
    MyMath::Vector3 startPoint() const;
    // 返回当前拓扑使用方向中的局部终点。
    MyMath::Vector3 endPoint() const;
    // 返回当前拓扑使用方向中的世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回当前拓扑使用方向中的世界终点。
    MyMath::Vector3 worldEndPoint() const;
    // 返回连续曲线的局部几何长度。
    double length() const;

    /// 局部空间查询

    // 返回当前拓扑使用方向规范化参数t对应的局部曲线点。
    MyMath::Vector3 pointAt(double t) const;
    // 返回当前拓扑使用方向规范化参数t对应的局部单位切向量。
    MyMath::Vector3 tangentAt(double t) const;

    /// 世界空间查询

    // 返回当前拓扑使用方向规范化参数t对应的世界曲线点。
    MyMath::Vector3 worldPointAt(double t) const;
    // 返回当前拓扑使用方向规范化参数t对应的世界单位切向量。
    MyMath::Vector3 worldTangentAt(double t) const;

private:
    // 根据当前拓扑边和Instance_Object放置建立世界空间缓存。
    void initialize();

private:
    Topology_Edge m_topology; // 当前Curve实例持有的局部拓扑边。
    Bounds3 m_worldBounds; // 当前Curve实例在世界坐标系中的轴对齐包围盒。
};

}

#endif // MYVOXEL_INSTANCE_CURVE_H