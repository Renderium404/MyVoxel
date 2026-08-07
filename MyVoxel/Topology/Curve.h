#ifndef MYVOXEL_TOPOLOGY_CURVE_H
#define MYVOXEL_TOPOLOGY_CURVE_H

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "Topology_Curve.h"

namespace MyVoxel
{

// 表示Topology_Curve在世界空间中的一次不可变放置，缓存正反变换和保守世界轴对齐包围盒。
class Curve
{
public:
    // 构造不包含局部拓扑曲线的空Curve实例。
    Curve();
    // 使用单位变换放置局部拓扑曲线。
    explicit Curve(const Topology_Curve& topology);
    // 使用可逆仿射变换放置局部拓扑曲线。
    Curve(const Topology_Curve& topology, const MyMath::Matrix4& localToWorld);
    Curve(const Curve&) = default;
    Curve& operator=(const Curve&) = default;

    /// 状态判断

    // 判断当前Curve是否包含有效局部拓扑曲线和可逆仿射变换。
    bool isValid() const;
    // 判断当前Curve是否未包含局部拓扑曲线。
    bool isNull() const;
    // 判断当前Curve是否包含有效局部拓扑曲线和可逆仿射变换。
    explicit operator bool() const;
    // 判断当前Curve是否与另一条Curve共享同一份局部几何资源。
    bool sharesGeometryWith(const Curve& other) const;

    /// 局部拓扑与几何资源

    // 返回当前Curve持有的局部拓扑曲线，空对象调用属于调用错误。
    const Topology_Curve& topology() const;
    // 返回当前Curve最终持有的不可变几何资源，空对象调用属于调用错误。
    const Geometry_Curve& geometry() const;
    // 返回当前Curve最终持有的不可变几何资源普通指针，空对象返回空指针。
    const Geometry_Curve* geometryPointer() const;
    // 返回当前Curve的标准类型，空对象调用属于调用错误。
    CurveKind kind() const;

    /// 空间数据

    // 返回当前Curve在局部坐标系中的轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回局部坐标到世界坐标的变换矩阵。
    const MyMath::Matrix4& localToWorld() const;
    // 返回世界坐标到局部坐标的变换矩阵。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前Curve在世界坐标系中的保守轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 局部空间查询

    // 返回曲线局部起点。
    const MyMath::Vector3& localStartPoint() const;
    // 返回曲线局部终点。
    const MyMath::Vector3& localEndPoint() const;
    // 返回曲线局部完整长度。
    double localLength() const;
    // 返回规范化参数t对应的局部曲线点，t必须位于[0,1]。
    MyMath::Vector3 localPointAt(double t) const;
    // 返回规范化参数t对应且沿曲线前进方向的局部单位切向量，t必须位于[0,1]。
    MyMath::Vector3 localTangentAt(double t) const;

    /// 世界空间查询

    // 返回曲线世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回曲线世界终点。
    MyMath::Vector3 worldEndPoint() const;
    // 返回规范化参数t对应的世界曲线点，t必须位于[0,1]。
    MyMath::Vector3 worldPointAt(double t) const;
    // 返回规范化参数t对应且沿曲线前进方向的世界单位切向量，t必须位于[0,1]。
    MyMath::Vector3 worldTangentAt(double t) const;

    /// 曲线创建

    // 返回空间轨迹相同且方向相反的新Curve实例，空对象返回空Curve。
    Curve reversed() const;

private:
    // 使用局部拓扑曲线和局部到世界变换初始化完整Curve实例数据。
    void initialize(const Topology_Curve& topology, const MyMath::Matrix4& localToWorld);

private:
    Topology_Curve m_topology; // 当前Curve实例持有的局部拓扑曲线。
    MyMath::Matrix4 m_localToWorld; // 局部坐标到世界坐标的变换矩阵。
    MyMath::Matrix4 m_worldToLocal; // 世界坐标到局部坐标的变换矩阵。
    Bounds3 m_worldBounds; // 当前Curve实例在世界坐标系中的保守轴对齐包围盒。
    bool m_valid; // 当前Curve实例是否包含完整有效数据。
};

// 表示空间曲线实例的有序列表。
typedef std::vector<Curve> CurveList;

}

#endif // MYVOXEL_TOPOLOGY_CURVE_H