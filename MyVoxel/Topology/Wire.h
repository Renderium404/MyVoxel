#ifndef MYVOXEL_TOPOLOGY_WIRE_H
#define MYVOXEL_TOPOLOGY_WIRE_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "Curve.h"
#include "Topology_Wire.h"

namespace MyVoxel
{

// 表示Topology_Wire在世界空间中的一次不可变放置，缓存正反变换和保守世界轴对齐包围盒。
class Wire
{
public:
    // 构造不包含局部拓扑Wire的空Wire实例。
    Wire();
    // 使用单位变换放置局部拓扑Wire。
    explicit Wire(const Topology_Wire& topology);
    // 使用可逆仿射变换放置局部拓扑Wire。
    Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld);
    Wire(const Wire&) = default;
    Wire& operator=(const Wire&) = default;

    /// 状态判断

    // 判断当前Wire是否包含有效局部拓扑Wire和可逆仿射变换。
    bool isValid() const;
    // 判断当前Wire是否未包含局部拓扑Wire。
    bool isNull() const;
    // 判断当前Wire是否包含有效局部拓扑Wire和可逆仿射变换。
    explicit operator bool() const;
    // 判断当前Wire是否为闭合Wire。
    bool isClosed() const;
    // 判断当前闭合Wire是否具有非零局部有符号面积。
    bool hasArea() const;

    /// 局部拓扑

    // 返回当前Wire持有的局部拓扑Wire，空对象调用属于调用错误。
    const Topology_Wire& topology() const;
    // 返回当前Wire包含的曲线数量。
    std::size_t curveCount() const;
    // 返回指定编号的局部拓扑曲线。
    const Topology_Curve& localCurve(std::size_t index) const;
    // 返回指定编号曲线经过当前变换后的空间Curve实例。
    Curve worldCurve(std::size_t index) const;

    /// 空间数据

    // 返回当前Wire在局部坐标系中的轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回局部坐标到世界坐标的变换矩阵。
    const MyMath::Matrix4& localToWorld() const;
    // 返回世界坐标到局部坐标的变换矩阵。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前Wire在世界坐标系中的保守轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 局部空间查询

    // 返回Wire局部起点。
    const MyMath::Vector3& localStartPoint() const;
    // 返回Wire局部终点。
    const MyMath::Vector3& localEndPoint() const;
    // 返回Wire全部局部曲线长度之和。
    double localLength() const;
    // 返回闭合Wire的局部精确有符号面积。
    double localSignedArea() const;
    // 判断指定局部XY平面点是否位于闭合Wire围成的区域内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point, double tolerance = 0.0) const;
    // 返回指定局部XY平面轴对齐矩形与闭合Wire区域之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds, double tolerance = 0.0) const;

    /// 世界空间查询

    // 返回Wire世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回Wire世界终点。
    MyMath::Vector3 worldEndPoint() const;

    /// Wire创建

    // 返回空间轨迹相同且方向相反的新Wire实例，空对象返回空Wire。
    Wire reversed() const;

private:
    // 使用局部拓扑Wire和局部到世界变换初始化完整Wire实例数据。
    void initialize(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld);

private:
    Topology_Wire m_topology; // 当前Wire实例持有的局部拓扑Wire。
    MyMath::Matrix4 m_localToWorld; // 局部坐标到世界坐标的变换矩阵。
    MyMath::Matrix4 m_worldToLocal; // 世界坐标到局部坐标的变换矩阵。
    Bounds3 m_worldBounds; // 当前Wire实例在世界坐标系中的保守轴对齐包围盒。
    bool m_valid; // 当前Wire实例是否包含完整有效数据。
};

}

#endif // MYVOXEL_TOPOLOGY_WIRE_H