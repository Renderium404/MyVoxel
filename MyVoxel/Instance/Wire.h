#ifndef MYVOXEL_TOPOLOGY_WIRE_H
#define MYVOXEL_TOPOLOGY_WIRE_H

#include <cstddef>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Instance/Instance_Object.h"
#include "Curve.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"

namespace MyVoxel
{

// 表示Topology_Wire在世界空间中的一次不可变放置，并缓存局部和世界轴对齐包围盒。
class Wire : public Instance_Object
{
public:
    // 构造不包含局部拓扑Wire的空实例。
    Wire();
    // 使用单位变换放置局部Topology_Wire。
    explicit Wire(const Topology_Wire& topology);
    // 使用指定可逆仿射变换放置局部Topology_Wire。
    Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld);
    Wire(const Wire&) = default;
    Wire& operator=(const Wire&) = default;

    /// 状态判断

    // 判断当前Wire是否包含有效局部拓扑、有效空间放置和有效空间范围。
    bool isValid() const;
    // 判断当前Wire是否未包含局部拓扑实体。
    bool isNull() const;
    // 判断当前Wire是否包含完整有效数据。
    explicit operator bool() const;
    // 判断当前Wire拓扑是否首尾闭合。
    bool isClosed() const;

    /// 局部拓扑

    // 返回当前Wire持有的局部Topology_Wire。
    const Topology_Wire& topology() const;
    // 返回当前Wire包含的Edge数量，空实例返回零。
    std::size_t edgeCount() const;
    // 返回当前Wire方向下指定位置的局部Topology_Edge。
    Topology_Edge edge(std::size_t index) const;
    // 返回当前Wire方向下的全部局部Topology_Edge。
    std::vector<Topology_Edge> edges() const;

    /// Curve实例

    // 返回指定Edge在当前Wire空间放置下对应的Curve实例。
    Curve curve(std::size_t index) const;

    /// 空间数据

    // 返回当前Wire在局部坐标系中的保守轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回当前Wire从局部空间到世界空间的放置变换。
    const MyMath::Matrix4& localToWorld() const;
    // 返回当前Wire从世界空间到局部空间的逆放置变换。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回当前Wire在世界坐标系中的保守轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 有向端点

    // 返回当前Wire遍历方向下的局部起点。
    MyMath::Vector3 localStartPoint() const;
    // 返回当前Wire遍历方向下的局部终点。
    MyMath::Vector3 localEndPoint() const;
    // 返回当前Wire遍历方向下的世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回当前Wire遍历方向下的世界终点。
    MyMath::Vector3 worldEndPoint() const;

    /// 方向操作

    // 返回共享同一Topology_TWire身份、保持同一空间放置但遍历方向相反的新Wire实例。
    Wire reversed() const;

private:
    // 根据当前Topology_Wire和Instance_Object放置建立空间范围缓存。
    void initialize();

private:
    Topology_Wire m_topology; // 当前Wire实例持有的局部拓扑实体。
    Bounds3 m_localBounds; // 当前Wire在局部坐标系中的保守轴对齐包围盒。
    Bounds3 m_worldBounds; // 当前Wire在世界坐标系中的保守轴对齐包围盒。
};

}

#endif // MYVOXEL_TOPOLOGY_WIRE_H