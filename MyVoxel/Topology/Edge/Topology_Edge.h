#ifndef MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H
#define MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "Topology_TEdge.h"

namespace MyVoxel
{

// 作为共享拓扑边实体的轻量公开句柄，使用方向决定当前起终点及曲线参数方向。
class Topology_Edge : public Topology_Object
{
public:
    // 构造空拓扑边句柄。
    Topology_Edge();
    // 使用起终拓扑点和标准方向连续曲线创建新的拓扑边身份。
    Topology_Edge(const Topology_Vertex& startVertex, const Topology_Vertex& endVertex, const Foundation::RefPtr<const Geometry_Curve>& geometry, double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

    /// 拓扑数据
    // 返回当前使用方向的起始拓扑点。
    const Topology_Vertex& startVertex() const;
    // 返回当前使用方向的终止拓扑点。
    const Topology_Vertex& endVertex() const;

    /// 几何数据

    // 返回底层拓扑边标准方向对应的连续曲线几何，句柄反向不会复制或反转该几何资源。
    const Geometry_Curve& geometry() const;

    // 返回曲线完整长度。
    double length() const;

    // 返回当前使用方向规范化参数t对应的曲线点。
    MyMath::Vector3 pointAt(double t) const;
    // 返回当前使用方向规范化参数t对应的单位切向量。
    MyMath::Vector3 tangentAt(double t) const;

    /// 拓扑创建
    // 返回共享同一Topology_TEdge身份但使用方向相反的拓扑边句柄。
    Topology_Edge reversed() const;

private:
    // 使用已有共享拓扑边身份和指定方向创建句柄。
    Topology_Edge(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation);

    // 返回当前句柄引用的强类型拓扑边实体。
    const Topology_TEdge& tEdge() const;
};

}

#endif // MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H