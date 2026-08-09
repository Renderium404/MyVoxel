#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_EDGE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_EDGE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "MyVoxel/Topology/Topology_Vertex.h"

namespace MyVoxel
{

class Topology_TEdge;

// 表示一维拓扑边的轻量句柄，共享Topology_TEdge身份并通过Topology_Object方向决定遍历方向。
class Topology_Edge : public Topology_Object
{
public:
    // 构造不引用任何共享拓扑边实体的空句柄。
    Topology_Edge();
    // 使用曲线裁剪区间和拓扑端点创建新的独立Topology_TEdge实体和Forward句柄，geometryTolerance只用于构造时一致性检查。
    Topology_Edge(const Foundation::RefPtr<const Geometry_Curve>& geometry, double firstParameter, double lastParameter,
                  const Topology_Vertex& firstVertex, const Topology_Vertex& lastVertex, double geometryTolerance);
    Topology_Edge(const Topology_Edge&) = default;
    Topology_Edge& operator=(const Topology_Edge&) = default;

    /// 几何支撑

    // 返回当前拓扑边引用的完整Geometry_Curve。
    const Geometry_Curve& geometry() const;
    // 返回当前使用方向下起点对应的底层曲线参数。
    double parameterStart() const;
    // 返回当前使用方向下终点对应的底层曲线参数。
    double parameterEnd() const;

    /// 有向拓扑端点

    // 返回当前使用方向下的起始Topology_Vertex。
    const Topology_Vertex& startVertex() const;
    // 返回当前使用方向下的终止Topology_Vertex。
    const Topology_Vertex& endVertex() const;

    /// 有向几何查询

    // 返回当前使用方向下规范化参数t对应的曲线点。
    MyMath::Vector3 pointAt(double t) const;
    // 返回当前使用方向下规范化参数t对应的单位切向量。
    MyMath::Vector3 tangentAt(double t) const;

    /// 方向操作

    // 返回共享同一Topology_TEdge身份但使用方向相反的新句柄。
    Topology_Edge reversed() const;

private:
    // 使用已有共享Topology_TEdge实体和明确方向构造拓扑边句柄。
    Topology_Edge(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation);
    // 返回当前句柄共享的强类型Topology_TEdge实体。
    const Topology_TEdge& tEdge() const;
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_EDGE_H
