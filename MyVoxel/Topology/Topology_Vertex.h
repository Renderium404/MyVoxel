#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_VERTEX_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_VERTEX_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"
#include "MyVoxel/Topology/Topology_Object.h"

namespace MyVoxel
{

// 表示零维拓扑顶点的轻量句柄，共享Topology_TVertex身份并继承Topology_Object使用方向语义。
//
// Topology_Vertex不直接保存坐标，几何位置由其Geometry_Point支撑资源提供。
class Topology_Vertex : public Topology_Object
{
public:
    // 构造不引用任何共享拓扑顶点实体的空句柄。
    Topology_Vertex();
    // 为指定非空Geometry_Point创建新的独立拓扑顶点实体和Forward句柄。
    explicit Topology_Vertex(const Foundation::RefPtr<const Geometry_Point>& geometry);
    Topology_Vertex(const Topology_Vertex&) = default;
    Topology_Vertex& operator=(const Topology_Vertex&) = default;

    /// 几何支撑

    // 返回当前拓扑顶点引用的Geometry_Point，空句柄调用属于调用错误。
    const Geometry_Point& geometry() const;

    /// 方向操作

    // 返回共享同一Topology_TVertex身份但使用方向相反的新句柄。
    Topology_Vertex reversed() const;

private:
    // 使用已有共享Topology_TVertex实体和明确方向构造拓扑顶点句柄。
    Topology_Vertex(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation);
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_VERTEX_H
