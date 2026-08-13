#ifndef MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H
#define MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Topology/Topology_TObject.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace MyVoxel
{

class Topology_Edge;

// 保存一个共享拓扑边身份及其标准方向顶点和连续曲线几何。
class Topology_TEdge : public Topology_TObject
{
    friend class Topology_Edge;

public:
    /// 拓扑数据

    // 返回标准方向起始拓扑点。
    const Topology_Vertex& startVertex() const;

    // 返回标准方向终止拓扑点。
    const Topology_Vertex& endVertex() const;

    /// 几何数据

    // 返回标准方向对应的连续曲线几何。
    const Geometry_Curve& geometry() const;

protected:
    // 使用标准方向起终拓扑点和连续曲线几何创建共享拓扑边实体。
    Topology_TEdge(const Topology_Vertex& startVertex, const Topology_Vertex& endVertex, const Foundation::RefPtr<const Geometry_Curve>& geometry, double connectionTolerance);

    // 通过Topology_Edge持有的最终共享引用释放拓扑边实体。
    ~Topology_TEdge() override = default;

private:
    Topology_Vertex m_startVertex; // 标准方向起始拓扑点。
    Topology_Vertex m_endVertex; // 标准方向终止拓扑点。
    Foundation::RefPtr<const Geometry_Curve> m_geometry; // 标准方向对应的不可变连续曲线几何。
};

}

#endif // MYVOXEL_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H