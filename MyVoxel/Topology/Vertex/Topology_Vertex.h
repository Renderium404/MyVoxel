#ifndef MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_VERTEX_H
#define MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_VERTEX_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "Topology_TVertex.h"

namespace MyVoxel
{

// 作为共享拓扑点实体的轻量公开句柄。
class Topology_Vertex : public Topology_Object
{
public:
    // 构造空拓扑点句柄。
    Topology_Vertex();

    // 使用指定有限三维点创建新的拓扑点身份。
    explicit Topology_Vertex(const MyMath::Vector3& point);

    /// 几何数据

    // 返回当前拓扑点对应的三维几何位置。
    const MyMath::Vector3& point() const;

private:
    // 返回当前句柄引用的强类型拓扑点实体。
    const Topology_TVertex& tVertex() const;
};

}

#endif // MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_VERTEX_H