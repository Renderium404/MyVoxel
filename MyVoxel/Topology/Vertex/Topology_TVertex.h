#ifndef MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_TVERTEX_H
#define MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_TVERTEX_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace MyVoxel
{

class Topology_Vertex;

// 保存一个共享拓扑点身份及其三维几何位置。
class Topology_TVertex : public Topology_TObject
{
    friend class Topology_Vertex;

public:
    /// 几何数据

    // 返回拓扑点对应的三维几何位置。
    const MyMath::Vector3& point() const;

protected:
    // 使用指定有限三维点创建共享拓扑点实体。
    explicit Topology_TVertex(const MyMath::Vector3& point);

    // 通过Topology_Vertex持有的最终共享引用释放拓扑点实体。
    ~Topology_TVertex() override = default;

private:
    MyMath::Vector3 m_point; // 当前拓扑点对应的三维几何位置。
};

}

#endif // MYVOXEL_TOPOLOGY_VERTEX_TOPOLOGY_TVERTEX_H