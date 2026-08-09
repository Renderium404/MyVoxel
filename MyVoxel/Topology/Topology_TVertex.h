#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_TVERTEX_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_TVERTEX_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace MyVoxel
{

// 表示真正被Topology_Vertex句柄共享的拓扑顶点实体，保存对应的零维几何资源。
class Topology_TVertex : public Topology_TObject
{
public:
    // 使用非空Geometry_Point资源构造共享拓扑顶点实体。
    explicit Topology_TVertex(const Foundation::RefPtr<const Geometry_Point>& geometry);

    /// 几何支撑

    // 返回当前拓扑顶点引用的不可变Geometry_Point资源。
    const Foundation::RefPtr<const Geometry_Point>& geometry() const;

protected:
    // 通过最终Topology_Object引用释放共享拓扑顶点实体。
    ~Topology_TVertex() override;

private:
    Foundation::RefPtr<const Geometry_Point> m_geometry; // 当前拓扑顶点引用的零维几何资源。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_TVERTEX_H
