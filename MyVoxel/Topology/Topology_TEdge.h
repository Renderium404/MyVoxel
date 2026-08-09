#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_TEDGE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_TEDGE_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Topology/Topology_TObject.h"
#include "MyVoxel/Topology/Topology_Vertex.h"

namespace MyVoxel
{

// 表示真正被Topology_Edge句柄共享的一维拓扑实体，保存曲线裁剪定义和两个无方向拓扑端点。
class Topology_TEdge : public Topology_TObject
{
public:
    // 使用已验证的曲线几何、递增裁剪区间和两个有效端点构造共享拓扑边实体。
    Topology_TEdge(const Foundation::RefPtr<const Geometry_Curve>& geometry, double firstParameter, double lastParameter,
                   const Topology_Vertex& firstVertex, const Topology_Vertex& lastVertex);

    /// 几何支撑

    // 返回当前拓扑边引用的完整不可变曲线几何资源。
    const Foundation::RefPtr<const Geometry_Curve>& geometry() const;
    // 返回底层曲线裁剪区间的首参数。
    double firstParameter() const;
    // 返回底层曲线裁剪区间的末参数。
    double lastParameter() const;

    /// 拓扑端点

    // 返回firstParameter对应的标准起始Topology_Vertex。
    const Topology_Vertex& firstVertex() const;
    // 返回lastParameter对应的标准终止Topology_Vertex。
    const Topology_Vertex& lastVertex() const;

protected:
    // 通过最终Topology_Object引用释放共享拓扑边实体。
    ~Topology_TEdge() override;

private:
    Foundation::RefPtr<const Geometry_Curve> m_geometry; // 当前拓扑边引用的完整不可变曲线几何资源。
    double m_firstParameter; // 底层曲线递增裁剪区间的首参数。
    double m_lastParameter; // 底层曲线递增裁剪区间的末参数。
    Topology_Vertex m_firstVertex; // firstParameter对应且规范为Forward的拓扑顶点。
    Topology_Vertex m_lastVertex; // lastParameter对应且规范为Forward的拓扑顶点。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_TEDGE_H
