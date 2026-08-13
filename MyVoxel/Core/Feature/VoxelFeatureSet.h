#ifndef MYVOXEL_CORE_FEATURE_VOXELFEATURESET_H
#define MYVOXEL_CORE_FEATURE_VOXELFEATURESET_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace MyVoxel
{

// 保存VoxelShape局部空间中需要显式保留的零等值面拓扑特征，不创建独立于Topology体系的新身份。
class VoxelFeatureSet
{
public:
    VoxelFeatureSet();

    /// 状态与数据

    // 判断当前集合内部拓扑身份、Edge端点引用和缓存包围盒是否一致，空集合属于有效状态。
    bool isValid() const;
    // 判断当前集合是否不包含任何显式特征点和特征边。
    bool isEmpty() const;
    // 返回显式特征点数量。
    std::size_t vertexCount() const;
    // 返回显式特征边数量。
    std::size_t edgeCount() const;
    // 返回全部显式特征点句柄。
    const std::vector<Topology_Vertex>& vertices() const;
    // 返回全部显式特征边句柄。
    const std::vector<Topology_Edge>& edges() const;
    // 返回全部特征形成的局部轴对齐包围盒，空集合返回无效Bounds3。
    const Bounds3& bounds() const;

    /// 身份查询

    // 判断集合中是否存在共享同一Topology_TVertex身份的特征点。
    bool containsVertex(const Topology_Vertex& vertex) const;
    // 判断集合中是否存在共享同一Topology_TEdge身份的特征边，Edge使用方向不参与判断。
    bool containsEdge(const Topology_Edge& edge) const;
    // 返回与指定Topology_Vertex身份相连的特征Edge数量，自环Edge只计为一条关联Edge。
    std::size_t incidentEdgeCount(const Topology_Vertex& vertex) const;
    // 返回与指定Topology_Vertex身份相连的全部特征Edge。
    void incidentEdges(const Topology_Vertex& vertex, std::vector<Topology_Edge>& result) const;

    /// 集合修改

    // 添加有效特征点；同一Topology身份已经存在时不修改并返回false。
    bool addVertex(const Topology_Vertex& vertex);
    // 添加有效特征边，并自动确保其起终Topology_Vertex存在于集合中；同一Edge身份已存在时返回false。
    bool addEdge(const Topology_Edge& edge);
    // 删除指定Topology_Edge身份；不会自动删除失去关联的Vertex。
    bool removeEdge(const Topology_Edge& edge);
    // 删除未被任何特征Edge引用的Topology_Vertex；仍有Edge引用或不存在时返回false。
    bool removeVertex(const Topology_Vertex& vertex);
    // 清空全部显式特征。
    void clear();

    /// 空间查询

    // 返回位于指定局部包围盒内部或边界上的显式特征点。
    void queryVertices(const Bounds3& queryBounds, std::vector<Topology_Vertex>& result, double tolerance = 0.0) const;
    // 返回几何包围盒与指定局部包围盒相交或接触的显式特征边。
    void queryEdges(const Bounds3& queryBounds, std::vector<Topology_Edge>& result, double tolerance = 0.0) const;

private:
    // 根据当前Vertex和Edge完整重建集合包围盒。
    void rebuildBounds();

private:
    std::vector<Topology_Vertex> m_vertices; // 当前零等值面显式特征点，按Topology身份唯一。
    std::vector<Topology_Edge> m_edges; // 当前零等值面显式特征边，按Topology身份唯一。
    Bounds3 m_bounds; // 当前全部特征点和特征边几何范围。
};

}

#endif // MYVOXEL_CORE_FEATURE_VOXELFEATURESET_H