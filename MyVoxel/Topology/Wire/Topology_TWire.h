#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_TWIRE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_TWIRE_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace MyVoxel
{

// 表示真正被Topology_Wire句柄共享的一维拓扑Wire实体，保存规范Forward方向下的连续有向Edge序列。
class Topology_TWire : public Topology_TObject
{
public:
    // 使用至少一条连续有向Edge和已经确定的首尾闭合状态构造共享拓扑Wire实体。
    Topology_TWire(const std::vector<Topology_Edge>& edges, bool closed);

    /// Edge序列

    // 返回规范Forward Wire包含的Edge数量。
    std::size_t edgeCount() const;
    // 返回规范Forward Wire指定位置的有向Topology_Edge。
    const Topology_Edge& edge(std::size_t index) const;
    // 返回规范Forward Wire保存的全部有向Topology_Edge。
    const std::vector<Topology_Edge>& edges() const;

    /// 拓扑状态

    // 判断规范Forward Wire的最后一个终点是否与第一个起点共享Topology_Vertex身份。
    bool isClosed() const;

protected:
    // 通过最终Topology_Object引用释放共享拓扑Wire实体。
    ~Topology_TWire() override;

private:
    std::vector<Topology_Edge> m_edges; // 规范Forward Wire按遍历顺序保存的连续有向Edge句柄。
    bool m_closed; // 首尾是否通过同一个Topology_Vertex身份闭合。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_TWIRE_H