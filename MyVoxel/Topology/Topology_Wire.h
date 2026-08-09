#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Topology/Topology_Edge.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "MyVoxel/Topology/Topology_Vertex.h"

namespace MyVoxel
{

class Topology_TWire;

// 表示一维拓扑Wire的轻量句柄，共享Topology_TWire身份并通过Topology_Object方向决定整体遍历方向。
//
// Topology_TWire保存规范Forward方向下的有向Edge序列；Reversed Wire访问时反转Edge顺序并翻转每条Edge方向。
class Topology_Wire : public Topology_Object
{
public:
    // 构造不引用任何共享拓扑Wire实体的空句柄。
    Topology_Wire();
    // 使用至少一条按Topology_Vertex身份连续连接的有向Edge创建新的独立Topology_TWire实体和Forward句柄。
    explicit Topology_Wire(const std::vector<Topology_Edge>& edges);
    Topology_Wire(const Topology_Wire&) = default;
    Topology_Wire& operator=(const Topology_Wire&) = default;

    /// 拓扑状态

    // 判断当前Wire最后一个有向端点是否与第一个有向起点共享同一Topology_Vertex身份。
    bool isClosed() const;

    /// 有向Edge序列

    // 返回当前Wire包含的Edge数量，空句柄返回零。
    std::size_t edgeCount() const;
    // 返回当前Wire遍历方向下指定位置的Topology_Edge，Reversed Wire会反序并翻转对应Edge方向。
    Topology_Edge edge(std::size_t index) const;
    // 返回当前Wire遍历方向下的完整有向Topology_Edge序列。
    std::vector<Topology_Edge> edges() const;

    /// 有向拓扑端点

    // 返回当前Wire遍历方向下的起始Topology_Vertex。
    const Topology_Vertex& startVertex() const;
    // 返回当前Wire遍历方向下的终止Topology_Vertex。
    const Topology_Vertex& endVertex() const;

    /// 方向操作

    // 返回共享同一Topology_TWire身份但整体遍历方向相反的新句柄。
    Topology_Wire reversed() const;

private:
    // 使用已有共享Topology_TWire实体和明确方向构造拓扑Wire句柄。
    Topology_Wire(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation);
    // 返回当前句柄共享的强类型Topology_TWire实体。
    const Topology_TWire& tWire() const;
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H
