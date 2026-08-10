#include "Topology_Wire.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Topology/Wire/Topology_TWire.h"

namespace
{

// 判断全部有向Edge是否按精确共享Topology_Vertex身份组成连续链。
bool isConnectedEdgeSequence(const std::vector<MyVoxel::Topology_Edge>& edges)
{
    if (edges.empty())
    {
        return false;
    }

    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        if (!edges[index].isValid())
        {
            return false;
        }

        if (index + 1 < edges.size() && !edges[index].endVertex().isSame(edges[index + 1].startVertex()))
        {
            return false;
        }
    }

    return true;
}

// 验证有向Edge序列并创建新的共享Topology_TWire实体。
MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject> createTWire(const std::vector<MyVoxel::Topology_Edge>& edges)
{
    const bool valid = isConnectedEdgeSequence(edges);
    MYVOXEL_ASSERT_MESSAGE(valid, "Topology_Wire requires at least one valid Edge and exact shared-Vertex connectivity.");

    if (!valid)
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>();
    }

    const bool closed = edges.back().endVertex().isSame(edges.front().startVertex());
    return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>(new MyVoxel::Topology_TWire(edges, closed));
}

}

namespace MyVoxel
{

Topology_Wire::Topology_Wire()
{
}

Topology_Wire::Topology_Wire(const std::vector<Topology_Edge>& edges)
    : Topology_Object(createTWire(edges), Topology_Orientation::Forward)
{
}

Topology_Wire::Topology_Wire(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 拓扑状态

bool Topology_Wire::isClosed() const
{
    return isValid() && tWire().isClosed();
}

/// 有向Edge序列

std::size_t Topology_Wire::edgeCount() const
{
    return isValid() ? tWire().edgeCount() : 0;
}

Topology_Edge Topology_Wire::edge(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an edge of an invalid Topology_Wire.");
    MYVOXEL_ASSERT_MESSAGE(index < edgeCount(), "Topology_Wire edge index is out of range.");

    if (!isValid() || index >= edgeCount())
    {
        return Topology_Edge();
    }

    if (isForward())
    {
        return tWire().edge(index);
    }

    return tWire().edge(edgeCount() - 1 - index).reversed();
}

std::vector<Topology_Edge> Topology_Wire::edges() const
{
    std::vector<Topology_Edge> result;

    if (!isValid())
    {
        return result;
    }

    result.reserve(edgeCount());

    for (std::size_t index = 0; index < edgeCount(); ++index)
    {
        result.push_back(edge(index));
    }

    return result;
}

/// 有向拓扑端点

const Topology_Vertex& Topology_Wire::startVertex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the start vertex of an invalid Topology_Wire.");

    if (isForward())
    {
        return tWire().edge(0).startVertex();
    }

    return tWire().edge(tWire().edgeCount() - 1).endVertex();
}

const Topology_Vertex& Topology_Wire::endVertex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the end vertex of an invalid Topology_Wire.");

    if (isForward())
    {
        return tWire().edge(tWire().edgeCount() - 1).endVertex();
    }

    return tWire().edge(0).startVertex();
}

/// 方向操作

Topology_Wire Topology_Wire::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Topology_Wire.");

    if (!isValid())
    {
        return Topology_Wire();
    }

    return Topology_Wire(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TWire& Topology_Wire::tWire() const
{
    return *static_cast<const Topology_TWire*>(tObject().get());
}

}