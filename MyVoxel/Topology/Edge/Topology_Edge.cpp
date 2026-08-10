#include "Topology_Edge.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_Edge::Topology_Edge()
{
}

Topology_Edge::Topology_Edge(const Topology_Vertex& startVertex, const Topology_Vertex& endVertex, const Foundation::RefPtr<const Geometry_Curve>& geometry, double connectionTolerance)
    : Topology_Object(Foundation::RefPtr<const Topology_TEdge>(new Topology_TEdge(startVertex, endVertex, geometry, connectionTolerance)), Topology_Orientation::Forward)
{
}

Topology_Edge::Topology_Edge(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 拓扑数据

const Topology_Vertex& Topology_Edge::startVertex() const
{
    return isForward() ? tEdge().startVertex() : tEdge().endVertex();
}

const Topology_Vertex& Topology_Edge::endVertex() const
{
    return isForward() ? tEdge().endVertex() : tEdge().startVertex();
}

/// 几何数据

const Geometry_Curve& Topology_Edge::geometry() const
{
    return tEdge().geometry();
}

double Topology_Edge::length() const
{
    return geometry().length();
}

MyMath::Vector3 Topology_Edge::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Edge.");
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Topology_Edge parameter must be in [0,1].");
    return geometry().pointAt(isForward() ? t : 1.0 - t);
}

MyMath::Vector3 Topology_Edge::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Edge.");
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Topology_Edge parameter must be in [0,1].");

    const MyMath::Vector3 tangent = geometry().tangentAt(isForward() ? t : 1.0 - t);
    return isForward() ? tangent : tangent * -1.0;
}

/// 拓扑创建

Topology_Edge Topology_Edge::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Topology_Edge.");
    return Topology_Edge(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TEdge& Topology_Edge::tEdge() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid Topology_Edge.");
    return *static_cast<const Topology_TEdge*>(tObject().get());
}

}