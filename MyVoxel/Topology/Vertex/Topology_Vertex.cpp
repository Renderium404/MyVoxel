#include "Topology_Vertex.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_Vertex::Topology_Vertex()
{
}

Topology_Vertex::Topology_Vertex(const MyMath::Vector3& point)
    : Topology_Object(Foundation::RefPtr<const Topology_TVertex>(new Topology_TVertex(point)), Topology_Orientation::Forward)
{
}

/// 几何数据

const MyMath::Vector3& Topology_Vertex::point() const
{
    return tVertex().point();
}

/// 内部访问

const Topology_TVertex& Topology_Vertex::tVertex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid Topology_Vertex.");
    return *static_cast<const Topology_TVertex*>(tObject().get());
}

}