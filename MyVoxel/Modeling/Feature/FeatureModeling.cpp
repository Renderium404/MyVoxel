#include "FeatureModeling.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Polyline.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace MyVoxel
{
namespace Modeling
{

/// 特征拓扑创建

Topology_Edge createFeatureEdge(const FeatureTrace& trace)
{
    MYVOXEL_ASSERT_MESSAGE(trace.hasSegment(),
                           "Feature edge modeling requires a FeatureTrace with at least two points.");

    if (!trace.hasSegment())
    {
        return Topology_Edge();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Polyline(trace.points()));
    const Topology_Vertex startVertex(trace.startPoint());

    if (trace.isClosed())
    {
        return Topology_Edge(startVertex, startVertex, geometry, 0.0);
    }

    const Topology_Vertex endVertex(trace.endPoint());
    return Topology_Edge(startVertex, endVertex, geometry, 0.0);
}

}
}