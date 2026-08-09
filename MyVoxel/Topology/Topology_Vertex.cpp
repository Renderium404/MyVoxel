#include "Topology_Vertex.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Topology/Topology_TVertex.h"

namespace
{

// 为非空Geometry_Point创建新的Topology_TVertex，并以Topology_TObject基类引用返回。
MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject> createTVertex(const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point>& geometry)
{
    if (!geometry)
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>();
    }

    return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>(new MyVoxel::Topology_TVertex(geometry));
}

}

namespace MyVoxel
{

Topology_Vertex::Topology_Vertex()
{
}

Topology_Vertex::Topology_Vertex(const Foundation::RefPtr<const Geometry_Point>& geometry)
    : Topology_Object(createTVertex(geometry), Topology_Orientation::Forward)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_Vertex requires a non-null Geometry_Point.");
}

Topology_Vertex::Topology_Vertex(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 几何支撑

const Geometry_Point& Topology_Vertex::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Topology_Vertex.");
    const Topology_TVertex* vertex = static_cast<const Topology_TVertex*>(tObject().get());
    return *vertex->geometry();
}

/// 方向操作

Topology_Vertex Topology_Vertex::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Topology_Vertex.");

    if (!isValid())
    {
        return Topology_Vertex();
    }

    return Topology_Vertex(tObject(), reversedOrientation());
}

}
