#include "Topology_TShape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_TShape::Topology_TShape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
    : m_geometry(geometry)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_TShape geometry must be non-null.");
}

/// 几何内核

const Geometry_Shape& Topology_TShape::geometry() const
{
    return *m_geometry;
}

}