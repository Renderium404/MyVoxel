#include "Topology_Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_Shape::Topology_Shape()
{
}

Topology_Shape::Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
    : Topology_Object(Foundation::RefPtr<const Topology_TShape>(new Topology_TShape(geometry)), Topology_Orientation::Forward)
{
}

/// 几何内核

const Geometry_Shape& Topology_Shape::geometry() const
{
    return tShape().geometry();
}

/// 内部访问

const Topology_TShape& Topology_Shape::tShape() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid Topology_Shape.");
    return *static_cast<const Topology_TShape*>(tObject().get());
}

}