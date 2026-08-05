#include "MeshModeling.h"

#include <utility>

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Mesh/MeshGeometry.h"
#include "MyVoxel/Geometry/ShapeGeometry.h"

namespace MyVoxel
{
namespace Modeling
{

/// 三角网格几何创建

Geometry::Shape makeMesh(const Geometry::Mesh& mesh)
{
    const Foundation::RefPtr<Geometry::MeshGeometry> geometry = Foundation::makeRef<Geometry::MeshGeometry>(mesh);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

Geometry::Shape makeMesh(Geometry::Mesh&& mesh)
{
    const Foundation::RefPtr<Geometry::MeshGeometry> geometry =
        Foundation::makeRef<Geometry::MeshGeometry>(std::move(mesh));

    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

}
}
