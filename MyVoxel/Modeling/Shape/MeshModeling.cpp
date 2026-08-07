#include "MeshModeling.h"

#include <utility>

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Mesh/Geometry_Mesh.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

Topology_Shape createMesh(const Mesh& mesh)
{
    const Foundation::RefPtr<Geometry_Mesh> geometry = Foundation::makeRef<Geometry_Mesh>(mesh);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

Topology_Shape createMesh(Mesh&& mesh)
{
    const Foundation::RefPtr<Geometry_Mesh> geometry = Foundation::makeRef<Geometry_Mesh>(std::move(mesh));
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

/// 空间Shape实例创建

Shape makeMesh(const Mesh& mesh)
{
    return Shape(createMesh(mesh));
}

Shape makeMesh(Mesh&& mesh)
{
    return Shape(createMesh(std::move(mesh)));
}

Shape makeMesh(const Mesh& mesh, const MyMath::Matrix4& localToWorld)
{
    return Shape(createMesh(mesh), localToWorld);
}

Shape makeMesh(Mesh&& mesh, const MyMath::Matrix4& localToWorld)
{
    return Shape(createMesh(std::move(mesh)), localToWorld);
}

}
}
