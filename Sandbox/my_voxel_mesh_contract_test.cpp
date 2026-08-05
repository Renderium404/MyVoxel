#include <cassert>
#include <cmath>

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Mesh/MeshQuery.h"

namespace
{

MyVoxel::Geometry::Mesh createColorlessCube()
{
    using MyVoxel::Geometry::Mesh;
    using MyVoxel::Geometry::MeshVertex;

    Mesh mesh;
    mesh.reserve(9, 36);

    const std::uint32_t v0 = mesh.appendVertex(MeshVertex(-1.0, -1.0, -1.0));
    const std::uint32_t v1 = mesh.appendVertex(MeshVertex( 1.0, -1.0, -1.0));
    const std::uint32_t v2 = mesh.appendVertex(MeshVertex( 1.0,  1.0, -1.0));
    const std::uint32_t v3 = mesh.appendVertex(MeshVertex(-1.0,  1.0, -1.0));
    const std::uint32_t v4 = mesh.appendVertex(MeshVertex(-1.0, -1.0,  1.0));
    const std::uint32_t v5 = mesh.appendVertex(MeshVertex( 1.0, -1.0,  1.0));
    const std::uint32_t v6 = mesh.appendVertex(MeshVertex( 1.0,  1.0,  1.0));
    const std::uint32_t v7 = mesh.appendVertex(MeshVertex(-1.0,  1.0,  1.0));

    // 该未引用顶点用于验证几何包围盒只由实际三角形决定。
    mesh.appendVertex(MeshVertex(1000.0, 1000.0, 1000.0));

    mesh.appendTriangle(v0, v2, v1);
    mesh.appendTriangle(v0, v3, v2);

    mesh.appendTriangle(v4, v5, v6);
    mesh.appendTriangle(v4, v6, v7);

    mesh.appendTriangle(v0, v1, v5);
    mesh.appendTriangle(v0, v5, v4);

    mesh.appendTriangle(v3, v7, v6);
    mesh.appendTriangle(v3, v6, v2);

    mesh.appendTriangle(v0, v4, v7);
    mesh.appendTriangle(v0, v7, v3);

    mesh.appendTriangle(v1, v2, v6);
    mesh.appendTriangle(v1, v6, v5);

    return mesh;
}

bool almostEqual(double first, double second, double epsilon = 1.0e-12)
{
    return std::fabs(first - second) <= epsilon;
}

}

int main()
{
    using MyMath::Vector3;
    using MyVoxel::Bounds3;
    using MyVoxel::Geometry::Mesh;
    using MyVoxel::Geometry::MeshQuery;

    const Mesh mesh = createColorlessCube();

    assert(mesh.isGeometryValid());
    assert(mesh.isValid());
    assert(!mesh.isRenderable());
    assert(!mesh.hasValidNormals());
    assert(!mesh.hasTriangleColors());
    assert(!mesh.hasDegenerateTriangles());

    const Bounds3 bounds = mesh.localBounds();

    assert(almostEqual(bounds.minimum().x(), -1.0));
    assert(almostEqual(bounds.minimum().y(), -1.0));
    assert(almostEqual(bounds.minimum().z(), -1.0));
    assert(almostEqual(bounds.maximum().x(), 1.0));
    assert(almostEqual(bounds.maximum().y(), 1.0));
    assert(almostEqual(bounds.maximum().z(), 1.0));

    const MeshQuery query(mesh);

    assert(query.isValid());
    assert(query.triangleCount() == 12);
    assert(query.containsPoint(Vector3(0.0, 0.0, 0.0)));
    assert(query.containsPoint(Vector3(1.0, 0.0, 0.0)));
    assert(!query.containsPoint(Vector3(2.0, 0.0, 0.0)));

    assert(almostEqual(
        query.distanceToPoint(Vector3(2.0, 0.0, 0.0)),
        1.0,
        1.0e-6));

    return 0;
}