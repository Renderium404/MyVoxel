#include "MeshGeometry.h"

#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{
namespace Geometry
{

MeshGeometry::MeshGeometry(const Mesh& mesh)
    : m_mesh(mesh)
    , m_query(m_mesh)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_mesh.isValid(),
        "MeshGeometry requires a valid Geometry::Mesh.");

    MYVOXEL_ASSERT_MESSAGE(
        !m_mesh.isEmpty(),
        "MeshGeometry requires a non-empty Geometry::Mesh.");

    MYVOXEL_ASSERT_MESSAGE(
        !m_mesh.hasDegenerateTriangles(),
        "MeshGeometry does not accept degenerate triangles.");

    MYVOXEL_ASSERT_MESSAGE(
        m_query.isValid(),
        "MeshGeometry failed to create valid query data.");
}

MeshGeometry::MeshGeometry(Mesh&& mesh)
    : m_mesh(std::move(mesh))
    , m_query(m_mesh)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_mesh.isValid(),
        "MeshGeometry requires a valid Geometry::Mesh.");

    MYVOXEL_ASSERT_MESSAGE(
        !m_mesh.isEmpty(),
        "MeshGeometry requires a non-empty Geometry::Mesh.");

    MYVOXEL_ASSERT_MESSAGE(
        !m_mesh.hasDegenerateTriangles(),
        "MeshGeometry does not accept degenerate triangles.");

    MYVOXEL_ASSERT_MESSAGE(
        m_query.isValid(),
        "MeshGeometry failed to create valid query data.");
}

/// 几何属性

ShapeKind MeshGeometry::kind() const
{
    return ShapeKind::Mesh;
}

Bounds3 MeshGeometry::localBounds() const
{
    return m_query.queryBounds();
}

const Mesh& MeshGeometry::mesh() const
{
    return m_mesh;
}

/// 标准空间查询

bool MeshGeometry::containsLocalPoint(const MyMath::Vector3& point) const
{
    return m_query.containsPoint(point);
}

ShapeRelation MeshGeometry::classifyLocalBounds(const Bounds3& bounds) const
{
    return m_query.classifyBounds(bounds);
}

/// 快速空间查询

ShapeRelation MeshGeometry::classifyLocalBoundsFast(
    const MyMath::Vector3& center,
    const MyMath::Vector3& extent) const
{
    return m_query.classifyBounds(center, extent);
}

}
}