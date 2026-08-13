#include "Geometry_Mesh.h"

#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Geometry_Mesh::Geometry_Mesh(const Mesh& mesh)
    : m_mesh(mesh)
    , m_query(m_mesh)
{
    MYVOXEL_ASSERT_MESSAGE(m_mesh.isValid(), "Geometry_Mesh requires a valid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!m_mesh.isEmpty(), "Geometry_Mesh requires a non-empty Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!m_mesh.hasDegenerateTriangles(), "Geometry_Mesh does not accept degenerate triangles.");
    MYVOXEL_ASSERT_MESSAGE(m_query.isValid(), "Geometry_Mesh failed to create valid query data.");
    setLocalBounds(m_query.queryBounds());
}

Geometry_Mesh::Geometry_Mesh(Mesh&& mesh)
    : m_mesh(std::move(mesh))
    , m_query(m_mesh)
{
    MYVOXEL_ASSERT_MESSAGE(m_mesh.isValid(), "Geometry_Mesh requires a valid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!m_mesh.isEmpty(), "Geometry_Mesh requires a non-empty Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!m_mesh.hasDegenerateTriangles(), "Geometry_Mesh does not accept degenerate triangles.");
    MYVOXEL_ASSERT_MESSAGE(m_query.isValid(), "Geometry_Mesh failed to create valid query data.");
    setLocalBounds(m_query.queryBounds());
}

/// 几何属性

ShapeKind Geometry_Mesh::kind() const
{
    return ShapeKind::Mesh;
}


const Mesh& Geometry_Mesh::mesh() const
{
    return m_mesh;
}

/// 标准空间查询

bool Geometry_Mesh::containsLocalPoint(const MyMath::Vector3& point) const
{
    return m_query.containsPoint(point);
}

ShapeRelation Geometry_Mesh::classifyLocalBounds(const Bounds3& bounds) const
{
    return m_query.classifyBounds(bounds);
}

/// 快速空间查询

ShapeRelation Geometry_Mesh::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    return m_query.classifyBounds(center, extent);
}

}
