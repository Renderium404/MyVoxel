#include "Topology_Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断包围盒半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

// 判断边界Face集合是否非空且全部有效。
bool areValidFaces(const std::vector<MyVoxel::Topology_Face>& faces)
{
    if (faces.empty())
    {
        return false;
    }

    for (std::size_t index = 0; index < faces.size(); ++index)
    {
        if (!faces[index].isValid())
        {
            return false;
        }
    }

    return true;
}

}

namespace MyVoxel
{

Topology_Shape::Topology_Shape()
{
}

Topology_Shape::Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_Shape geometry must not be null.");

    if (!geometry)
    {
        return;
    }

    const Bounds3& bounds = geometry->localBounds();

    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Shape must return valid local bounds.");
    MYVOXEL_ASSERT_MESSAGE(bounds.hasVolume(), "Geometry_Shape local bounds must have volume.");

    if (!bounds.isValid() || !bounds.hasVolume())
    {
        return;
    }

    m_geometry = geometry;
}

Topology_Shape::Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry, const std::vector<Topology_Face>& faces)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_Shape geometry must not be null.");
    MYVOXEL_ASSERT_MESSAGE(areValidFaces(faces), "Topology_Shape boundary faces must be non-empty and valid.");

    if (!geometry || !areValidFaces(faces))
    {
        return;
    }

    const Bounds3& bounds = geometry->localBounds();

    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Shape must return valid local bounds.");
    MYVOXEL_ASSERT_MESSAGE(bounds.hasVolume(), "Geometry_Shape local bounds must have volume.");

    if (!bounds.isValid() || !bounds.hasVolume())
    {
        return;
    }

    m_geometry = geometry;
    m_faces = faces;
}

/// 状态判断

bool Topology_Shape::isValid() const
{
    return static_cast<bool>(m_geometry);
}

bool Topology_Shape::isNull() const
{
    return !m_geometry;
}

Topology_Shape::operator bool() const
{
    return isValid();
}

bool Topology_Shape::sharesGeometryWith(const Topology_Shape& other) const
{
    return m_geometry && m_geometry == other.m_geometry;
}

bool Topology_Shape::hasFaces() const
{
    return isValid() && !m_faces.empty();
}

/// 几何资源

const Geometry_Shape& Topology_Shape::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Topology_Shape.");
    return *m_geometry;
}

const Geometry_Shape* Topology_Shape::geometryPointer() const
{
    return m_geometry.get();
}

const Foundation::RefPtr<const Geometry_Shape>& Topology_Shape::geometryResource() const
{
    return m_geometry;
}

ShapeKind Topology_Shape::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Topology_Shape.");
    return m_geometry->kind();
}

/// 边界拓扑

std::size_t Topology_Shape::faceCount() const
{
    return m_faces.size();
}

const Topology_Face& Topology_Shape::face(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access a face of an invalid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(index < m_faces.size(), "Topology_Shape face index is out of range.");
    return m_faces[index];
}

const std::vector<Topology_Face>& Topology_Shape::faces() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access faces of an invalid Topology_Shape.");
    return m_faces;
}

/// 局部空间数据与查询

const Bounds3& Topology_Shape::bounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the bounds of an invalid Topology_Shape.");
    return m_geometry->localBounds();
}

bool Topology_Shape::containsPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Topology_Shape query point must be finite.");
    return m_geometry->containsLocalPoint(point);
}

ShapeRelation Topology_Shape::classifyBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Topology_Shape query bounds must be valid.");
    return m_geometry->classifyLocalBounds(bounds);
}

ShapeRelation Topology_Shape::classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "Topology_Shape query bounds center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "Topology_Shape query bounds extent must be finite and non-negative.");
    return m_geometry->classifyLocalBoundsFast(center, extent);
}

}