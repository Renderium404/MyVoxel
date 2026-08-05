#include "Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

}

namespace MyVoxel
{
namespace Geometry
{

Shape::Shape()
{
}

Shape::Shape(const Foundation::RefPtr<const ShapeGeometry>& geometry)
    : m_geometry(geometry)
{
    MYVOXEL_ASSERT_MESSAGE(m_geometry, "Shape geometry must not be null.");
}

/// 状态判断

bool Shape::isValid() const
{
    return static_cast<bool>(m_geometry);
}

bool Shape::sharesGeometryWith(const Shape& other) const
{
    return m_geometry && m_geometry == other.m_geometry;
}

/// 几何访问

const ShapeGeometry& Shape::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Shape.");
    return *m_geometry;
}

const ShapeGeometry* Shape::geometryPointer() const
{
    return m_geometry.get();
}

ShapeKind Shape::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Shape.");
    return m_geometry->kind();
}

Bounds3 Shape::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Shape.");

    const Bounds3 bounds = m_geometry->localBounds();

    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "ShapeGeometry must return valid local bounds.");
    MYVOXEL_ASSERT_MESSAGE(bounds.hasVolume(), "ShapeGeometry local bounds must have volume.");
    return bounds;
}

/// 标准空间查询

bool Shape::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Shape query point must be finite.");
    return m_geometry->containsLocalPoint(point);
}

ShapeRelation Shape::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Shape query bounds must be valid.");
    return m_geometry->classifyLocalBounds(bounds);
}

/// 快速空间查询

ShapeRelation Shape::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "Shape query bounds center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "Shape query bounds extent must be finite and non-negative.");
    return m_geometry->classifyLocalBoundsFast(center, extent);
}

}
}