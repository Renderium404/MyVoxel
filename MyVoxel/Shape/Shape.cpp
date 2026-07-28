#include "Shape.h"

#include <cassert>

namespace MyVoxel
{

Shape::Shape(const std::shared_ptr<const ShapeGeometry>& geometry)
    : m_geometry(geometry)
    , m_transform(MyMath::Matrix4::identity())
{
    assert(m_geometry);
}

Shape::Shape(const std::shared_ptr<const ShapeGeometry>& geometry, const MyMath::Matrix4& transform)
    : m_geometry(geometry)
    , m_transform(transform)
{
    assert(m_geometry);
    assert(transform.isAffine());
}

const ShapeGeometry& Shape::geometry() const
{
    assert(m_geometry);
    return *m_geometry;
}

const std::shared_ptr<const ShapeGeometry>& Shape::geometryHandle() const
{
    assert(m_geometry);
    return m_geometry;
}

bool Shape::sharesGeometryWith(const Shape& other) const
{
    return m_geometry == other.m_geometry;
}

ShapeBounds Shape::localBounds() const
{
    assert(m_geometry);
    return m_geometry->localBounds();
}

bool Shape::containsLocalPoint(const MyMath::Vector3& point) const
{
    assert(m_geometry);
    assert(point.isFinite());
    return m_geometry->containsLocalPoint(point);
}

ShapeRegionRelation Shape::classifyLocalBounds(const ShapeBounds& bounds) const
{
    assert(m_geometry);
    assert(bounds.isValid());
    return m_geometry->classifyLocalBounds(bounds);
}

const MyMath::Matrix4& Shape::transform() const
{
    return m_transform;
}

void Shape::setTransform(const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());
    m_transform = transform;
}

void Shape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

}