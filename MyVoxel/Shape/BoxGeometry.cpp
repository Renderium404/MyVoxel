#include "BoxGeometry.h"

#include <cassert>

namespace MyVoxel
{

BoxGeometry::BoxGeometry(double minimumX, double minimumY, double minimumZ, double maximumX, double maximumY, double maximumZ)
    : m_bounds(minimumX, minimumY, minimumZ, maximumX, maximumY, maximumZ)
{
    assert(m_bounds.hasVolume());
}

double BoxGeometry::minimumX() const
{
    return m_bounds.minimumX;
}

double BoxGeometry::minimumY() const
{
    return m_bounds.minimumY;
}

double BoxGeometry::minimumZ() const
{
    return m_bounds.minimumZ;
}

double BoxGeometry::maximumX() const
{
    return m_bounds.maximumX;
}

double BoxGeometry::maximumY() const
{
    return m_bounds.maximumY;
}

double BoxGeometry::maximumZ() const
{
    return m_bounds.maximumZ;
}

ShapeBounds BoxGeometry::localBounds() const
{
    return m_bounds;
}

bool BoxGeometry::containsLocalPoint(const MyMath::Vector3& point) const
{
    assert(point.isFinite());

    return point.x() >= m_bounds.minimumX && point.x() < m_bounds.maximumX &&
           point.y() >= m_bounds.minimumY && point.y() < m_bounds.maximumY &&
           point.z() >= m_bounds.minimumZ && point.z() < m_bounds.maximumZ;
}

ShapeRegionRelation BoxGeometry::classifyLocalBounds(const ShapeBounds& bounds) const
{
    assert(bounds.isValid());

    if (bounds.maximumX <= m_bounds.minimumX || bounds.minimumX >= m_bounds.maximumX ||
        bounds.maximumY <= m_bounds.minimumY || bounds.minimumY >= m_bounds.maximumY ||
        bounds.maximumZ <= m_bounds.minimumZ || bounds.minimumZ >= m_bounds.maximumZ)
    {
        return ShapeRegionRelation::Outside;
    }

    if (bounds.minimumX >= m_bounds.minimumX && bounds.maximumX <= m_bounds.maximumX &&
        bounds.minimumY >= m_bounds.minimumY && bounds.maximumY <= m_bounds.maximumY &&
        bounds.minimumZ >= m_bounds.minimumZ && bounds.maximumZ <= m_bounds.maximumZ)
    {
        return ShapeRegionRelation::Inside;
    }

    return ShapeRegionRelation::Intersecting;
}

}