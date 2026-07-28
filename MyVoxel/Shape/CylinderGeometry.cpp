#include "CylinderGeometry.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace
{

// 返回指定数值到闭区间的最短距离。
double distanceToInterval(double value, double minimum, double maximum)
{
    if (value < minimum)
    {
        return minimum - value;
    }

    if (value > maximum)
    {
        return value - maximum;
    }

    return 0.0;
}

}

namespace MyVoxel
{

CylinderGeometry::CylinderGeometry(double centerX, double centerY, double minimumZ, double maximumZ, double radius)
    : m_centerX(centerX)
    , m_centerY(centerY)
    , m_minimumZ(minimumZ)
    , m_maximumZ(maximumZ)
    , m_radius(radius)
    , m_radiusSquared(radius * radius)
{
    assert(std::isfinite(centerX) && std::isfinite(centerY));
    assert(std::isfinite(minimumZ) && std::isfinite(maximumZ));
    assert(std::isfinite(radius) && radius > 0.0);
    assert(minimumZ < maximumZ);
}

double CylinderGeometry::centerX() const
{
    return m_centerX;
}

double CylinderGeometry::centerY() const
{
    return m_centerY;
}

double CylinderGeometry::minimumZ() const
{
    return m_minimumZ;
}

double CylinderGeometry::maximumZ() const
{
    return m_maximumZ;
}

double CylinderGeometry::radius() const
{
    return m_radius;
}

ShapeBounds CylinderGeometry::localBounds() const
{
    return ShapeBounds(m_centerX - m_radius, m_centerY - m_radius, m_minimumZ, m_centerX + m_radius, m_centerY + m_radius, m_maximumZ);
}

bool CylinderGeometry::containsLocalPoint(const MyMath::Vector3& point) const
{
    assert(point.isFinite());

    if (point.z() < m_minimumZ || point.z() >= m_maximumZ)
    {
        return false;
    }

    const double offsetX = point.x() - m_centerX;
    const double offsetY = point.y() - m_centerY;
    return offsetX * offsetX + offsetY * offsetY <= m_radiusSquared;
}

ShapeRegionRelation CylinderGeometry::classifyLocalBounds(const ShapeBounds& bounds) const
{
    assert(bounds.isValid());

    if (bounds.maximumZ <= m_minimumZ || bounds.minimumZ >= m_maximumZ)
    {
        return ShapeRegionRelation::Outside;
    }

    const double nearestX = distanceToInterval(m_centerX, bounds.minimumX, bounds.maximumX);
    const double nearestY = distanceToInterval(m_centerY, bounds.minimumY, bounds.maximumY);
    const double minimumDistanceSquared = nearestX * nearestX + nearestY * nearestY;

    if (minimumDistanceSquared > m_radiusSquared)
    {
        return ShapeRegionRelation::Outside;
    }

    const double farthestX = (std::max)(std::fabs(bounds.minimumX - m_centerX), std::fabs(bounds.maximumX - m_centerX));
    const double farthestY = (std::max)(std::fabs(bounds.minimumY - m_centerY), std::fabs(bounds.maximumY - m_centerY));
    const double maximumDistanceSquared = farthestX * farthestX + farthestY * farthestY;
    const bool insideHeight = bounds.minimumZ >= m_minimumZ && bounds.maximumZ <= m_maximumZ;

    if (insideHeight && maximumDistanceSquared <= m_radiusSquared)
    {
        return ShapeRegionRelation::Inside;
    }

    return ShapeRegionRelation::Intersecting;
}

}