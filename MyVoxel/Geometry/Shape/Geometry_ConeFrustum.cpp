#include "Geometry_ConeFrustum.h"

#include <algorithm>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double HalfScale = 0.5; // 完整高度转换为半高度使用的固定比例。

// 判断数值是否为有限数值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断数值是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 判断数值是否为有限正数。
bool isFinitePositive(double value)
{
    return isFiniteValue(value) && value > 0.0;
}

// 返回坐标原点到闭区间的最短距离。
double distanceToInterval(double minimum, double maximum)
{
    if (minimum > 0.0)
    {
        return minimum;
    }

    if (maximum < 0.0)
    {
        return -maximum;
    }

    return 0.0;
}

// 返回坐标原点到闭区间端点的最大距离平方。
double maximumSquaredDistanceToInterval(double minimum, double maximum)
{
    return (std::max)(minimum * minimum, maximum * maximum);
}

}

namespace MyVoxel
{


Geometry_ConeFrustum::Geometry_ConeFrustum(double bottomRadius, double topRadius, double height)
    : m_bottomRadius(bottomRadius)
    , m_topRadius(topRadius)
    , m_height(height)
    , m_halfHeight(height * HalfScale)
    , m_bounds(MyMath::Vector3(-(std::max)(bottomRadius, topRadius),
                               -(std::max)(bottomRadius, topRadius),
                               -height * HalfScale),
               MyMath::Vector3((std::max)(bottomRadius, topRadius),
                               (std::max)(bottomRadius, topRadius),
                               height * HalfScale))
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(bottomRadius), "Geometry_ConeFrustum bottom radius must be finite and non-negative.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(topRadius), "Geometry_ConeFrustum top radius must be finite and non-negative.");
    MYVOXEL_ASSERT_MESSAGE(bottomRadius > 0.0 || topRadius > 0.0, "Geometry_ConeFrustum requires at least one radius greater than zero.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(height), "Geometry_ConeFrustum height must be finite and greater than zero.");
}

/// 几何参数

double Geometry_ConeFrustum::bottomRadius() const
{
    return m_bottomRadius;
}

double Geometry_ConeFrustum::topRadius() const
{
    return m_topRadius;
}

double Geometry_ConeFrustum::height() const
{
    return m_height;
}

double Geometry_ConeFrustum::radiusAt(double localZ) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(localZ), "Geometry_ConeFrustum localZ must be finite.");
    MYVOXEL_ASSERT_MESSAGE(localZ >= -m_halfHeight && localZ <= m_halfHeight, "Geometry_ConeFrustum localZ is outside the geometry height range.");

    const double ratio = (localZ + m_halfHeight) / m_height;
    return m_bottomRadius + (m_topRadius - m_bottomRadius) * ratio;
}

/// 几何属性

ShapeKind Geometry_ConeFrustum::kind() const
{
    return ShapeKind::ConeFrustum;
}

Bounds3 Geometry_ConeFrustum::localBounds() const
{
    return m_bounds;
}

/// 空间查询

bool Geometry_ConeFrustum::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_ConeFrustum query point must be finite.");

    if (point.z() < -m_halfHeight || point.z() > m_halfHeight)
    {
        return false;
    }

    const double localRadius = radiusAt(point.z());
    const double radialDistanceSquared = point.x() * point.x() + point.y() * point.y();
    return radialDistanceSquared <= localRadius * localRadius;
}

ShapeRelation Geometry_ConeFrustum::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_ConeFrustum query bounds must be valid.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    if (maximum.z() < -m_halfHeight || minimum.z() > m_halfHeight)
    {
        return ShapeRelation::Outside;
    }

    const double overlapMinimumZ = (std::max)(minimum.z(), -m_halfHeight);
    const double overlapMaximumZ = (std::min)(maximum.z(), m_halfHeight);
    const double overlapMinimumRadius = radiusAt(overlapMinimumZ);
    const double overlapMaximumRadius = radiusAt(overlapMaximumZ);
    const double maximumOverlappingRadius = (std::max)(overlapMinimumRadius, overlapMaximumRadius);

    const double nearestX = distanceToInterval(minimum.x(), maximum.x());
    const double nearestY = distanceToInterval(minimum.y(), maximum.y());
    const double minimumRadialDistanceSquared = nearestX * nearestX + nearestY * nearestY;

    if (minimumRadialDistanceSquared > maximumOverlappingRadius * maximumOverlappingRadius)
    {
        return ShapeRelation::Outside;
    }

    const bool strictlyInsideHeight = minimum.z() > -m_halfHeight && maximum.z() < m_halfHeight;

    if (strictlyInsideHeight)
    {
        const double minimumRadius = (std::min)(radiusAt(minimum.z()), radiusAt(maximum.z()));
        const double maximumRadialDistanceSquared =
            maximumSquaredDistanceToInterval(minimum.x(), maximum.x()) +
            maximumSquaredDistanceToInterval(minimum.y(), maximum.y());

        if (maximumRadialDistanceSquared < minimumRadius * minimumRadius)
        {
            return ShapeRelation::Inside;
        }
    }

    return ShapeRelation::Intersecting;
}


}