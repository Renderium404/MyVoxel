#include "Geometry_Sphere.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断数值是否为有限正数。
bool isFinitePositive(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity && value > 0.0;
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


Geometry_Sphere::Geometry_Sphere(double radius)
    : Geometry_Shape(Bounds3(MyMath::Vector3(-radius, -radius, -radius),
                             MyMath::Vector3(radius, radius, radius)))
    , m_radius(radius)
    , m_radiusSquared(radius * radius)
{
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(radius), "Geometry_Sphere radius must be finite and greater than zero.");
}

/// 几何参数

double Geometry_Sphere::radius() const
{
    return m_radius;
}

/// 几何属性

ShapeKind Geometry_Sphere::kind() const
{
    return ShapeKind::Sphere;
}


/// 标准空间查询

bool Geometry_Sphere::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Sphere query point must be finite.");

    const double distanceSquared = point.x() * point.x() + point.y() * point.y() + point.z() * point.z();
    return distanceSquared <= m_radiusSquared;
}

bool Geometry_Sphere::supportsSignedDistance() const
{
    return true;
}

double Geometry_Sphere::signedDistanceLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Sphere signed-distance query point must be finite.");
    return std::sqrt(point.x() * point.x() + point.y() * point.y() + point.z() * point.z()) - m_radius;
}

ShapeRelation Geometry_Sphere::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Sphere query bounds must be valid.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    const double nearestX = distanceToInterval(minimum.x(), maximum.x());
    const double nearestY = distanceToInterval(minimum.y(), maximum.y());
    const double nearestZ = distanceToInterval(minimum.z(), maximum.z());
    const double minimumDistanceSquared = nearestX * nearestX + nearestY * nearestY + nearestZ * nearestZ;

    if (minimumDistanceSquared > m_radiusSquared)
    {
        return ShapeRelation::Outside;
    }

    const double maximumDistanceSquared =
        maximumSquaredDistanceToInterval(minimum.x(), maximum.x()) +
        maximumSquaredDistanceToInterval(minimum.y(), maximum.y()) +
        maximumSquaredDistanceToInterval(minimum.z(), maximum.z());

    if (maximumDistanceSquared < m_radiusSquared)
    {
        return ShapeRelation::Inside;
    }

    return ShapeRelation::Intersecting;
}


}
