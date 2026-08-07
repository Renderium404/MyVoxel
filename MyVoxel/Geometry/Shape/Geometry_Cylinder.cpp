#include "Geometry_Cylinder.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double HalfScale = 0.5; // 完整高度转换为半高度使用的固定比例。

// 判断数值是否为有限正数。
bool isFinitePositive(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity && value > 0.0;
}

// 判断半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
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


Geometry_Cylinder::Geometry_Cylinder(double radius, double height)
    : Geometry_Shape(Bounds3(MyMath::Vector3(-radius, -radius, -height * HalfScale),
                             MyMath::Vector3(radius, radius, height * HalfScale)))
    , m_radius(radius)
    , m_radiusSquared(radius * radius)
    , m_height(height)
    , m_halfHeight(height * HalfScale)
{
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(radius), "Geometry_Cylinder radius must be finite and greater than zero.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(height), "Geometry_Cylinder height must be finite and greater than zero.");
}

/// 几何参数

double Geometry_Cylinder::radius() const
{
    return m_radius;
}

double Geometry_Cylinder::height() const
{
    return m_height;
}

/// 几何属性

ShapeKind Geometry_Cylinder::kind() const
{
    return ShapeKind::Cylinder;
}


/// 标准空间查询

bool Geometry_Cylinder::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Cylinder query point must be finite.");

    if (point.z() < -m_halfHeight || point.z() > m_halfHeight)
    {
        return false;
    }

    const double radialDistanceSquared = point.x() * point.x() + point.y() * point.y();
    return radialDistanceSquared <= m_radiusSquared;
}

ShapeRelation Geometry_Cylinder::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Cylinder query bounds must be valid.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    if (maximum.z() < -m_halfHeight || minimum.z() > m_halfHeight)
    {
        return ShapeRelation::Outside;
    }

    const double nearestX = distanceToInterval(minimum.x(), maximum.x());
    const double nearestY = distanceToInterval(minimum.y(), maximum.y());
    const double minimumRadialDistanceSquared = nearestX * nearestX + nearestY * nearestY;

    if (minimumRadialDistanceSquared > m_radiusSquared)
    {
        return ShapeRelation::Outside;
    }

    const double maximumRadialDistanceSquared =
        maximumSquaredDistanceToInterval(minimum.x(), maximum.x()) +
        maximumSquaredDistanceToInterval(minimum.y(), maximum.y());

    const bool strictlyInsideHeight = minimum.z() > -m_halfHeight && maximum.z() < m_halfHeight;

    if (strictlyInsideHeight && maximumRadialDistanceSquared < m_radiusSquared)
    {
        return ShapeRelation::Inside;
    }

    return ShapeRelation::Intersecting;
}




}