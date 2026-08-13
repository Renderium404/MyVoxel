#include "Geometry_Shape.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Geometry_Shape::Geometry_Shape()
{
}

Geometry_Shape::Geometry_Shape(const Bounds3& localBounds)
    : m_localBounds(localBounds)
{
}

/// 几何属性

const Bounds3& Geometry_Shape::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(m_localBounds.isValid(), "Geometry_Shape local bounds have not been initialized.");
    return m_localBounds;
}

/// 标准空间查询

bool Geometry_Shape::supportsSignedDistance() const
{
    return false;
}

double Geometry_Shape::signedDistanceLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Shape signed-distance query point must be finite.");
    MYVOXEL_ASSERT_MESSAGE(false, "Current Geometry_Shape does not provide signed-distance queries.");
    return (std::numeric_limits<double>::infinity)();
}

/// 派生类缓存维护

void Geometry_Shape::setLocalBounds(const Bounds3& localBounds)
{
    MYVOXEL_ASSERT_MESSAGE(localBounds.isValid(), "Geometry_Shape local bounds must be valid.");
    m_localBounds = localBounds;
}

void Geometry_Shape::clearLocalBounds()
{
    m_localBounds.clear();
}

}
