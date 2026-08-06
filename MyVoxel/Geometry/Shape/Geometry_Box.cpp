#include "Geometry_Box.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double HalfScale = 0.5; // 完整尺寸转换为半尺寸使用的固定比例。

// 判断数值是否为有限正数。
bool isFinitePositive(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity && value > 0.0;
}

// 判断指定包围盒是否严格位于另一个包围盒内部。
bool strictlyContains(const MyVoxel::Bounds3& outer, const MyVoxel::Bounds3& inner)
{
    return inner.minimum().x() > outer.minimum().x() &&
           inner.maximum().x() < outer.maximum().x() &&
           inner.minimum().y() > outer.minimum().y() &&
           inner.maximum().y() < outer.maximum().y() &&
           inner.minimum().z() > outer.minimum().z() &&
           inner.maximum().z() < outer.maximum().z();
}

}

namespace MyVoxel
{


Geometry_Box::Geometry_Box(double sizeX, double sizeY, double sizeZ)
    : m_sizeX(sizeX)
    , m_sizeY(sizeY)
    , m_sizeZ(sizeZ)
    , m_bounds(MyMath::Vector3(-sizeX * HalfScale, -sizeY * HalfScale, -sizeZ * HalfScale),
               MyMath::Vector3(sizeX * HalfScale, sizeY * HalfScale, sizeZ * HalfScale))
{
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(sizeX), "Geometry_Box sizeX must be finite and greater than zero.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(sizeY), "Geometry_Box sizeY must be finite and greater than zero.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(sizeZ), "Geometry_Box sizeZ must be finite and greater than zero.");
}

/// 几何参数

double Geometry_Box::sizeX() const
{
    return m_sizeX;
}

double Geometry_Box::sizeY() const
{
    return m_sizeY;
}

double Geometry_Box::sizeZ() const
{
    return m_sizeZ;
}

/// 几何属性

ShapeKind Geometry_Box::kind() const
{
    return ShapeKind::Box;
}

Bounds3 Geometry_Box::localBounds() const
{
    return m_bounds;
}

/// 空间查询

bool Geometry_Box::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Box query point must be finite.");
    return m_bounds.contains(point);
}

ShapeRelation Geometry_Box::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Box classification bounds must be valid.");

    const Bounds3 boxBounds = localBounds();
    const MyMath::Vector3& boxMinimum = boxBounds.minimum();
    const MyMath::Vector3& boxMaximum = boxBounds.maximum();
    const MyMath::Vector3& boundsMinimum = bounds.minimum();
    const MyMath::Vector3& boundsMaximum = bounds.maximum();

    if (boundsMaximum.x() < boxMinimum.x() || boundsMinimum.x() > boxMaximum.x() ||
        boundsMaximum.y() < boxMinimum.y() || boundsMinimum.y() > boxMaximum.y() ||
        boundsMaximum.z() < boxMinimum.z() || boundsMinimum.z() > boxMaximum.z())
    {
        return ShapeRelation::Outside;
    }

    if (boundsMinimum.x() >= boxMinimum.x() && boundsMaximum.x() <= boxMaximum.x() &&
        boundsMinimum.y() >= boxMinimum.y() && boundsMaximum.y() <= boxMaximum.y() &&
        boundsMinimum.z() >= boxMinimum.z() && boundsMaximum.z() <= boxMaximum.z())
    {
        return ShapeRelation::Inside;
    }

    return ShapeRelation::Intersecting;
}


}