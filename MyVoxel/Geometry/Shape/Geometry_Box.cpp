#include "Geometry_Box.h"

#include <algorithm>
#include <cmath>
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
    : Geometry_Shape(Bounds3(MyMath::Vector3(-sizeX * HalfScale, -sizeY * HalfScale, -sizeZ * HalfScale),
                             MyMath::Vector3(sizeX * HalfScale, sizeY * HalfScale, sizeZ * HalfScale)))
    , m_sizeX(sizeX)
    , m_sizeY(sizeY)
    , m_sizeZ(sizeZ)
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


/// 空间查询

bool Geometry_Box::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Box query point must be finite.");
    return localBounds().contains(point);
}

bool Geometry_Box::supportsSignedDistance() const
{
    return true;
}

double Geometry_Box::signedDistanceLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Box signed-distance query point must be finite.");

    const double halfX = m_sizeX * HalfScale;
    const double halfY = m_sizeY * HalfScale;
    const double halfZ = m_sizeZ * HalfScale;
    const double deltaX = std::fabs(point.x()) - halfX;
    const double deltaY = std::fabs(point.y()) - halfY;
    const double deltaZ = std::fabs(point.z()) - halfZ;
    const double outsideX = (std::max)(deltaX, 0.0);
    const double outsideY = (std::max)(deltaY, 0.0);
    const double outsideZ = (std::max)(deltaZ, 0.0);
    const double outsideDistance = std::sqrt(outsideX * outsideX + outsideY * outsideY + outsideZ * outsideZ);
    const double insideDistance = (std::min)((std::max)(deltaX, (std::max)(deltaY, deltaZ)), 0.0);
    return outsideDistance + insideDistance;
}

ShapeRelation Geometry_Box::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Box classification bounds must be valid.");

    const Bounds3& boxBounds = localBounds();
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
