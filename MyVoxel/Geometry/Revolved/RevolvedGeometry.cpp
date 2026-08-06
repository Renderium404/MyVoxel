#include "RevolvedGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

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

// 使用缩放计算二维向量长度，避免中间平方溢出。
double stableLength(double first, double second)
{
    const double absoluteFirst = std::fabs(first);
    const double absoluteSecond = std::fabs(second);
    const double scale = (std::max)(absoluteFirst, absoluteSecond);

    if (scale == 0.0)
    {
        return 0.0;
    }

    const double normalizedFirst = absoluteFirst / scale;
    const double normalizedSecond = absoluteSecond / scale;
    return scale * std::sqrt(normalizedFirst * normalizedFirst + normalizedSecond * normalizedSecond);
}

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

RevolvedGeometry::RevolvedGeometry(const CurveLoop& profile)
    : m_profile(profile)
    , m_radialSign(0.0)
{
    MYVOXEL_ASSERT_MESSAGE(profile.isValid(), "RevolvedGeometry requires a valid CurveLoop.");

    if (!profile.isValid())
    {
        return;
    }

    const double minimumX = profile.bounds().minimum().x();
    const double maximumX = profile.bounds().maximum().x();
    const double tolerance = profile.connectionTolerance();
    const bool onPositiveSide = minimumX >= -tolerance;
    const bool onNegativeSide = maximumX <= tolerance;

    MYVOXEL_ASSERT_MESSAGE(onPositiveSide || onNegativeSide, "RevolvedGeometry profile must remain on one side of the local Y axis.");

    if (!onPositiveSide && !onNegativeSide)
    {
        return;
    }

    m_radialSign = onPositiveSide ? 1.0 : -1.0;

    const double maximumRadius = (std::max)(std::fabs(minimumX), std::fabs(maximumX));
    const double minimumZ = profile.bounds().minimum().y();
    const double maximumZ = profile.bounds().maximum().y();
    m_bounds = Bounds3(MyMath::Vector3(-maximumRadius, -maximumRadius, minimumZ), MyMath::Vector3(maximumRadius, maximumRadius, maximumZ));

    MYVOXEL_ASSERT_MESSAGE(m_bounds.hasVolume(), "RevolvedGeometry profile must produce a three-dimensional volume.");
}

/// 几何数据

const CurveLoop& RevolvedGeometry::profile() const
{
    return m_profile;
}

double RevolvedGeometry::radialSign() const
{
    return m_radialSign;
}

/// 几何属性

ShapeKind RevolvedGeometry::kind() const
{
    return ShapeKind::Revolved;
}

Bounds3 RevolvedGeometry::localBounds() const
{
    return m_bounds;
}

/// 空间查询

bool RevolvedGeometry::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "RevolvedGeometry query point must be finite.");

    const double radius = stableLength(point.x(), point.y());
    return m_profile.containsPoint(MyMath::Vector3(m_radialSign * radius, point.z(), 0.0));
}

ShapeRelation RevolvedGeometry::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "RevolvedGeometry query bounds must be valid.");
    return classifyRange(bounds.minimum(), bounds.maximum());
}

/// 快速空间查询

ShapeRelation RevolvedGeometry::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "RevolvedGeometry query center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "RevolvedGeometry query extent must be finite and non-negative.");
    return classifyRange(center - extent, center + extent);
}

ShapeRelation RevolvedGeometry::classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const
{
    if (maximum.x() < m_bounds.minimum().x() || minimum.x() > m_bounds.maximum().x() || maximum.y() < m_bounds.minimum().y() || minimum.y() > m_bounds.maximum().y() || maximum.z() < m_bounds.minimum().z() || minimum.z() > m_bounds.maximum().z())
    {
        return ShapeRelation::Outside;
    }

    const double nearestX = distanceToInterval(minimum.x(), maximum.x());
    const double nearestY = distanceToInterval(minimum.y(), maximum.y());
    const double farthestX = (std::max)(std::fabs(minimum.x()), std::fabs(maximum.x()));
    const double farthestY = (std::max)(std::fabs(minimum.y()), std::fabs(maximum.y()));
    const double minimumRadius = stableLength(nearestX, nearestY);
    const double maximumRadius = stableLength(farthestX, farthestY);

    const double profileMinimumX = m_radialSign > 0.0 ? minimumRadius : -maximumRadius;
    const double profileMaximumX = m_radialSign > 0.0 ? maximumRadius : -minimumRadius;
    const Bounds3 profileBounds(MyMath::Vector3(profileMinimumX, minimum.z(), 0.0), MyMath::Vector3(profileMaximumX, maximum.z(), 0.0));
    const ShapeRelation relation = m_profile.classifyBounds(profileBounds);

    if (relation != ShapeRelation::Intersecting || minimumRadius > 0.0)
    {
        return relation;
    }

    double coordinateScale = 1.0;
    coordinateScale = (std::max)(coordinateScale, maximumRadius);
    coordinateScale = (std::max)(coordinateScale, std::fabs(minimum.z()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(maximum.z()));

    const double axisOffset = (std::max)(m_profile.connectionTolerance(), coordinateScale * std::numeric_limits<double>::epsilon() * 64.0); // 旋转轴上的轮廓边退化为轴线，使用极小径向探针区分真实表面相交。
    const double probeMinimumX = m_radialSign > 0.0 ? axisOffset : -maximumRadius;
    const double probeMaximumX = m_radialSign > 0.0 ? maximumRadius : -axisOffset;
    const Bounds3 probeBounds(MyMath::Vector3((std::min)(probeMinimumX, probeMaximumX), minimum.z(), 0.0), MyMath::Vector3((std::max)(probeMinimumX, probeMaximumX), maximum.z(), 0.0));

    return m_profile.classifyBounds(probeBounds) == ShapeRelation::Inside ? ShapeRelation::Inside : relation;
}

}
}
