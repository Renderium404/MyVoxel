#include "Geometry_Revolved.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆弧交点计算统一使用弧度制。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断点是否为局部XY平面中的有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

// 判断半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

// 将数值限制到指定闭区间。
double clampValue(double value, double minimum, double maximum)
{
    return (std::max)(minimum, (std::min)(maximum, value));
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

// 返回点到局部XY平面直线段的最短距离平方。
double pointLineDistanceSquared(const MyMath::Vector3& point, const MyVoxel::Geometry_Line& line)
{
    const MyMath::Vector3 segment = line.endPoint() - line.startPoint();
    const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();
    const MyMath::Vector3 relative = point - line.startPoint();
    const double parameter = clampValue((relative.x() * segment.x() + relative.y() * segment.y()) / lengthSquared, 0.0, 1.0);
    const MyMath::Vector3 closest = line.startPoint() + segment * parameter;
    const double deltaX = point.x() - closest.x();
    const double deltaY = point.y() - closest.y();
    return deltaX * deltaX + deltaY * deltaY;
}

// 判断点是否位于局部XY平面圆弧边界上。
bool pointOnArc(const MyMath::Vector3& point, const MyVoxel::Geometry_Arc& arc, double tolerance)
{
    const double deltaX = point.x() - arc.center().x();
    const double deltaY = point.y() - arc.center().y();
    const double distance = stableLength(deltaX, deltaY);

    if (std::fabs(distance - arc.radius()) > tolerance)
    {
        return false;
    }

    const double angle = std::atan2(deltaY, deltaX);
    const double angleTolerance = tolerance > 0.0 ? tolerance / arc.radius() : 0.0;
    return arc.containsAngle(angle, angleTolerance);
}

// 判断点是否位于指定曲线几何边界上。
bool pointOnCurve(const MyMath::Vector3& point, const MyVoxel::Geometry_Curve& curve, double tolerance)
{
    if (curve.kind() == MyVoxel::CurveKind::Line)
    {
        return pointLineDistanceSquared(point, static_cast<const MyVoxel::Geometry_Line&>(curve)) <= tolerance * tolerance;
    }

    return pointOnArc(point, static_cast<const MyVoxel::Geometry_Arc&>(curve), tolerance);
}

// 返回直线段与向右水平射线的交点数量，查询Y已避开轮廓端点和极值。
unsigned int lineHorizontalRayIntersections(const MyMath::Vector3& point, double queryY, const MyVoxel::Geometry_Line& line, double tolerance)
{
    const double startY = line.startPoint().y();
    const double endY = line.endPoint().y();

    if ((startY > queryY) == (endY > queryY))
    {
        return 0;
    }

    const double parameter = (queryY - startY) / (endY - startY);
    const double intersectionX = line.startPoint().x() + (line.endPoint().x() - line.startPoint().x()) * parameter;
    return intersectionX > point.x() + tolerance ? 1U : 0U;
}

// 返回圆弧与向右水平射线的交点数量，查询Y已避开轮廓端点和极值。
unsigned int arcHorizontalRayIntersections(const MyMath::Vector3& point, double queryY, const MyVoxel::Geometry_Arc& arc, double tolerance)
{
    const double sine = (queryY - arc.center().y()) / arc.radius();

    if (sine < -1.0 || sine > 1.0)
    {
        return 0;
    }

    const double clampedSine = clampValue(sine, -1.0, 1.0);
    const double firstAngle = std::asin(clampedSine);
    const double secondAngle = Pi - firstAngle;
    const double angleTolerance = std::numeric_limits<double>::epsilon() * 64.0; // 覆盖反三角函数和角度规范化的舍入误差。
    const double candidateAngles[2] = {firstAngle, secondAngle};
    unsigned int count = 0;

    for (int index = 0; index < 2; ++index)
    {
        const double angle = candidateAngles[index];

        if (!arc.containsAngle(angle, angleTolerance))
        {
            continue;
        }

        const double cosine = std::cos(angle);

        if (std::fabs(cosine) <= angleTolerance)
        {
            continue;
        }

        const double intersectionX = arc.center().x() + arc.radius() * cosine;

        if (intersectionX > point.x() + tolerance)
        {
            ++count;
        }
    }

    return count;
}

// 判断点是否位于局部XY平面矩形内部或边界上。
bool rectangleContainsPoint(const MyVoxel::Bounds3& bounds, const MyMath::Vector3& point, double tolerance)
{
    return point.x() >= bounds.minimum().x() - tolerance && point.x() <= bounds.maximum().x() + tolerance && point.y() >= bounds.minimum().y() - tolerance && point.y() <= bounds.maximum().y() + tolerance;
}

// 使用二维Slab算法判断直线段是否与局部XY平面矩形相交或接触。
bool lineIntersectsRectangle(const MyVoxel::Geometry_Line& line, const MyVoxel::Bounds3& bounds, double tolerance)
{
    double minimumParameter = 0.0;
    double maximumParameter = 1.0;
    const MyMath::Vector3 direction = line.endPoint() - line.startPoint();
    const double starts[2] = {line.startPoint().x(), line.startPoint().y()};
    const double deltas[2] = {direction.x(), direction.y()};
    const double minimums[2] = {bounds.minimum().x() - tolerance, bounds.minimum().y() - tolerance};
    const double maximums[2] = {bounds.maximum().x() + tolerance, bounds.maximum().y() + tolerance};

    for (int axis = 0; axis < 2; ++axis)
    {
        if (deltas[axis] == 0.0)
        {
            if (starts[axis] < minimums[axis] || starts[axis] > maximums[axis])
            {
                return false;
            }

            continue;
        }

        double first = (minimums[axis] - starts[axis]) / deltas[axis];
        double second = (maximums[axis] - starts[axis]) / deltas[axis];

        if (first > second)
        {
            std::swap(first, second);
        }

        minimumParameter = (std::max)(minimumParameter, first);
        maximumParameter = (std::min)(maximumParameter, second);

        if (minimumParameter > maximumParameter)
        {
            return false;
        }
    }

    return true;
}

// 判断指定圆弧角点是否落在局部XY平面矩形边界范围内。
bool arcCandidateInsideRectangleEdge(const MyVoxel::Geometry_Arc& arc, double angle, const MyVoxel::Bounds3& bounds, double tolerance, bool verticalEdge)
{
    const double angleTolerance = std::numeric_limits<double>::epsilon() * 64.0; // 覆盖反三角函数和角度规范化的舍入误差。

    if (!arc.containsAngle(angle, angleTolerance))
    {
        return false;
    }

    const double x = arc.center().x() + arc.radius() * std::cos(angle);
    const double y = arc.center().y() + arc.radius() * std::sin(angle);

    if (verticalEdge)
    {
        return y >= bounds.minimum().y() - tolerance && y <= bounds.maximum().y() + tolerance;
    }

    return x >= bounds.minimum().x() - tolerance && x <= bounds.maximum().x() + tolerance;
}

// 判断圆弧是否与局部XY平面矩形相交、接触或完整位于矩形内部。
bool arcIntersectsRectangle(const MyVoxel::Geometry_Arc& arc, const MyVoxel::Bounds3& bounds, double tolerance)
{
    if (rectangleContainsPoint(bounds, arc.startPoint(), tolerance) || rectangleContainsPoint(bounds, arc.endPoint(), tolerance))
    {
        return true;
    }

    const double verticalEdges[2] = {bounds.minimum().x(), bounds.maximum().x()};

    for (int edgeIndex = 0; edgeIndex < 2; ++edgeIndex)
    {
        const double cosine = (verticalEdges[edgeIndex] - arc.center().x()) / arc.radius();

        if (cosine >= -1.0 && cosine <= 1.0)
        {
            const double angle = std::acos(clampValue(cosine, -1.0, 1.0));

            if (arcCandidateInsideRectangleEdge(arc, angle, bounds, tolerance, true) || arcCandidateInsideRectangleEdge(arc, -angle, bounds, tolerance, true))
            {
                return true;
            }
        }
    }

    const double horizontalEdges[2] = {bounds.minimum().y(), bounds.maximum().y()};

    for (int edgeIndex = 0; edgeIndex < 2; ++edgeIndex)
    {
        const double sine = (horizontalEdges[edgeIndex] - arc.center().y()) / arc.radius();

        if (sine >= -1.0 && sine <= 1.0)
        {
            const double angle = std::asin(clampValue(sine, -1.0, 1.0));

            if (arcCandidateInsideRectangleEdge(arc, angle, bounds, tolerance, false) || arcCandidateInsideRectangleEdge(arc, Pi - angle, bounds, tolerance, false))
            {
                return true;
            }
        }
    }

    return false;
}

// 判断指定曲线几何是否与局部XY平面矩形相交、接触或完整位于矩形内部。
bool curveIntersectsRectangle(const MyVoxel::Geometry_Curve& curve, const MyVoxel::Bounds3& bounds, double tolerance)
{
    if (curve.kind() == MyVoxel::CurveKind::Line)
    {
        return lineIntersectsRectangle(static_cast<const MyVoxel::Geometry_Line&>(curve), bounds, tolerance);
    }

    return arcIntersectsRectangle(static_cast<const MyVoxel::Geometry_Arc&>(curve), bounds, tolerance);
}

// 返回指定有向曲线几何对闭合轮廓有符号面积的精确贡献。
double curveSignedAreaContribution(const MyVoxel::Geometry_Curve& curve)
{
    if (curve.kind() == MyVoxel::CurveKind::Line)
    {
        const MyVoxel::Geometry_Line& line = static_cast<const MyVoxel::Geometry_Line&>(curve);
        return 0.5 * (line.startPoint().x() * line.endPoint().y() - line.endPoint().x() * line.startPoint().y());
    }

    const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(curve);
    const double firstAngle = arc.startAngle();
    const double secondAngle = arc.startAngle() + arc.sweepAngle();
    const double integral = arc.radius() * arc.center().x() * (std::sin(secondAngle) - std::sin(firstAngle)) - arc.radius() * arc.center().y() * (std::cos(secondAngle) - std::cos(firstAngle)) + arc.radius() * arc.radius() * arc.sweepAngle();
    return integral * 0.5;
}

}

namespace MyVoxel
{

Geometry_Revolved::Geometry_Revolved(const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves, double connectionTolerance)
    : m_profileCurves(profileCurves)
    , m_connectionTolerance(connectionTolerance)
    , m_profileSignedArea(0.0)
    , m_radialSign(0.0)
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(connectionTolerance) && connectionTolerance >= 0.0, "Geometry_Revolved connection tolerance must be finite and non-negative.");
    rebuild();
}

/// 轮廓几何数据

std::size_t Geometry_Revolved::profileCurveCount() const
{
    return m_profileCurves.size();
}

const Geometry_Curve& Geometry_Revolved::profileCurve(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot access an invalid Geometry_Revolved.");
    MYVOXEL_ASSERT_MESSAGE(index < m_profileCurves.size(), "Geometry_Revolved profile curve index is out of range.");
    return *m_profileCurves[index];
}

const std::vector<Foundation::RefPtr<const Geometry_Curve> >& Geometry_Revolved::profileCurves() const
{
    return m_profileCurves;
}

double Geometry_Revolved::connectionTolerance() const
{
    return m_connectionTolerance;
}

const Bounds3& Geometry_Revolved::profileBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot access the profile bounds of an invalid Geometry_Revolved.");
    return m_profileBounds;
}

double Geometry_Revolved::profileSignedArea() const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot access the profile area of an invalid Geometry_Revolved.");
    return m_profileSignedArea;
}

double Geometry_Revolved::radialSign() const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot access the radial sign of an invalid Geometry_Revolved.");
    return m_radialSign;
}

/// 几何属性

ShapeKind Geometry_Revolved::kind() const
{
    return ShapeKind::Revolved;
}


/// 标准空间查询

bool Geometry_Revolved::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot query an invalid Geometry_Revolved.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Geometry_Revolved query point must be finite.");

    if (!localBounds().contains(point))
    {
        return false;
    }

    const double radius = stableLength(point.x(), point.y());
    return containsProfilePoint(MyMath::Vector3(m_radialSign * radius, point.z(), 0.0), m_connectionTolerance);
}

ShapeRelation Geometry_Revolved::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot query an invalid Geometry_Revolved.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Revolved query bounds must be valid.");
    return classifyRange(bounds.minimum(), bounds.maximum());
}

/// 快速空间查询

ShapeRelation Geometry_Revolved::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Cannot query an invalid Geometry_Revolved.");
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "Geometry_Revolved query center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "Geometry_Revolved query extent must be finite and non-negative.");
    return classifyRange(center - extent, center + extent);
}

/// 缓存建立

void Geometry_Revolved::rebuild()
{
    m_profileBounds.clear();
    m_profileSignedArea = 0.0;
    m_radialSign = 0.0;
    clearLocalBounds();
    m_valid = false;

    if (!isFiniteValue(m_connectionTolerance) || m_connectionTolerance < 0.0 || m_profileCurves.empty())
    {
        return;
    }

    for (std::size_t index = 0; index < m_profileCurves.size(); ++index)
    {
        const Foundation::RefPtr<const Geometry_Curve>& resource = m_profileCurves[index];

        if (!resource || (resource->kind() != CurveKind::Line && resource->kind() != CurveKind::Arc) || !isFinitePlanarPoint(resource->startPoint()) || !isFinitePlanarPoint(resource->endPoint()) || !isFiniteValue(resource->length()) || resource->length() <= 0.0 || !resource->bounds().isValid())
        {
            return;
        }
    }

    for (std::size_t index = 0; index < m_profileCurves.size(); ++index)
    {
        const Geometry_Curve& current = *m_profileCurves[index];
        const Geometry_Curve& next = *m_profileCurves[(index + 1) % m_profileCurves.size()];

        if (!current.endPoint().isEqualTo(next.startPoint(), m_connectionTolerance))
        {
            return;
        }

        m_profileBounds.include(current.bounds());
        m_profileSignedArea += curveSignedAreaContribution(current);
    }

    if (!m_profileBounds.isValid() || !isFiniteValue(m_profileSignedArea) || m_profileSignedArea == 0.0)
    {
        return;
    }

    const double minimumX = m_profileBounds.minimum().x();
    const double maximumX = m_profileBounds.maximum().x();
    const bool onPositiveSide = minimumX >= -m_connectionTolerance;
    const bool onNegativeSide = maximumX <= m_connectionTolerance;

    if (!onPositiveSide && !onNegativeSide)
    {
        return;
    }

    m_radialSign = onPositiveSide ? 1.0 : -1.0;

    const double maximumRadius = (std::max)(std::fabs(minimumX), std::fabs(maximumX));
    const double minimumZ = m_profileBounds.minimum().y();
    const double maximumZ = m_profileBounds.maximum().y();
    const Bounds3 bounds(MyMath::Vector3(-maximumRadius, -maximumRadius, minimumZ),
                         MyMath::Vector3(maximumRadius, maximumRadius, maximumZ));

    if (!bounds.isValid() || !bounds.hasVolume())
    {
        return;
    }

    setLocalBounds(bounds);
    m_valid = true;
}

/// 轮廓区域查询

bool Geometry_Revolved::containsProfilePoint(const MyMath::Vector3& point, double tolerance) const
{
    if (!m_profileBounds.contains(point, tolerance))
    {
        return false;
    }

    for (std::size_t index = 0; index < m_profileCurves.size(); ++index)
    {
        if (pointOnCurve(point, *m_profileCurves[index], tolerance))
        {
            return true;
        }
    }

    double coordinateScale = 1.0;
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().y()));

    const double queryY = point.y() + coordinateScale * std::numeric_limits<double>::epsilon() * 64.0; // 避开轮廓顶点和圆弧水平极值，稳定奇偶射线计数。
    unsigned int intersectionCount = 0;

    for (std::size_t index = 0; index < m_profileCurves.size(); ++index)
    {
        const Geometry_Curve& curve = *m_profileCurves[index];

        if (curve.kind() == CurveKind::Line)
        {
            intersectionCount += lineHorizontalRayIntersections(point, queryY, static_cast<const Geometry_Line&>(curve), tolerance);
        }
        else
        {
            intersectionCount += arcHorizontalRayIntersections(point, queryY, static_cast<const Geometry_Arc&>(curve), tolerance);
        }
    }

    return (intersectionCount & 1U) != 0;
}

ShapeRelation Geometry_Revolved::classifyProfileBounds(const Bounds3& bounds, double tolerance) const
{
    if (!m_profileBounds.intersects(bounds, tolerance))
    {
        return ShapeRelation::Outside;
    }

    for (std::size_t index = 0; index < m_profileCurves.size(); ++index)
    {
        if (curveIntersectsRectangle(*m_profileCurves[index], bounds, tolerance))
        {
            return ShapeRelation::Intersecting;
        }
    }

    return containsProfilePoint(bounds.center(), tolerance) ? ShapeRelation::Inside : ShapeRelation::Outside;
}

ShapeRelation Geometry_Revolved::classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const
{
    const Bounds3& bounds = localBounds();

    if (maximum.x() < bounds.minimum().x() || minimum.x() > bounds.maximum().x() ||
        maximum.y() < bounds.minimum().y() || minimum.y() > bounds.maximum().y() ||
        maximum.z() < bounds.minimum().z() || minimum.z() > bounds.maximum().z())
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
    const ShapeRelation relation = classifyProfileBounds(profileBounds, m_connectionTolerance);

    if (relation != ShapeRelation::Intersecting || minimumRadius > 0.0)
    {
        return relation;
    }

    double coordinateScale = 1.0;
    coordinateScale = (std::max)(coordinateScale, maximumRadius);
    coordinateScale = (std::max)(coordinateScale, std::fabs(minimum.z()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(maximum.z()));

    const double axisOffset = (std::max)(m_connectionTolerance, coordinateScale * std::numeric_limits<double>::epsilon() * 64.0); // 旋转轴上的轮廓边退化为轴线，使用极小径向探针区分真实表面相交。
    const double probeMinimumX = m_radialSign > 0.0 ? axisOffset : -maximumRadius;
    const double probeMaximumX = m_radialSign > 0.0 ? maximumRadius : -axisOffset;
    const Bounds3 probeBounds(MyMath::Vector3((std::min)(probeMinimumX, probeMaximumX), minimum.z(), 0.0), MyMath::Vector3((std::max)(probeMinimumX, probeMaximumX), maximum.z(), 0.0));

    return classifyProfileBounds(probeBounds, m_connectionTolerance) == ShapeRelation::Inside ? ShapeRelation::Inside : relation;
}

}