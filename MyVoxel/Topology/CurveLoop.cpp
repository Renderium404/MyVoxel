#include "CurveLoop.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ArcCurve.h"
#include "LineCurve.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆弧交点计算统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整圆对应的弧度值。

// 判断数值是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断指定点是否为局部XY平面中的有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

// 将数值限制到指定闭区间。
double clampValue(double value, double minimum, double maximum)
{
    return (std::max)(minimum, (std::min)(maximum, value));
}

// 返回点到局部XY平面直线段的最短距离平方。
double pointLineDistanceSquared(const MyMath::Vector3& point, const MyVoxel::Geometry::LineCurve& line)
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
bool pointOnArc(const MyMath::Vector3& point, const MyVoxel::Geometry::ArcCurve& arc, double tolerance)
{
    const double deltaX = point.x() - arc.center().x();
    const double deltaY = point.y() - arc.center().y();
    const double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

    if (std::fabs(distance - arc.radius()) > tolerance)
    {
        return false;
    }

    const double angle = std::atan2(deltaY, deltaX);
    const double angleTolerance = tolerance > 0.0 ? tolerance / arc.radius() : 0.0;
    return arc.containsAngle(angle, angleTolerance);
}

// 判断点是否位于指定曲线边界上。
bool pointOnCurve(const MyMath::Vector3& point, const MyVoxel::Geometry::Geo_Curve& curve, double tolerance)
{
    if (curve.kind() == MyVoxel::Geometry::CurveKind::Line)
    {
        return pointLineDistanceSquared(point, static_cast<const MyVoxel::Geometry::LineCurve&>(curve)) <= tolerance * tolerance;
    }

    return pointOnArc(point, static_cast<const MyVoxel::Geometry::ArcCurve&>(curve), tolerance);
}

// 返回直线段与向右水平射线的交点数量，查询Y已避开轮廓端点和极值。
unsigned int lineHorizontalRayIntersections(const MyMath::Vector3& point, double queryY, const MyVoxel::Geometry::LineCurve& line, double tolerance)
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
unsigned int arcHorizontalRayIntersections(const MyMath::Vector3& point, double queryY, const MyVoxel::Geometry::ArcCurve& arc, double tolerance)
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
    unsigned int count = 0;

    const double candidateAngles[2] = {firstAngle, secondAngle};

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
bool lineIntersectsRectangle(const MyVoxel::Geometry::LineCurve& line, const MyVoxel::Bounds3& bounds, double tolerance)
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
bool arcCandidateInsideRectangleEdge(const MyVoxel::Geometry::ArcCurve& arc, double angle, const MyVoxel::Bounds3& bounds, double tolerance, bool verticalEdge)
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
bool arcIntersectsRectangle(const MyVoxel::Geometry::ArcCurve& arc, const MyVoxel::Bounds3& bounds, double tolerance)
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

// 判断指定曲线是否与局部XY平面矩形相交、接触或完整位于矩形内部。
bool curveIntersectsRectangle(const MyVoxel::Geometry::Curve& curve, const MyVoxel::Bounds3& bounds, double tolerance)
{
    if (curve.kind() == MyVoxel::Geometry::CurveKind::Line)
    {
        return lineIntersectsRectangle(static_cast<const MyVoxel::Geometry::LineCurve&>(curve), bounds, tolerance);
    }

    return arcIntersectsRectangle(static_cast<const MyVoxel::Geometry::ArcCurve&>(curve), bounds, tolerance);
}

// 返回指定有向曲线对闭合轮廓有符号面积的精确贡献。
double curveSignedAreaContribution(const MyVoxel::Geometry::Curve& curve)
{
    if (curve.kind() == MyVoxel::Geometry::CurveKind::Line)
    {
        const MyVoxel::Geometry::LineCurve& line = static_cast<const MyVoxel::Geometry::LineCurve&>(curve);
        return 0.5 * (line.startPoint().x() * line.endPoint().y() - line.endPoint().x() * line.startPoint().y());
    }

    const MyVoxel::Geometry::ArcCurve& arc = static_cast<const MyVoxel::Geometry::ArcCurve&>(curve);
    const double firstAngle = arc.startAngle();
    const double secondAngle = arc.startAngle() + arc.sweepAngle();
    const double integral = arc.radius() * arc.center().x() * (std::sin(secondAngle) - std::sin(firstAngle)) - arc.radius() * arc.center().y() * (std::cos(secondAngle) - std::cos(firstAngle)) + arc.radius() * arc.radius() * arc.sweepAngle();
    return integral * 0.5;
}

}

namespace MyVoxel
{
namespace Geometry
{

CurveLoop::CurveLoop()
    : m_connectionTolerance(0.0)
    , m_signedArea(0.0)
    , m_valid(false)
{
}

CurveLoop::CurveLoop(const std::vector<Foundation::RefPtr<const Curve>>& curves, double connectionTolerance)
    : m_curves(curves)
    , m_connectionTolerance(connectionTolerance)
    , m_signedArea(0.0)
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(connectionTolerance) && connectionTolerance >= 0.0, "CurveLoop connection tolerance must be finite and non-negative.");
    rebuild();
}

/// 状态判断

bool CurveLoop::isValid() const
{
    return m_valid;
}

bool CurveLoop::isCounterClockwise() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query the direction of an invalid CurveLoop.");
    return m_signedArea > 0.0;
}

/// 轮廓数据

std::size_t CurveLoop::curveCount() const
{
    return m_curves.size();
}

const Curve& CurveLoop::curve(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid CurveLoop.");
    MYVOXEL_ASSERT_MESSAGE(index < m_curves.size(), "CurveLoop curve index is out of range.");
    return *m_curves[index];
}

const std::vector<Foundation::RefPtr<const Curve>>& CurveLoop::curves() const
{
    return m_curves;
}

double CurveLoop::connectionTolerance() const
{
    return m_connectionTolerance;
}

const Bounds3& CurveLoop::bounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the bounds of an invalid CurveLoop.");
    return m_bounds;
}

double CurveLoop::signedArea() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the area of an invalid CurveLoop.");
    return m_signedArea;
}

/// 空间查询

bool CurveLoop::containsPoint(const MyMath::Vector3& point, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid CurveLoop.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePlanarPoint(point), "CurveLoop query point must be finite and lie in the local XY plane.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(tolerance) && tolerance >= 0.0, "CurveLoop query tolerance must be finite and non-negative.");

    if (!m_bounds.contains(point, tolerance))
    {
        return false;
    }

    for (std::size_t index = 0; index < m_curves.size(); ++index)
    {
        if (pointOnCurve(point, *m_curves[index], tolerance))
        {
            return true;
        }
    }

    double coordinateScale = 1.0;
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_bounds.minimum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_bounds.minimum().y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_bounds.maximum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_bounds.maximum().y()));

    const double queryY = point.y() + coordinateScale * std::numeric_limits<double>::epsilon() * 64.0; // 避开轮廓顶点和圆弧水平极值，稳定奇偶射线计数。
    unsigned int intersectionCount = 0;

    for (std::size_t index = 0; index < m_curves.size(); ++index)
    {
        if (m_curves[index]->kind() == CurveKind::Line)
        {
            intersectionCount += lineHorizontalRayIntersections(point, queryY, static_cast<const LineCurve&>(*m_curves[index]), tolerance);
        }
        else
        {
            intersectionCount += arcHorizontalRayIntersections(point, queryY, static_cast<const ArcCurve&>(*m_curves[index]), tolerance);
        }
    }

    return (intersectionCount & 1U) != 0;
}

ShapeRelation CurveLoop::classifyBounds(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid CurveLoop.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "CurveLoop query bounds must be valid.");
    MYVOXEL_ASSERT_MESSAGE(bounds.minimum().z() == 0.0 && bounds.maximum().z() == 0.0, "CurveLoop query bounds must lie in the local XY plane.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(tolerance) && tolerance >= 0.0, "CurveLoop query tolerance must be finite and non-negative.");

    if (!m_bounds.intersects(bounds, tolerance))
    {
        return ShapeRelation::Outside;
    }

    for (std::size_t index = 0; index < m_curves.size(); ++index)
    {
        if (curveIntersectsRectangle(*m_curves[index], bounds, tolerance))
        {
            return ShapeRelation::Intersecting;
        }
    }

    return containsPoint(bounds.center(), tolerance) ? ShapeRelation::Inside : ShapeRelation::Outside;
}

/// 轮廓创建

CurveLoop CurveLoop::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid CurveLoop.");

    std::vector<Foundation::RefPtr<const Curve>> reversedCurves;
    reversedCurves.reserve(m_curves.size());

    for (std::size_t reverseIndex = m_curves.size(); reverseIndex > 0; --reverseIndex)
    {
        reversedCurves.push_back(m_curves[reverseIndex - 1]->reversed());
    }

    return CurveLoop(reversedCurves, m_connectionTolerance);
}

void CurveLoop::rebuild()
{
    m_bounds.clear();
    m_signedArea = 0.0;
    m_valid = false;

    if (!isFiniteValue(m_connectionTolerance) || m_connectionTolerance < 0.0 || m_curves.empty())
    {
        return;
    }

    for (std::size_t index = 0; index < m_curves.size(); ++index)
    {
        if (!m_curves[index] || !m_curves[index]->bounds().isValid())
        {
            return;
        }

        const std::size_t nextIndex = (index + 1) % m_curves.size();

        if (!m_curves[index]->endPoint().isEqualTo(m_curves[nextIndex]->startPoint(), m_connectionTolerance))
        {
            return;
        }

        m_bounds.include(m_curves[index]->bounds());
        m_signedArea += curveSignedAreaContribution(*m_curves[index]);
    }

    if (!m_bounds.isValid() || m_signedArea == 0.0)
    {
        return;
    }

    m_valid = true;
}

}
}
