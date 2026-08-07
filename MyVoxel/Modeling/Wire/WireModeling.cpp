#include "WireModeling.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"

namespace
{

const double HalfScale = 0.5; // 完整矩形尺寸转换为半尺寸使用的固定比例。
const double Pi = 3.1415926535897932384626433832795; // 标准完整圆构造使用的圆周率。
const double TwoPi = Pi * 2.0; // 标准完整圆逆时针扫掠角。
const double CircleClosureToleranceScale = 64.0; // 覆盖sin/cos计算完整圆端点时的双精度舍入误差。

// 判断指定点是否为局部XY平面有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

// 判断指定顶点序列是否全部位于局部XY平面且相邻顶点互不相同。
bool isValidPointSequence(const std::vector<MyMath::Vector3>& points, bool closed)
{
    if (points.size() < (closed ? 3U : 2U))
    {
        return false;
    }

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        if (!isFinitePlanarPoint(points[index]))
        {
            return false;
        }

        if (index > 0 && points[index].isEqualTo(points[index - 1], 0.0))
        {
            return false;
        }
    }

    return !closed || !points.front().isEqualTo(points.back(), 0.0);
}

// 根据完整圆坐标尺度返回只用于数值闭合判断的最小容差。
double circleClosureTolerance(const MyMath::Vector3& center, double radius)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(center.x()));
    scale = (std::max)(scale, std::fabs(center.y()));
    scale = (std::max)(scale, std::fabs(radius));
    return scale * (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 局部拓扑曲线创建

Topology_Curve createLine(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
{
    MYVOXEL_ASSERT_MESSAGE(isFinitePlanarPoint(startPoint) && isFinitePlanarPoint(endPoint) && !startPoint.isEqualTo(endPoint, 0.0),
                           "Line modeling requires two different finite points in the local XY plane.");

    if (!isFinitePlanarPoint(startPoint) || !isFinitePlanarPoint(endPoint) || startPoint.isEqualTo(endPoint, 0.0))
    {
        return Topology_Curve();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Line(startPoint, endPoint));
    return Topology_Curve(geometry);
}

Topology_Curve createArc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
{
    const bool valid = isFinitePlanarPoint(center) && std::isfinite(radius) && radius > 0.0 &&
                       std::isfinite(startAngle) && std::isfinite(sweepAngle) && sweepAngle != 0.0 &&
                       std::fabs(sweepAngle) <= TwoPi;

    MYVOXEL_ASSERT_MESSAGE(valid, "Arc modeling requires finite planar center, positive radius and non-zero sweep within 2*pi.");

    if (!valid)
    {
        return Topology_Curve();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Arc(center, radius, startAngle, sweepAngle));
    return Topology_Curve(geometry);
}

/// 空间Curve实例创建

Curve makeLine(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
{
    return Curve(createLine(startPoint, endPoint));
}

Curve makeLine(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint, const MyMath::Matrix4& localToWorld)
{
    return Curve(createLine(startPoint, endPoint), localToWorld);
}

Curve makeArc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
{
    return Curve(createArc(center, radius, startAngle, sweepAngle));
}

Curve makeArc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle, const MyMath::Matrix4& localToWorld)
{
    return Curve(createArc(center, radius, startAngle, sweepAngle), localToWorld);
}

/// 局部Topology_Wire创建

Topology_Wire createWire(const Topology_CurveList& curves, double connectionTolerance)
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(connectionTolerance) && connectionTolerance >= 0.0,
                           "Wire modeling connection tolerance must be finite and non-negative.");

    if (!std::isfinite(connectionTolerance) || connectionTolerance < 0.0)
    {
        return Topology_Wire();
    }

    return Topology_Wire(curves, connectionTolerance);
}

Topology_Wire createPolyline(const std::vector<MyMath::Vector3>& points)
{
    MYVOXEL_ASSERT_MESSAGE(isValidPointSequence(points, false),
                           "Polyline modeling requires at least two finite planar points without repeated adjacent vertices.");

    if (!isValidPointSequence(points, false))
    {
        return Topology_Wire();
    }

    Topology_CurveList curves;
    curves.reserve(points.size() - 1);

    for (std::size_t index = 0; index + 1 < points.size(); ++index)
    {
        curves.push_back(createLine(points[index], points[index + 1]));
    }

    return createWire(curves, 0.0);
}

Topology_Wire createPolygon(const std::vector<MyMath::Vector3>& points)
{
    MYVOXEL_ASSERT_MESSAGE(isValidPointSequence(points, true),
                           "Polygon modeling requires at least three finite planar vertices and must not repeat the first vertex at the end.");

    if (!isValidPointSequence(points, true))
    {
        return Topology_Wire();
    }

    Topology_CurveList curves;
    curves.reserve(points.size());

    for (std::size_t index = 0; index + 1 < points.size(); ++index)
    {
        curves.push_back(createLine(points[index], points[index + 1]));
    }

    curves.push_back(createLine(points.back(), points.front()));
    return createWire(curves, 0.0);
}

Topology_Wire createRectangle(double sizeX, double sizeY)
{
    return createRectangle(MyMath::Vector3(0.0, 0.0, 0.0), sizeX, sizeY);
}

Topology_Wire createRectangle(const MyMath::Vector3& center, double sizeX, double sizeY)
{
    const bool valid = isFinitePlanarPoint(center) && std::isfinite(sizeX) && sizeX > 0.0 &&
                       std::isfinite(sizeY) && sizeY > 0.0;

    MYVOXEL_ASSERT_MESSAGE(valid, "Rectangle modeling requires finite planar center and positive finite sizes.");

    if (!valid)
    {
        return Topology_Wire();
    }

    const double halfX = sizeX * HalfScale;
    const double halfY = sizeY * HalfScale;
    std::vector<MyMath::Vector3> points;
    points.reserve(4); // 标准矩形固定由四个逆时针顶点定义。
    points.push_back(MyMath::Vector3(center.x() - halfX, center.y() - halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() + halfX, center.y() - halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() + halfX, center.y() + halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() - halfX, center.y() + halfY, 0.0));
    return createPolygon(points);
}

Topology_Wire createCircle(double radius)
{
    return createCircle(MyMath::Vector3(0.0, 0.0, 0.0), radius);
}

Topology_Wire createCircle(const MyMath::Vector3& center, double radius)
{
    const bool valid = isFinitePlanarPoint(center) && std::isfinite(radius) && radius > 0.0;

    MYVOXEL_ASSERT_MESSAGE(valid, "Circle modeling requires finite planar center and positive finite radius.");

    if (!valid)
    {
        return Topology_Wire();
    }

    Topology_CurveList curves;
    curves.reserve(1); // 完整圆直接由一条2*pi逆时针Geometry_Arc表示。
    curves.push_back(createArc(center, radius, 0.0, TwoPi));
    return createWire(curves, circleClosureTolerance(center, radius));
}

/// 空间Wire实例创建

Wire makeWire(const Topology_CurveList& curves, double connectionTolerance)
{
    return Wire(createWire(curves, connectionTolerance));
}

Wire makeWire(const Topology_CurveList& curves, double connectionTolerance, const MyMath::Matrix4& localToWorld)
{
    return Wire(createWire(curves, connectionTolerance), localToWorld);
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolyline(points));
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points, const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolyline(points), localToWorld);
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolygon(points));
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points, const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolygon(points), localToWorld);
}

Wire makeRectangle(double sizeX, double sizeY)
{
    return Wire(createRectangle(sizeX, sizeY));
}

Wire makeRectangle(double sizeX, double sizeY, const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(sizeX, sizeY), localToWorld);
}

Wire makeRectangle(const MyMath::Vector3& center, double sizeX, double sizeY)
{
    return Wire(createRectangle(center, sizeX, sizeY));
}

Wire makeRectangle(const MyMath::Vector3& center, double sizeX, double sizeY, const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(center, sizeX, sizeY), localToWorld);
}

Wire makeCircle(double radius)
{
    return Wire(createCircle(radius));
}

Wire makeCircle(double radius, const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(radius), localToWorld);
}

Wire makeCircle(const MyMath::Vector3& center, double radius)
{
    return Wire(createCircle(center, radius));
}

Wire makeCircle(const MyMath::Vector3& center, double radius, const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(center, radius), localToWorld);
}

}
}
