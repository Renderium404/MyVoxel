#include "WireModeling.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "MyVoxel/Modeling/Curve/CurveModeling.h"
namespace
{

const double HalfScale = 0.5; // 完整矩形尺寸转换为半尺寸使用的固定比例。
const double Pi = 3.1415926535897932384626433832795; // 圆弧和完整圆建模使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆对应的绝对扫掠角。
const double CircleClosureToleranceScale = 64.0; // 覆盖完整圆三角函数端点舍入误差的双精度容差倍数。

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

// 判断指定圆弧扫掠是否表示数值意义上的完整圆。
bool isFullCircleSweep(double sweepAngle)
{
    return std::fabs(std::fabs(sweepAngle) - TwoPi) <= (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
}

// 根据完整圆坐标尺度返回只用于起终点几何一致性验证的最小容差。
double circleClosureTolerance(const MyMath::Vector3& center, double radius)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(center.x()));
    scale = (std::max)(scale, std::fabs(center.y()));
    scale = (std::max)(scale, std::fabs(radius));
    return scale * (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
}

// 使用已经确定的共享拓扑顶点创建直线Topology_Edge。
MyVoxel::Topology_Edge createLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(new MyVoxel::Geometry_Line(startVertex.point(), endVertex.point()));
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, 0.0);
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Wire创建

Topology_Wire createWire(const std::vector<Topology_Edge>& edges)
{
    return Topology_Wire(edges);
}

Topology_Wire createPolyline(const std::vector<MyMath::Vector3>& points)
{
    const bool valid = isValidPointSequence(points, false);
    MYVOXEL_ASSERT_MESSAGE(valid, "Polyline modeling requires at least two finite planar points without repeated adjacent vertices.");

    if (!valid)
    {
        return Topology_Wire();
    }

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size() - 1);

    for (std::size_t index = 0; index + 1 < vertices.size(); ++index)
    {
        edges.push_back(createLineEdge(vertices[index], vertices[index + 1]));
    }

    return createWire(edges);
}

Topology_Wire createPolygon(const std::vector<MyMath::Vector3>& points)
{
    const bool valid = isValidPointSequence(points, true);
    MYVOXEL_ASSERT_MESSAGE(valid, "Polygon modeling requires at least three finite planar vertices and must not repeat the first vertex at the end.");

    if (!valid)
    {
        return Topology_Wire();
    }

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size());

    for (std::size_t index = 0; index + 1 < vertices.size(); ++index)
    {
        edges.push_back(createLineEdge(vertices[index], vertices[index + 1]));
    }

    edges.push_back(createLineEdge(vertices.back(), vertices.front()));
    return createWire(edges);
}

Topology_Wire createRectangle(double sizeX, double sizeY)
{
    return createRectangle(MyMath::Vector3(0.0, 0.0, 0.0), sizeX, sizeY);
}

Topology_Wire createRectangle(const MyMath::Vector3& center, double sizeX, double sizeY)
{
    const bool valid = isFinitePlanarPoint(center) && std::isfinite(sizeX) && sizeX > 0.0 && std::isfinite(sizeY) && sizeY > 0.0;
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

    std::vector<Topology_Edge> edges;
    edges.reserve(1); // 完整圆直接由一条2*pi逆时针闭合圆弧Edge表示。
    edges.push_back(createArc(center, radius, 0.0, TwoPi));

    if (!edges.front().isValid())
    {
        return Topology_Wire();
    }

    return createWire(edges);
}

/// 空间Wire实例创建

Wire makeWire(const std::vector<Topology_Edge>& edges)
{
    return Wire(createWire(edges));
}

Wire makeWire(const std::vector<Topology_Edge>& edges, const MyMath::Matrix4& localToWorld)
{
    return Wire(createWire(edges), localToWorld);
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