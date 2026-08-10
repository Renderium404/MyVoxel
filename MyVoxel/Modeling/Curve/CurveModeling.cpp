#include "CurveModeling.h"

#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧建模使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆对应的绝对扫掠角。
const double CircleClosureToleranceScale = 64.0; // 覆盖完整圆三角函数端点舍入误差的双精度容差倍数。

bool isFinite(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 根据完整圆坐标尺度返回只用于几何端点闭合验证的最小容差。
double circleClosureTolerance(const MyMath::Vector3& center, double radius)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(center.x()));
    scale = (std::max)(scale, std::fabs(center.y()));
    scale = (std::max)(scale, std::fabs(center.z()));
    scale = (std::max)(scale, std::fabs(radius));
    return scale * (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
}

// 根据连续曲线几何创建具有正确端点拓扑身份的Topology_Edge。
MyVoxel::Topology_Edge createEdgeFromGeometry(const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>& geometry, double endpointTolerance, bool closed)
{
    if (!geometry)
    {
        return MyVoxel::Topology_Edge();
    }

    const MyVoxel::Topology_Vertex startVertex(geometry->startPoint());

    if (closed)
    {
        return MyVoxel::Topology_Edge(startVertex, startVertex, geometry, endpointTolerance);
    }

    const MyVoxel::Topology_Vertex endVertex(geometry->endPoint());
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, endpointTolerance);
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Edge创建

Topology_Edge createLine(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
{
    const bool valid = startPoint.isFinite() && endPoint.isFinite() && !startPoint.isEqualTo(endPoint, 0.0);
    MYVOXEL_ASSERT_MESSAGE(valid, "Line modeling requires two different finite points.");

    if (!valid)
    {
        return Topology_Edge();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Line(startPoint, endPoint));
    return createEdgeFromGeometry(geometry, 0.0, false);
}

Topology_Edge createArc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
{
    const bool valid = center.isFinite() && isFinite(radius) && radius > 0.0 && isFinite(startAngle) && isFinite(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= TwoPi;
    MYVOXEL_ASSERT_MESSAGE(valid, "Arc modeling requires finite center, positive radius and non-zero sweep within 2*pi.");

    if (!valid)
    {
        return Topology_Edge();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Arc(center, radius, startAngle, sweepAngle));
    const bool closed = std::fabs(std::fabs(sweepAngle) - TwoPi) <= (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
    return createEdgeFromGeometry(geometry, closed ? circleClosureTolerance(center, radius) : 0.0, closed);
}

Topology_Edge createArc(const MyMath::CoordinateSystem& coordinateSystem, double radius, double startAngle, double sweepAngle)
{
    const bool valid = coordinateSystem.isValid() && isFinite(radius) && radius > 0.0 && isFinite(startAngle) && isFinite(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= TwoPi;
    MYVOXEL_ASSERT_MESSAGE(valid, "Arc modeling requires valid coordinate system, positive radius and non-zero sweep within 2*pi.");

    if (!valid)
    {
        return Topology_Edge();
    }

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Arc(coordinateSystem, radius, startAngle, sweepAngle));
    const bool closed = std::fabs(std::fabs(sweepAngle) - TwoPi) <= (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
    return createEdgeFromGeometry(geometry, closed ? circleClosureTolerance(coordinateSystem.origin(), radius) : 0.0, closed);
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

Curve makeArc(const MyMath::CoordinateSystem& coordinateSystem, double radius, double startAngle, double sweepAngle)
{
    return Curve(createArc(coordinateSystem, radius, startAngle, sweepAngle));
}

Curve makeArc(const MyMath::CoordinateSystem& coordinateSystem, double radius, double startAngle, double sweepAngle, const MyMath::Matrix4& localToWorld)
{
    return Curve(createArc(coordinateSystem, radius, startAngle, sweepAngle), localToWorld);
}

}
}