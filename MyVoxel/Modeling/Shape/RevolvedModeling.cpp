#include "RevolvedModeling.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"

namespace
{

const double NumericalScale = 64.0; // 平面法向和浮点端点转换覆盖舍入误差使用的固定倍数。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断点是否在指定容差内位于回转母线局部XY平面。
bool isPlanarPoint(const MyMath::Vector3& point, double tolerance)
{
    return point.isFinite() && std::fabs(point.z()) <= tolerance;
}

// 投影母线点到严格局部XY平面。
MyMath::Vector3 projectedPoint(const MyMath::Vector3& point)
{
    return MyMath::Vector3(point.x(), point.y(), 0.0);
}

// 返回圆弧平面法向允许偏离局部Z轴的数值容差。
double arcNormalTolerance(const MyVoxel::Geometry_Arc& arc, double profileTolerance)
{
    double scale = 1.0;
    scale = (std::max)(scale, arc.radius());
    scale = (std::max)(scale, std::fabs(arc.center().x()));
    scale = (std::max)(scale, std::fabs(arc.center().y()));
    return (std::max)(std::numeric_limits<double>::epsilon() * NumericalScale, profileTolerance / scale);
}

// 判断Topology_Edge引用的Arc是否可以作为回转体局部XY母线。
bool isPlanarArc(const MyVoxel::Geometry_Arc& arc, double profileTolerance)
{
    if (!isPlanarPoint(arc.center(), profileTolerance) || !arc.bounds().isValid())
    {
        return false;
    }

    if (std::fabs(arc.bounds().minimum().z()) > profileTolerance || std::fabs(arc.bounds().maximum().z()) > profileTolerance)
    {
        return false;
    }

    const MyMath::Vector3 normal = arc.normal();
    const double normalTolerance = arcNormalTolerance(arc, profileTolerance);
    return normal.isFinite() && std::fabs(normal.x()) <= normalTolerance && std::fabs(normal.y()) <= normalTolerance &&
           std::fabs(std::fabs(normal.z()) - 1.0) <= normalTolerance;
}

// 根据Topology_Edge当前裁剪区间和使用方向创建严格局部XY平面Line母线。
MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> profileLineFromEdge(const MyVoxel::Topology_Edge& edge, double profileTolerance)
{
    const MyMath::Vector3 sourceStart = edge.pointAt(0.0);
    const MyMath::Vector3 sourceEnd = edge.pointAt(1.0);

    if (!isPlanarPoint(sourceStart, profileTolerance) || !isPlanarPoint(sourceEnd, profileTolerance))
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    const MyMath::Vector3 startPoint = projectedPoint(sourceStart);
    const MyMath::Vector3 endPoint = projectedPoint(sourceEnd);

    if (startPoint.isEqualTo(endPoint, 0.0))
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(startPoint, endPoint));
}

// 根据Topology_Edge当前使用方向和Arc自身坐标系创建严格局部XY平面Arc母线。
MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> profileArcFromEdge(const MyVoxel::Topology_Edge& edge, double profileTolerance)
{
    const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());

    if (!isPlanarArc(arc, profileTolerance))
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    const MyMath::Vector3 sourceStart = edge.pointAt(0.0);
    const MyMath::Vector3 sourceEnd = edge.pointAt(1.0);

    if (!isPlanarPoint(sourceStart, profileTolerance) || !isPlanarPoint(sourceEnd, profileTolerance))
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    const MyMath::Vector3 center = projectedPoint(arc.center());
    const MyMath::Vector3 startPoint = projectedPoint(sourceStart);
    const MyMath::Vector3 endPoint = projectedPoint(sourceEnd);
    const double startAngle = std::atan2(startPoint.y() - center.y(), startPoint.x() - center.x());

    // Arc正扫掠相对于自身坐标系法向定义；Edge反向使用时需要同时翻转扫掠方向。
    const double edgeDirectionSign = edge.isForward() ? 1.0 : -1.0;
    const double planeOrientationSign = arc.normal().z() >= 0.0 ? 1.0 : -1.0;
    const double profileSweep = arc.sweepAngle() * edgeDirectionSign * planeOrientationSign;

    if (!isFiniteValue(profileSweep) || profileSweep == 0.0)
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> result(
        new MyVoxel::Geometry_Arc(center, arc.radius(), startAngle, profileSweep));

    if (!result->startPoint().isEqualTo(startPoint, profileTolerance) || !result->endPoint().isEqualTo(endPoint, profileTolerance))
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    return result;
}

// 将Topology_Edge当前有向使用转换为独立的规范化二维母线曲线。
MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> profileCurveFromEdge(const MyVoxel::Topology_Edge& edge, double profileTolerance)
{
    if (!edge.isValid())
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
    }

    if (edge.geometry().kind() == MyVoxel::CurveKind::Line)
    {
        return profileLineFromEdge(edge, profileTolerance);
    }

    if (edge.geometry().kind() == MyVoxel::CurveKind::Arc)
    {
        return profileArcFromEdge(edge, profileTolerance);
    }

    return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>();
}

// 将闭合Topology_Wire转换为按当前Wire方向排列的规范化母线曲线。
bool buildProfileCurves(const MyVoxel::Topology_Wire& profile, double profileTolerance,
                        std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> >& curves)
{
    curves.clear();

    if (!profile.isValid() || !profile.isClosed() || profile.edgeCount() == 0)
    {
        return false;
    }

    curves.reserve(profile.edgeCount());

    for (std::size_t index = 0; index < profile.edgeCount(); ++index)
    {
        const MyVoxel::Topology_Edge edge = profile.edge(index);
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> curve = profileCurveFromEdge(edge, profileTolerance);

        if (!curve)
        {
            curves.clear();
            return false;
        }

        curves.push_back(curve);
    }

    return true;
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

Topology_Shape createRevolved(const Topology_Wire& profile, double profileTolerance)
{
    const bool validTolerance = isFiniteValue(profileTolerance) && profileTolerance >= 0.0;
    const bool validProfile = profile.isValid() && profile.isClosed() && profile.edgeCount() > 0;

    MYVOXEL_ASSERT_MESSAGE(validTolerance, "Revolved modeling profile tolerance must be finite and non-negative.");
    MYVOXEL_ASSERT_MESSAGE(validProfile, "Revolved modeling requires a valid closed Topology_Wire.");

    if (!validTolerance || !validProfile)
    {
        return Topology_Shape();
    }

    std::vector<Foundation::RefPtr<const Geometry_Curve> > profileCurves;

    if (!buildProfileCurves(profile, profileTolerance, profileCurves))
    {
        MYVOXEL_ASSERT_MESSAGE(false, "Revolved modeling requires Line or Arc Edge uses lying in the local XY profile plane.");
        return Topology_Shape();
    }

    const Foundation::RefPtr<Geometry_Revolved> geometry =
        Foundation::makeRef<Geometry_Revolved>(profileCurves, profileTolerance);

    MYVOXEL_ASSERT_MESSAGE(geometry && geometry->isValid(),
                           "Revolved modeling profile must define a closed non-zero-area region entirely on one side of the local Y axis.");

    if (!geometry || !geometry->isValid())
    {
        return Topology_Shape();
    }

    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

/// 空间Shape实例创建

Shape makeRevolved(const Topology_Wire& profile, double profileTolerance)
{
    return Shape(createRevolved(profile, profileTolerance));
}

Shape makeRevolved(const Topology_Wire& profile, double profileTolerance, const MyMath::Matrix4& localToWorld)
{
    return Shape(createRevolved(profile, profileTolerance), localToWorld);
}

Shape makeRevolved(const Wire& profile, double profileTolerance)
{
    MYVOXEL_ASSERT_MESSAGE(profile.isValid(), "Revolved modeling requires a valid Wire instance.");

    if (!profile.isValid())
    {
        return Shape();
    }

    return Shape(createRevolved(profile.topology(), profileTolerance), profile.localToWorld());
}

}
}