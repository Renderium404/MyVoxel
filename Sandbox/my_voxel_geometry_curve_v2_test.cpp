#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Geometry_Object.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧测试统一使用弧度制。
const double Tolerance = 1.0e-12; // 浮点三角函数和三维坐标变换比较使用的测试容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

bool near(double left, double right, double tolerance = Tolerance)
{
    return std::fabs(left - right) <= tolerance;
}

}

int main()
{
    /// Geometry_Curve基础层

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> baseCurve(
        new MyVoxel::Geometry_Line(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(1.0, 2.0, 3.0)));
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Object> geometryObject = baseCurve;
    check(static_cast<bool>(geometryObject), "Geometry_Curve converts to Geometry_Object resource");

    /// 三维Geometry_Line

    const MyMath::Vector3 lineStart(1.0, 2.0, 3.0);
    const MyMath::Vector3 lineEnd(4.0, 6.0, 15.0);
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Line> line(new MyVoxel::Geometry_Line(lineStart, lineEnd));

    check(line->kind() == MyVoxel::CurveKind::Line, "Line kind preserved");
    check(line->startPoint().isEqualTo(lineStart, 0.0), "3D line start preserved");
    check(line->endPoint().isEqualTo(lineEnd, 0.0), "3D line end preserved");
    check(near(line->length(), 13.0), "3D line length correct");
    check(line->pointAt(0.5).isEqualTo(MyMath::Vector3(2.5, 4.0, 9.0), Tolerance), "3D line midpoint correct");
    check(line->tangentAt(0.25).isEqualTo(MyMath::Vector3(3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0), Tolerance), "3D line tangent correct");
    check(line->bounds().minimum().isEqualTo(lineStart, 0.0), "3D line bounds minimum correct");
    check(line->bounds().maximum().isEqualTo(lineEnd, 0.0), "3D line bounds maximum correct");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> reversedLine = line->reversed();
    check(reversedLine->startPoint().isEqualTo(lineEnd, 0.0), "Reversed line swaps start");
    check(reversedLine->endPoint().isEqualTo(lineStart, 0.0), "Reversed line swaps end");
    check(reversedLine->tangentAt(0.5).isEqualTo(-line->tangentAt(0.5), Tolerance), "Reversed line flips tangent");

    /// 世界XY便捷Geometry_Arc

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Arc> xyArc(
        new MyVoxel::Geometry_Arc(MyMath::Vector3(10.0, 20.0, 30.0), 2.0, 0.0, Pi * 0.5));

    check(xyArc->kind() == MyVoxel::CurveKind::Arc, "Arc kind preserved");
    check(xyArc->center().isEqualTo(MyMath::Vector3(10.0, 20.0, 30.0), 0.0), "XY arc center may have arbitrary Z");
    check(xyArc->startPoint().isEqualTo(MyMath::Vector3(12.0, 20.0, 30.0), Tolerance), "XY arc start correct");
    check(xyArc->endPoint().isEqualTo(MyMath::Vector3(10.0, 22.0, 30.0), Tolerance), "XY arc end correct");
    check(xyArc->normal().isEqualTo(MyMath::Vector3::unitZ(), Tolerance), "XY arc normal correct");
    check(near(xyArc->length(), Pi), "XY arc length correct");

    /// 任意三维平面Geometry_Arc

    const MyMath::CoordinateSystem yzSystem = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(10.0, 20.0, 30.0),
        MyMath::Vector3(0.0, 1.0, 0.0),
        MyMath::Vector3(0.0, 0.0, 1.0),
        MyMath::Vector3(1.0, 0.0, 0.0));
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Arc> yzArc(new MyVoxel::Geometry_Arc(yzSystem, 2.0, 0.0, Pi * 0.5));

    check(yzArc->coordinateSystem().isValid(), "3D arc coordinate system preserved");
    check(yzArc->xAxis().isEqualTo(MyMath::Vector3(0.0, 1.0, 0.0), Tolerance), "3D arc local X axis preserved");
    check(yzArc->yAxis().isEqualTo(MyMath::Vector3(0.0, 0.0, 1.0), Tolerance), "3D arc local Y axis preserved");
    check(yzArc->normal().isEqualTo(MyMath::Vector3(1.0, 0.0, 0.0), Tolerance), "3D arc normal preserved");
    check(yzArc->startPoint().isEqualTo(MyMath::Vector3(10.0, 22.0, 30.0), Tolerance), "3D arc start correct");
    check(yzArc->endPoint().isEqualTo(MyMath::Vector3(10.0, 20.0, 32.0), Tolerance), "3D arc end correct");
    check(yzArc->pointAt(0.5).isEqualTo(MyMath::Vector3(10.0, 20.0 + std::sqrt(2.0), 30.0 + std::sqrt(2.0)), Tolerance), "3D arc midpoint correct");
    check(yzArc->tangentAt(0.0).isEqualTo(MyMath::Vector3(0.0, 0.0, 1.0), Tolerance), "3D arc start tangent correct");
    check(near(yzArc->bounds().minimum().x(), 10.0) && near(yzArc->bounds().maximum().x(), 10.0), "3D arc bounds preserve constant world X");
    check(near(yzArc->bounds().minimum().y(), 20.0) && near(yzArc->bounds().maximum().y(), 22.0), "3D arc bounds world Y correct");
    check(near(yzArc->bounds().minimum().z(), 30.0) && near(yzArc->bounds().maximum().z(), 32.0), "3D arc bounds world Z correct");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> reversedArc = yzArc->reversed();
    check(reversedArc->startPoint().isEqualTo(yzArc->endPoint(), Tolerance), "Reversed 3D arc swaps start");
    check(reversedArc->endPoint().isEqualTo(yzArc->startPoint(), Tolerance), "Reversed 3D arc swaps end");
    check(reversedArc->tangentAt(0.0).isEqualTo(-yzArc->tangentAt(1.0), Tolerance), "Reversed 3D arc flips tangent");

    /// 倾斜完整圆三维包围盒

    const double invSqrt2 = std::sqrt(0.5); // 构造绕世界Z轴45度倾斜的正交圆平面基。
    const MyMath::CoordinateSystem tiltedSystem = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(1.0, 2.0, 3.0),
        MyMath::Vector3(invSqrt2, invSqrt2, 0.0),
        MyMath::Vector3(0.0, 0.0, 1.0),
        MyMath::Vector3(invSqrt2, -invSqrt2, 0.0));
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Arc> fullCircle(new MyVoxel::Geometry_Arc(tiltedSystem, 4.0, 0.0, Pi * 2.0));

    check(fullCircle->startPoint().distanceTo(fullCircle->endPoint()) <= Tolerance, "Full 3D circle closes geometrically");
    check(near(fullCircle->bounds().minimum().x(), 1.0 - 4.0 * invSqrt2), "Tilted circle minimum X exact");
    check(near(fullCircle->bounds().maximum().x(), 1.0 + 4.0 * invSqrt2), "Tilted circle maximum X exact");
    check(near(fullCircle->bounds().minimum().y(), 2.0 - 4.0 * invSqrt2), "Tilted circle minimum Y exact");
    check(near(fullCircle->bounds().maximum().y(), 2.0 + 4.0 * invSqrt2), "Tilted circle maximum Y exact");
    check(near(fullCircle->bounds().minimum().z(), -1.0), "Tilted circle minimum Z exact");
    check(near(fullCircle->bounds().maximum().z(), 7.0), "Tilted circle maximum Z exact");

    std::cout << "Geometry_CurveV2 Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
