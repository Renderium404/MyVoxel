#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "MyVoxel/Geometry/Curve/ArcCurve.h"
#include "MyVoxel/Geometry/Curve/CurveLoop.h"
#include "MyVoxel/Geometry/Curve/LineCurve.h"
#include "MyVoxel/Geometry/Revolved/RevolvedGeometry.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeQuery.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，测试圆弧统一使用弧度制。
const double TestTolerance = 1.0e-10; // 双精度曲线和回转查询测试使用的比较误差。

int g_passed = 0;
int g_failed = 0;

void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[通过] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[失败] " << name << std::endl;
    }
}

bool nearlyEqual(double first, double second, double tolerance = TestTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

bool nearlyEqual(const MyMath::Vector3& first, const MyMath::Vector3& second, double tolerance = TestTolerance)
{
    return first.isEqualTo(second, tolerance);
}

MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve> line(double startX, double startY, double endX, double endY)
{
    return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>(new MyVoxel::Geometry::LineCurve(MyMath::Vector3(startX, startY, 0.0), MyMath::Vector3(endX, endY, 0.0)));
}

MyVoxel::Geometry::CurveLoop rectangleProfile(double minimumX, double maximumX, double minimumY, double maximumY)
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>> curves;
    curves.push_back(line(minimumX, minimumY, maximumX, minimumY));
    curves.push_back(line(maximumX, minimumY, maximumX, maximumY));
    curves.push_back(line(maximumX, maximumY, minimumX, maximumY));
    curves.push_back(line(minimumX, maximumY, minimumX, minimumY));
    return MyVoxel::Geometry::CurveLoop(curves, TestTolerance);
}

void testLineCurve()
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve> curve = line(-1.0, 2.0, 3.0, 5.0);
    check(curve->kind() == MyVoxel::Geometry::CurveKind::Line, "LineCurve类型");
    check(nearlyEqual(curve->length(), 5.0), "LineCurve长度");
    check(nearlyEqual(curve->pointAt(0.5), MyMath::Vector3(1.0, 3.5, 0.0)), "LineCurve参数点");
    check(nearlyEqual(curve->tangentAt(0.25), MyMath::Vector3(0.8, 0.6, 0.0)), "LineCurve单位切线");
    check(nearlyEqual(curve->bounds().minimum(), MyMath::Vector3(-1.0, 2.0, 0.0)) && nearlyEqual(curve->bounds().maximum(), MyMath::Vector3(3.0, 5.0, 0.0)), "LineCurve包围盒");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve> reversed = curve->reversed();
    check(nearlyEqual(reversed->startPoint(), curve->endPoint()) && nearlyEqual(reversed->endPoint(), curve->startPoint()), "LineCurve反向");
}

void testArcCurve()
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve> base(new MyVoxel::Geometry::ArcCurve(MyMath::Vector3(1.0, 2.0, 0.0), 2.0, 0.0, Pi * 0.5));
    const MyVoxel::Geometry::ArcCurve& arc = static_cast<const MyVoxel::Geometry::ArcCurve&>(*base);
    const double rootTwo = std::sqrt(2.0);

    check(arc.kind() == MyVoxel::Geometry::CurveKind::Arc, "ArcCurve类型");
    check(nearlyEqual(arc.length(), Pi), "ArcCurve长度");
    check(nearlyEqual(arc.startPoint(), MyMath::Vector3(3.0, 2.0, 0.0)), "ArcCurve起点");
    check(nearlyEqual(arc.endPoint(), MyMath::Vector3(1.0, 4.0, 0.0)), "ArcCurve终点");
    check(nearlyEqual(arc.pointAt(0.5), MyMath::Vector3(1.0 + rootTwo, 2.0 + rootTwo, 0.0)), "ArcCurve参数点");
    check(nearlyEqual(arc.tangentAt(0.0), MyMath::Vector3(0.0, 1.0, 0.0)), "ArcCurve单位切线");
    check(nearlyEqual(arc.bounds().minimum(), MyMath::Vector3(1.0, 2.0, 0.0)) && nearlyEqual(arc.bounds().maximum(), MyMath::Vector3(3.0, 4.0, 0.0)), "ArcCurve包围盒");
    check(arc.containsAngle(Pi * 0.25) && !arc.containsAngle(Pi), "ArcCurve角度范围");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve> reversed = arc.reversed();
    check(nearlyEqual(reversed->startPoint(), arc.endPoint()) && nearlyEqual(reversed->endPoint(), arc.startPoint()), "ArcCurve反向");
}

void testCurveLoop()
{
    const MyVoxel::Geometry::CurveLoop loop = rectangleProfile(0.0, 3.0, -2.0, 2.0);
    check(loop.isValid(), "CurveLoop有效性");
    check(loop.curveCount() == 4, "CurveLoop曲线数量");
    check(loop.isCounterClockwise(), "CurveLoop方向");
    check(nearlyEqual(loop.signedArea(), 12.0), "CurveLoop精确面积");
    check(loop.containsPoint(MyMath::Vector3(1.0, 0.0, 0.0)), "CurveLoop内部点");
    check(loop.containsPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "CurveLoop边界点");
    check(!loop.containsPoint(MyMath::Vector3(4.0, 0.0, 0.0)), "CurveLoop外部点");

    const MyVoxel::Bounds3 inside(MyMath::Vector3(0.5, -0.5, 0.0), MyMath::Vector3(1.5, 0.5, 0.0));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(4.0, -0.5, 0.0), MyMath::Vector3(5.0, 0.5, 0.0));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(2.5, -0.5, 0.0), MyMath::Vector3(3.5, 0.5, 0.0));
    check(loop.classifyBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside, "CurveLoop内部矩形");
    check(loop.classifyBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside, "CurveLoop外部矩形");
    check(loop.classifyBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting, "CurveLoop相交矩形");

    const MyVoxel::Geometry::CurveLoop reversed = loop.reversed();
    check(reversed.isValid() && !reversed.isCounterClockwise() && nearlyEqual(reversed.signedArea(), -12.0), "CurveLoop整体反向");
    check(reversed.containsPoint(MyMath::Vector3(1.0, 0.0, 0.0)), "CurveLoop反向后区域不变");
}

void testArcLoop()
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>> curves;
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>(new MyVoxel::Geometry::ArcCurve(MyMath::Vector3(3.0, 0.0, 0.0), 1.0, 0.0, Pi)));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>(new MyVoxel::Geometry::ArcCurve(MyMath::Vector3(3.0, 0.0, 0.0), 1.0, Pi, Pi)));
    const MyVoxel::Geometry::CurveLoop loop(curves, TestTolerance);

    check(loop.isValid(), "圆弧CurveLoop有效性");
    check(nearlyEqual(loop.signedArea(), Pi, 1.0e-9), "圆弧CurveLoop精确面积");
    check(loop.containsPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "圆弧CurveLoop内部点");
    check(loop.containsPoint(MyMath::Vector3(4.0, 0.0, 0.0)), "圆弧CurveLoop边界点");
    check(!loop.containsPoint(MyMath::Vector3(1.5, 0.0, 0.0)), "圆弧CurveLoop外部点");
}

void testRevolvedGeometry()
{
    const MyVoxel::Geometry::CurveLoop profile = rectangleProfile(0.0, 3.0, -2.0, 2.0);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::RevolvedGeometry> geometry = MyVoxel::Foundation::makeRef<MyVoxel::Geometry::RevolvedGeometry>(profile);

    check(geometry->kind() == MyVoxel::Geometry::ShapeKind::Revolved, "RevolvedGeometry类型");
    check(nearlyEqual(geometry->localBounds().minimum(), MyMath::Vector3(-3.0, -3.0, -2.0)) && nearlyEqual(geometry->localBounds().maximum(), MyMath::Vector3(3.0, 3.0, 2.0)), "RevolvedGeometry包围盒");
    check(geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), "RevolvedGeometry轴上内部点");
    check(geometry->containsLocalPoint(MyMath::Vector3(2.0, 0.0, 1.0)), "RevolvedGeometry普通内部点");
    check(geometry->containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "RevolvedGeometry侧面边界点");
    check(!geometry->containsLocalPoint(MyMath::Vector3(3.1, 0.0, 0.0)), "RevolvedGeometry径向外部点");
    check(!geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 2.1)), "RevolvedGeometry轴向外部点");

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-0.5, -0.5, -0.5), MyMath::Vector3(0.5, 0.5, 0.5));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(4.0, -0.5, -0.5), MyMath::Vector3(5.0, 0.5, 0.5));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(2.5, -0.5, -0.5), MyMath::Vector3(3.5, 0.5, 0.5));
    check(geometry->classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside, "RevolvedGeometry内部包围盒");
    check(geometry->classifyLocalBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside, "RevolvedGeometry外部包围盒");
    check(geometry->classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting, "RevolvedGeometry相交包围盒");
    check(geometry->classifyLocalBoundsFast(inside.center(), inside.extent()) == geometry->classifyLocalBounds(inside), "RevolvedGeometry快速分类一致性");

    const MyVoxel::Geometry::CurveLoop negativeProfile = rectangleProfile(-3.0, 0.0, -2.0, 2.0);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::RevolvedGeometry> negativeGeometry = MyVoxel::Foundation::makeRef<MyVoxel::Geometry::RevolvedGeometry>(negativeProfile);
    check(negativeGeometry->radialSign() == -1.0 && negativeGeometry->containsLocalPoint(MyMath::Vector3(2.0, 0.0, 0.0)), "RevolvedGeometry负侧轮廓");

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::ShapeGeometry> shapeGeometry = geometry;
    const MyVoxel::Geometry::Shape shape(shapeGeometry);
    const MyVoxel::Geometry::ShapeQuery query(shape);
    check(shape.isValid() && query.isValid() && query.containsPoint(MyMath::Vector3(1.0, 1.0, 0.0)), "RevolvedGeometry接入ShapeQuery");
}

void testTorusProfile()
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>> curves;
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>(new MyVoxel::Geometry::ArcCurve(MyMath::Vector3(3.0, 0.0, 0.0), 1.0, 0.0, Pi)));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::Curve>(new MyVoxel::Geometry::ArcCurve(MyMath::Vector3(3.0, 0.0, 0.0), 1.0, Pi, Pi)));
    const MyVoxel::Geometry::CurveLoop profile(curves, TestTolerance);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::RevolvedGeometry> geometry = MyVoxel::Foundation::makeRef<MyVoxel::Geometry::RevolvedGeometry>(profile);

    check(geometry->containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "圆环回转几何内部点");
    check(geometry->containsLocalPoint(MyMath::Vector3(4.0, 0.0, 0.0)), "圆环回转几何边界点");
    check(!geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), "圆环回转几何中心孔");
    check(!geometry->containsLocalPoint(MyMath::Vector3(4.1, 0.0, 0.0)), "圆环回转几何外部点");
}

}

int main()
{
    std::cout << "开始曲线、闭合轮廓和完整回转几何测试" << std::endl << std::endl;

    testLineCurve();
    testArcCurve();
    testCurveLoop();
    testArcLoop();
    testRevolvedGeometry();
    testTorusProfile();

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
