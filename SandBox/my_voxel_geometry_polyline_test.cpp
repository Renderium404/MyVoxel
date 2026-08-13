#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Polyline.h"
#include "MyVoxel/Geometry/Geometry_Object.h"

namespace
{

const double Tolerance = 1.0e-12; // 累计弧长参数换算和反向曲线比较使用的浮点容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 判断两个双精度数是否在测试容差内一致。
bool near(double first, double second, double tolerance = Tolerance)
{
    return std::fabs(first - second) <= tolerance;
}

}

int main()
{
    std::cout << "MyVoxel Geometry_Polyline test" << std::endl << std::endl;

    /// 三维开放折线基础语义

    std::vector<MyMath::Vector3> points;
    points.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(3.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(3.0, 4.0, 0.0));
    points.push_back(MyMath::Vector3(3.0, 4.0, 12.0));

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Polyline> polyline(new MyVoxel::Geometry_Polyline(points));
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> curve = polyline;
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Object> geometry = curve;

    check(static_cast<bool>(polyline) && static_cast<bool>(curve) && static_cast<bool>(geometry), "Polyline participates in Geometry resource hierarchy");
    check(polyline->kind() == MyVoxel::CurveKind::Polyline, "Polyline kind preserved");
    check(polyline->pointCount() == 4 && polyline->segmentCount() == 3, "Polyline point and segment counts correct");
    check(polyline->points().size() == 4 && polyline->point(2).isEqualTo(points[2], 0.0), "Polyline control points preserved exactly");
    check(!polyline->isClosed(), "Open polyline not closed");
    check(polyline->startPoint().isEqualTo(points.front(), 0.0) && polyline->endPoint().isEqualTo(points.back(), 0.0), "Polyline endpoints preserved");
    check(near(polyline->segmentLength(0), 3.0) && near(polyline->segmentLength(1), 4.0) && near(polyline->segmentLength(2), 12.0), "Polyline segment lengths correct");
    check(near(polyline->length(), 19.0), "Polyline total length correct");
    check(polyline->bounds().minimum().isEqualTo(MyMath::Vector3(0.0, 0.0, 0.0), 0.0) &&
          polyline->bounds().maximum().isEqualTo(MyMath::Vector3(3.0, 4.0, 12.0), 0.0), "Polyline 3D bounds correct");

    /// 累计弧长参数化

    check(polyline->pointAt(0.0).isEqualTo(points[0], 0.0), "Polyline t=0 returns start point");
    check(polyline->pointAt(1.0).isEqualTo(points[3], 0.0), "Polyline t=1 returns end point");
    check(polyline->pointAt(1.0 / 19.0).isEqualTo(MyMath::Vector3(1.0, 0.0, 0.0), Tolerance), "Polyline parameter uses arc length on first segment");
    check(polyline->pointAt(3.0 / 19.0).isEqualTo(points[1], Tolerance), "Polyline first knot parameter correct");
    check(polyline->pointAt(5.0 / 19.0).isEqualTo(MyMath::Vector3(3.0, 2.0, 0.0), Tolerance), "Polyline parameter uses arc length on second segment");
    check(polyline->pointAt(7.0 / 19.0).isEqualTo(points[2], Tolerance), "Polyline second knot parameter correct");
    check(polyline->pointAt(13.0 / 19.0).isEqualTo(MyMath::Vector3(3.0, 4.0, 6.0), Tolerance), "Polyline parameter uses arc length on third segment");

    check(polyline->tangentAt(0.0).isEqualTo(MyMath::Vector3(1.0, 0.0, 0.0), Tolerance), "Polyline start tangent uses first segment");
    check(polyline->tangentAt(3.0 / 19.0).isEqualTo(MyMath::Vector3(0.0, 1.0, 0.0), Tolerance), "Polyline internal knot tangent uses successor segment");
    check(polyline->tangentAt(7.0 / 19.0).isEqualTo(MyMath::Vector3(0.0, 0.0, 1.0), Tolerance), "Polyline second knot tangent uses successor segment");
    check(polyline->tangentAt(1.0).isEqualTo(MyMath::Vector3(0.0, 0.0, 1.0), Tolerance), "Polyline end tangent uses last segment");

    /// 反向语义

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> reversed = polyline->reversed();
    check(reversed->kind() == MyVoxel::CurveKind::Polyline, "Reversed curve remains Polyline");
    check(reversed->startPoint().isEqualTo(polyline->endPoint(), 0.0) && reversed->endPoint().isEqualTo(polyline->startPoint(), 0.0), "Reversed Polyline swaps endpoints");
    check(near(reversed->length(), polyline->length()), "Reversed Polyline preserves length");
    check(reversed->bounds().isEqualTo(polyline->bounds(), 0.0), "Reversed Polyline preserves bounds");

    const double reverseSamples[4] = {0.1, 0.25, 0.6, 0.9}; // 避开内部折点，因为折点切向在数学上不唯一。
    bool reversePointsCorrect = true;
    bool reverseTangentsCorrect = true;

    for (int index = 0; index < 4; ++index)
    {
        const double t = reverseSamples[index];
        reversePointsCorrect = reversePointsCorrect && reversed->pointAt(t).isEqualTo(polyline->pointAt(1.0 - t), Tolerance);
        reverseTangentsCorrect = reverseTangentsCorrect && reversed->tangentAt(t).isEqualTo(polyline->tangentAt(1.0 - t) * -1.0, Tolerance);
    }

    check(reversePointsCorrect, "Reversed Polyline complements normalized arc-length parameter");
    check(reverseTangentsCorrect, "Reversed Polyline reverses tangent away from knots");

    /// 闭合折线语义

    std::vector<MyMath::Vector3> closedPoints;
    closedPoints.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    closedPoints.push_back(MyMath::Vector3(2.0, 0.0, 0.0));
    closedPoints.push_back(MyMath::Vector3(1.0, 2.0, 1.0));
    closedPoints.push_back(closedPoints.front());

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Polyline> closed(new MyVoxel::Geometry_Polyline(closedPoints));
    check(closed->isClosed(), "Exact repeated endpoint forms closed Polyline");
    check(closed->segmentCount() == 3, "Closed Polyline keeps explicit closing segment");
    check(closed->startPoint().isEqualTo(closed->endPoint(), 0.0), "Closed Polyline start and end points are identical");
    check(closed->pointCount() == closedPoints.size(), "Polyline constructor does not simplify caller points");

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}