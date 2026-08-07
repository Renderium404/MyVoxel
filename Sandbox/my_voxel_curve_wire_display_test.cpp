#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Adapter/Display_CurveAdapter.h"
#include "MyVoxel/Display/Adapter/Display_WireAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Modeling/Wire/WireModeling.h"
#include "MyVoxel/Tool/Discretization/CurveDiscretizer.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧离散数量测试使用的圆周率。
std::size_t g_passed = 0;
std::size_t g_failed = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

}

int main()
{
    const MyVoxel::Display_Color color(0.2, 0.7, 0.9, 1.0);

    const MyVoxel::Topology_Curve lineTopology =
        MyVoxel::Modeling::createLine(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(4.0, 2.0, 0.0));
    const std::vector<MyMath::Vector3> linePoints = MyVoxel::CurveDiscretizer::buildSegments(lineTopology);
    check(linePoints.size() == 2, "Line discretizes to one segment");
    check(linePoints.front().isEqualTo(lineTopology.startPoint(), 0.0) &&
          linePoints.back().isEqualTo(lineTopology.endPoint(), 0.0), "Line endpoints preserved");

    const MyVoxel::Topology_Curve fullCircle =
        MyVoxel::Modeling::createArc(MyMath::Vector3(0.0, 0.0, 0.0), 3.0, 0.0, Pi * 2.0);
    check(MyVoxel::CurveDiscretizer::segmentCount(fullCircle) == 64, "Full circle default 64 segments");
    const std::vector<MyMath::Vector3> circlePoints = MyVoxel::CurveDiscretizer::buildSegments(fullCircle);
    check(circlePoints.size() == 128, "Full circle GL_LINES point count");

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_LineResource> resource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_LineResource>(circlePoints, color);
    check(resource && resource->isValid(), "Display_LineResource valid");
    check(resource->segmentCount() == 64 && resource->vertexCount() == 128, "Display_LineResource counts");
    check(resource->memoryByteSize() == resource->vertexCount() * sizeof(MyVoxel::Display_LineVertex), "Display_LineResource memory size");

    const MyMath::Matrix4 transform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -2.0, 3.0));
    const MyVoxel::Curve curve =
        MyVoxel::Modeling::makeArc(MyMath::Vector3(0.0, 0.0, 0.0), 3.0, 0.0, Pi, transform);
    const MyVoxel::Display_LineObjectSnapshot curveSnapshot =
        MyVoxel::Display_CurveAdapter::buildSnapshot(1, curve, MyVoxel::Display_Color::yellow(), 2.0f);
    check(curveSnapshot.isValid(), "Curve display snapshot valid");
    check(curveSnapshot.parts.size() == 1, "Curve display one part");
    check(curveSnapshot.segmentCount() == 32, "Half circle display segment count");
    check(curveSnapshot.localToWorld.isEqualTo(transform, 0.0), "Curve display preserves transform");

    const MyVoxel::Wire rectangle =
        MyVoxel::Modeling::makeRectangle(8.0, 5.0, transform);
    const MyVoxel::Display_LineObjectSnapshot rectangleSnapshot =
        MyVoxel::Display_WireAdapter::buildSnapshot(2, rectangle, MyVoxel::Display_Color::cyan(), 1.5f);
    check(rectangleSnapshot.isValid(), "Rectangle Wire display snapshot valid");
    check(rectangleSnapshot.parts.size() == 4, "Rectangle Wire one part per curve");
    check(rectangleSnapshot.segmentCount() == 4, "Rectangle Wire four line segments");
    check(rectangleSnapshot.localToWorld.isEqualTo(transform, 0.0), "Wire display preserves transform");

    const MyVoxel::Wire circle =
        MyVoxel::Modeling::makeCircle(4.0);
    const MyVoxel::Display_LineObjectSnapshot circleSnapshot =
        MyVoxel::Display_WireAdapter::buildSnapshot(3, circle, MyVoxel::Display_Color::magenta(), 2.0f);
    check(circleSnapshot.isValid(), "Circle Wire display snapshot valid");
    check(circleSnapshot.parts.size() == 1 && circleSnapshot.segmentCount() == 64, "Circle Wire display segments");

    std::cout << "Curve/Wire Display Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
