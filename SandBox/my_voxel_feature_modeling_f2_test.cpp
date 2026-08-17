#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Geometry/Curve/CurveKind.h"
#include "MyVoxel/Geometry/Curve/Geometry_Polyline.h"
#include "MyVoxel/Modeling/Feature/FeatureModeling.h"
#include "MyVoxel/Modeling/Feature/FeatureTrace.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
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

// 验证开放FeatureTrace到Topology_Edge的完整映射。
void testOpenTrace()
{
    const MyMath::Vector3 p0(0.0, 0.0, 0.0);
    const MyMath::Vector3 p1(2.0, 0.0, 0.0);
    const MyMath::Vector3 p2(2.0, 3.0, 0.0);
    const MyMath::Vector3 p3(2.0, 3.0, 4.0);

    MyVoxel::Modeling::FeatureTrace trace;
    trace.addPoint(p0);
    trace.addPoint(p1);
    trace.addPoint(p2);
    trace.addPoint(p3);

    const MyVoxel::Topology_Edge edge =
        MyVoxel::Modeling::createFeatureEdge(trace);

    check(edge.isValid(),
          "Open FeatureTrace creates valid Topology_Edge");

    check(edge.geometry().kind() == MyVoxel::CurveKind::Polyline,
          "FeatureTrace creates Geometry_Polyline");

    check(!edge.startVertex().isSame(edge.endVertex()),
          "Open FeatureTrace creates distinct endpoint topology identities");

    check(edge.startVertex().point().isEqualTo(p0, 0.0),
          "Open feature Edge start Vertex matches trace start");

    check(edge.endVertex().point().isEqualTo(p3, 0.0),
          "Open feature Edge end Vertex matches trace end");

    check(edge.pointAt(0.0).isEqualTo(p0, 0.0) &&
              edge.pointAt(1.0).isEqualTo(p3, 0.0),
          "Open feature Edge preserves trace endpoints");

    const MyVoxel::Geometry_Polyline& polyline =
        static_cast<const MyVoxel::Geometry_Polyline&>(
            edge.geometry());

    check(polyline.pointCount() == trace.pointCount(),
          "Feature Polyline preserves every trace sample point");

    check(polyline.point(1).isEqualTo(p1, 0.0) &&
              polyline.point(2).isEqualTo(p2, 0.0),
          "Internal trace samples remain Geometry_Polyline points");

    check(polyline.startPoint().isEqualTo(
              edge.startVertex().point(), 0.0) &&
              polyline.endPoint().isEqualTo(
                  edge.endVertex().point(), 0.0),
          "Feature Polyline endpoints match Topology_Edge vertices");

    check(!polyline.isClosed(),
          "Open FeatureTrace creates open Geometry_Polyline");

    check(edge.parameterStart() == 0.0 &&
              edge.parameterEnd() == 1.0,
          "Feature Edge uses complete normalized Polyline domain");

    const MyVoxel::Topology_Edge reversed = edge.reversed();

    check(reversed.isSame(edge),
          "Reversed feature Edge preserves topology identity");

    check(reversed.startVertex().isSame(edge.endVertex()) &&
              reversed.endVertex().isSame(edge.startVertex()),
          "Reversed feature Edge swaps topology endpoints");

    check(reversed.pointAt(0.0).isEqualTo(p3, 0.0) &&
              reversed.pointAt(1.0).isEqualTo(p0, 0.0),
          "Reversed feature Edge reverses trace traversal");
}

// 验证两个点也保持Geometry_Polyline语义，不自动转换为Geometry_Line。
void testTwoPointTrace()
{
    const MyMath::Vector3 p0(-1.0, 2.0, 3.0);
    const MyMath::Vector3 p1(4.0, 2.0, 3.0);

    MyVoxel::Modeling::FeatureTrace trace;
    trace.addPoint(p0);
    trace.addPoint(p1);

    const MyVoxel::Topology_Edge edge =
        MyVoxel::Modeling::createFeatureEdge(trace);

    check(edge.isValid(),
          "Two-point FeatureTrace creates valid feature Edge");

    check(edge.geometry().kind() == MyVoxel::CurveKind::Polyline,
          "Two-point FeatureTrace remains Geometry_Polyline");

    const MyVoxel::Geometry_Polyline& polyline =
        static_cast<const MyVoxel::Geometry_Polyline&>(
            edge.geometry());

    check(polyline.pointCount() == 2 &&
              polyline.segmentCount() == 1,
          "Two-point FeatureTrace creates one Polyline segment");

    check(polyline.point(0).isEqualTo(p0, 0.0) &&
              polyline.point(1).isEqualTo(p1, 0.0),
          "Two-point FeatureTrace preserves exact geometry points");
}

// 验证闭合FeatureTrace建立共享seam Vertex和self-loop Topology_Edge。
void testClosedTrace()
{
    const MyMath::Vector3 p0(0.0, 0.0, 0.0);
    const MyMath::Vector3 p1(3.0, 0.0, 0.0);
    const MyMath::Vector3 p2(3.0, 2.0, 0.0);
    const MyMath::Vector3 p3(0.0, 2.0, 0.0);

    MyVoxel::Modeling::FeatureTrace trace;
    trace.addPoint(p0);
    trace.addPoint(p1);
    trace.addPoint(p2);
    trace.addPoint(p3);
    trace.addPoint(p0);

    const MyVoxel::Topology_Edge edge =
        MyVoxel::Modeling::createFeatureEdge(trace);

    check(edge.isValid(),
          "Closed FeatureTrace creates valid Topology_Edge");

    check(edge.startVertex().isSame(edge.endVertex()),
          "Closed FeatureTrace creates shared seam Topology_Vertex");

    check(edge.startVertex().point().isEqualTo(p0, 0.0),
          "Closed feature seam Vertex matches repeated trace endpoint");

    const MyVoxel::Geometry_Polyline& polyline =
        static_cast<const MyVoxel::Geometry_Polyline&>(
            edge.geometry());

    check(polyline.isClosed(),
          "Closed FeatureTrace creates closed Geometry_Polyline");

    check(polyline.pointCount() == 5 &&
              polyline.segmentCount() == 4,
          "Closed feature Polyline preserves explicit closing sample");

    check(polyline.point(0).isEqualTo(
              polyline.point(polyline.pointCount() - 1), 0.0),
          "Closed feature Polyline keeps identical first and last geometry points");

    check(edge.pointAt(0.0).isEqualTo(
              edge.pointAt(1.0), 0.0),
          "Closed feature Edge is geometrically closed");

    const MyVoxel::Topology_Edge reversed = edge.reversed();

    check(reversed.isSame(edge),
          "Reversed closed feature Edge preserves topology identity");

    check(reversed.startVertex().isSame(
              reversed.endVertex()),
          "Reversed closed feature Edge preserves seam identity");

    check(reversed.pointAt(0.0).isEqualTo(p0, 0.0) &&
              reversed.pointAt(1.0).isEqualTo(p0, 0.0),
          "Reversed closed feature Edge preserves seam position");
}

}

int main()
{
    std::cout << "MyVoxel Feature modeling F2 test"
              << std::endl
              << std::endl;

    testOpenTrace();
    testTwoPointTrace();
    testClosedTrace();

    std::cout << std::endl;
    std::cout << "Passed: "
              << g_passedCount
              << std::endl;
    std::cout << "Failed: "
              << g_failedCount
              << std::endl;

    return g_failedCount == 0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}