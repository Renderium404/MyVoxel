#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"
#include "MyVoxel/Topology/Topology_Edge.h"
#include "MyVoxel/Topology/Topology_Orientation.h"
#include "MyVoxel/Topology/Topology_Vertex.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆Edge测试统一使用弧度制。
const double Tolerance = 1.0e-12; // 圆弧与浮点方向比较使用的测试容差。
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

MyVoxel::Topology_Vertex makeVertex(const MyMath::Vector3& position)
{
    return MyVoxel::Topology_Vertex(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point>(new MyVoxel::Geometry_Point(position)));
}

}

int main()
{
    const MyVoxel::Topology_Edge empty;
    check(!empty.isValid(), "Default edge invalid");
    check(empty.isNull(), "Default edge null");
    check(!static_cast<bool>(empty), "Default edge bool false");
    check(!empty.isSame(empty), "Null edge has no topology identity");
    check(!empty.isForward(), "Invalid edge not forward");
    check(!empty.isReversed(), "Invalid edge not reversed");

    const MyMath::Vector3 pointA(1.0, 2.0, 3.0);
    const MyMath::Vector3 pointB(4.0, 6.0, 15.0);
    const MyVoxel::Topology_Vertex vertexA = makeVertex(pointA);
    const MyVoxel::Topology_Vertex vertexB = makeVertex(pointB);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> lineAB(new MyVoxel::Geometry_Line(pointA, pointB));
    const MyVoxel::Topology_Edge edgeAB(lineAB, 0.0, 1.0, vertexA, vertexB, 0.0);

    check(edgeAB.isValid(), "3D line edge valid");
    check(edgeAB.isForward(), "New edge defaults forward");
    check(edgeAB.orientation() == MyVoxel::Topology_Orientation::Forward, "New edge orientation preserved");
    check(&edgeAB.geometry() == lineAB.get(), "Edge preserves exact curve geometry resource");
    check(edgeAB.parameterStart() == 0.0 && edgeAB.parameterEnd() == 1.0, "Forward edge exposes ascending curve interval");
    check(edgeAB.startVertex().isSame(vertexA), "Forward edge preserves start vertex identity");
    check(edgeAB.endVertex().isSame(vertexB), "Forward edge preserves end vertex identity");
    check(edgeAB.startVertex().isForward() && edgeAB.endVertex().isForward(), "Stored edge vertices normalized forward");
    check(edgeAB.pointAt(0.0).isEqualTo(pointA, 0.0), "Forward edge pointAt zero matches start");
    check(edgeAB.pointAt(0.5).isEqualTo(MyMath::Vector3(2.5, 4.0, 9.0), Tolerance), "Forward edge pointAt half correct");
    check(edgeAB.pointAt(1.0).isEqualTo(pointB, 0.0), "Forward edge pointAt one matches end");
    check(edgeAB.tangentAt(0.25).isEqualTo(lineAB->tangentAt(0.25), Tolerance), "Forward edge tangent follows geometry");

    const MyVoxel::Topology_Edge copied(edgeAB);
    check(copied.isSame(edgeAB), "Copied edge shares topology identity");
    check(copied.orientation() == edgeAB.orientation(), "Copied edge preserves orientation");

    MyVoxel::Topology_Edge assigned;
    assigned = edgeAB;
    check(assigned.isSame(edgeAB), "Assigned edge shares topology identity");

    const MyVoxel::Topology_Edge independent(lineAB, 0.0, 1.0, vertexA, vertexB, 0.0);
    check(!independent.isSame(edgeAB), "Same definition does not imply same edge topology identity");
    check(&independent.geometry() == &edgeAB.geometry(), "Independent edges may share curve geometry");
    check(independent.startVertex().isSame(edgeAB.startVertex()) && independent.endVertex().isSame(edgeAB.endVertex()),
          "Independent edges may share topology vertices");

    const MyVoxel::Topology_Vertex trimStart = makeVertex(lineAB->pointAt(0.2));
    const MyVoxel::Topology_Vertex trimEnd = makeVertex(lineAB->pointAt(0.8));
    const MyVoxel::Topology_Edge trimmed(lineAB, 0.2, 0.8, trimStart, trimEnd, 0.0);
    check(trimmed.isValid(), "Trimmed edge valid");
    check(trimmed.parameterStart() == 0.2 && trimmed.parameterEnd() == 0.8, "Trimmed edge preserves curve interval");
    check(trimmed.pointAt(0.0).isEqualTo(trimStart.geometry().position(), Tolerance), "Trimmed edge start matches vertex geometry");
    check(trimmed.pointAt(0.5).isEqualTo(lineAB->pointAt(0.5), Tolerance), "Trimmed edge normalized midpoint correct");
    check(trimmed.pointAt(1.0).isEqualTo(trimEnd.geometry().position(), Tolerance), "Trimmed edge end matches vertex geometry");

    const MyVoxel::Topology_Edge reversed = trimmed.reversed();
    check(reversed.isSame(trimmed), "Reversed edge preserves Topology_TEdge identity");
    check(reversed.isReversed(), "Reversed edge flips orientation");
    check(reversed.parameterStart() == 0.8 && reversed.parameterEnd() == 0.2, "Reversed edge exposes descending curve interval");
    check(reversed.startVertex().isSame(trimmed.endVertex()), "Reversed edge swaps start vertex");
    check(reversed.endVertex().isSame(trimmed.startVertex()), "Reversed edge swaps end vertex");
    check(reversed.pointAt(0.0).isEqualTo(trimmed.pointAt(1.0), Tolerance), "Reversed edge pointAt zero matches original end");
    check(reversed.pointAt(0.5).isEqualTo(trimmed.pointAt(0.5), Tolerance), "Reversed edge midpoint preserves geometry");
    check(reversed.pointAt(1.0).isEqualTo(trimmed.pointAt(0.0), Tolerance), "Reversed edge pointAt one matches original start");
    check(reversed.tangentAt(0.0).isEqualTo(-trimmed.tangentAt(1.0), Tolerance), "Reversed edge flips start tangent");

    const MyVoxel::Topology_Edge restored = reversed.reversed();
    check(restored.isSame(trimmed), "Double reversed edge preserves topology identity");
    check(restored.isForward(), "Double reversed edge restores forward orientation");
    check(restored.startVertex().isSame(trimmed.startVertex()), "Double reversed edge restores start vertex");
    check(restored.endVertex().isSame(trimmed.endVertex()), "Double reversed edge restores end vertex");

    const MyVoxel::Topology_Edge fromReversedVertices(lineAB, 0.0, 1.0, vertexA.reversed(), vertexB.reversed(), 0.0);
    check(fromReversedVertices.isValid(), "Edge accepts reversed vertex handles by identity");
    check(fromReversedVertices.startVertex().isSame(vertexA), "Reversed input start preserves vertex identity");
    check(fromReversedVertices.endVertex().isSame(vertexB), "Reversed input end preserves vertex identity");
    check(fromReversedVertices.startVertex().isForward() && fromReversedVertices.endVertex().isForward(),
          "Topology_TEdge removes input vertex use orientation");

    const MyMath::Vector3 pointC(7.0, 10.0, 18.0);
    const MyVoxel::Topology_Vertex vertexC = makeVertex(pointC);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> lineBC(new MyVoxel::Geometry_Line(pointB, pointC));
    const MyVoxel::Topology_Edge edgeBC(lineBC, 0.0, 1.0, vertexB, vertexC, 0.0);
    check(edgeAB.endVertex().isSame(edgeBC.startVertex()), "Adjacent edges connect by exact shared vertex identity");

    const MyVoxel::Topology_Vertex duplicateB = makeVertex(pointB);
    const MyVoxel::Topology_Edge disconnectedBC(lineBC, 0.0, 1.0, duplicateB, vertexC, 0.0);
    check(!edgeAB.endVertex().isSame(disconnectedBC.startVertex()), "Equal coordinates do not create edge connectivity");

    const MyVoxel::Topology_Vertex tolerantStart = makeVertex(pointA + MyMath::Vector3(0.0, 1.0e-7, 0.0));
    const MyVoxel::Topology_Edge tolerantEdge(lineAB, 0.0, 1.0, tolerantStart, vertexB, 1.0e-6);
    check(tolerantEdge.isValid(), "Geometry tolerance permits nearby endpoint geometry");
    check(tolerantEdge.startVertex().isSame(tolerantStart), "Geometry tolerance does not replace topology vertex identity");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> circle(
        new MyVoxel::Geometry_Arc(MyMath::Vector3(3.0, 4.0, 5.0), 2.0, 0.0, Pi * 2.0));
    const MyVoxel::Topology_Vertex circleVertex = makeVertex(circle->pointAt(0.0));
    const MyVoxel::Topology_Edge circleEdge(circle, 0.0, 1.0, circleVertex, circleVertex, Tolerance);
    check(circleEdge.isValid(), "Full circle edge valid");
    check(circleEdge.startVertex().isSame(circleEdge.endVertex()), "Full circle edge may share one endpoint topology identity");
    check(circleEdge.pointAt(0.0).distanceTo(circleEdge.pointAt(1.0)) <= Tolerance, "Full circle edge closes geometrically");
    check(circleEdge.reversed().isSame(circleEdge), "Reversing full circle edge preserves topology identity");

#ifdef NDEBUG
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> nullGeometry;
    const MyVoxel::Topology_Edge nullEdge(nullGeometry, 0.0, 1.0, vertexA, vertexB, 0.0);
    check(nullEdge.isNull(), "Null curve geometry rejected");

    const MyVoxel::Topology_Edge mismatch(lineAB, 0.0, 1.0, vertexA, vertexC, 0.0);
    check(mismatch.isNull(), "Endpoint geometry mismatch rejected");

    check(empty.reversed().isNull(), "Reversing invalid edge returns null");
#endif

    std::cout << "Topology_EdgeV2 Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
