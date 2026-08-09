#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"
#include "MyVoxel/Topology/Topology_Edge.h"
#include "MyVoxel/Topology/Topology_Vertex.h"
#include "MyVoxel/Topology/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆闭合Wire测试统一使用弧度制。
const double Tolerance = 1.0e-12; // 完整圆首尾浮点误差检查使用的测试容差。
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

MyVoxel::Topology_Edge makeLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(
        new MyVoxel::Geometry_Line(startVertex.geometry().position(), endVertex.geometry().position()));
    return MyVoxel::Topology_Edge(geometry, 0.0, 1.0, startVertex, endVertex, 0.0);
}

}

int main()
{
    /// 空句柄

    const MyVoxel::Topology_Wire empty;
    check(!empty.isValid(), "Default wire invalid");
    check(empty.isNull(), "Default wire null");
    check(!static_cast<bool>(empty), "Default wire bool false");
    check(!empty.isClosed(), "Default wire not closed");
    check(empty.edgeCount() == 0, "Default wire has zero edges");
    check(!empty.isSame(empty), "Null wire has no topology identity");
    check(!empty.isForward(), "Invalid wire not forward");
    check(!empty.isReversed(), "Invalid wire not reversed");

    /// 开放Wire

    const MyVoxel::Topology_Vertex vertexA = makeVertex(MyMath::Vector3(0.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex vertexB = makeVertex(MyMath::Vector3(4.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex vertexC = makeVertex(MyMath::Vector3(4.0, 3.0, 1.0));
    const MyVoxel::Topology_Edge edgeAB = makeLineEdge(vertexA, vertexB);
    const MyVoxel::Topology_Edge edgeBC = makeLineEdge(vertexB, vertexC);

    std::vector<MyVoxel::Topology_Edge> openEdges;
    openEdges.push_back(edgeAB);
    openEdges.push_back(edgeBC);
    const MyVoxel::Topology_Wire openWire(openEdges);

    check(openWire.isValid(), "Connected open wire valid");
    check(openWire.isForward(), "New wire defaults forward");
    check(!openWire.isClosed(), "Connected open wire remains open");
    check(openWire.edgeCount() == 2, "Open wire preserves edge count");
    check(openWire.edges().size() == 2, "Open wire exposes oriented edge sequence");
    check(openWire.edge(0).isSame(edgeAB), "Open wire preserves first edge identity");
    check(openWire.edge(1).isSame(edgeBC), "Open wire preserves second edge identity");
    check(openWire.edge(0).orientation() == edgeAB.orientation(), "Open wire preserves first edge orientation");
    check(openWire.edge(1).orientation() == edgeBC.orientation(), "Open wire preserves second edge orientation");
    check(openWire.startVertex().isSame(vertexA), "Open wire start vertex identity preserved");
    check(openWire.endVertex().isSame(vertexC), "Open wire end vertex identity preserved");
    check(openWire.edge(0).endVertex().isSame(openWire.edge(1).startVertex()), "Open wire adjacency uses exact shared vertex identity");

    /// Topology_TWire身份

    const MyVoxel::Topology_Wire copied(openWire);
    check(copied.isSame(openWire), "Copied wire shares Topology_TWire identity");
    check(copied.orientation() == openWire.orientation(), "Copied wire preserves orientation");

    MyVoxel::Topology_Wire assigned;
    assigned = openWire;
    check(assigned.isSame(openWire), "Assigned wire shares Topology_TWire identity");

    const MyVoxel::Topology_Wire independent(openEdges);
    check(independent.isValid(), "Independent same-definition wire valid");
    check(!independent.isSame(openWire), "Same definition does not imply same wire topology identity");
    check(independent.edge(0).isSame(openWire.edge(0)), "Independent wires may share first edge identity");
    check(independent.edge(1).isSame(openWire.edge(1)), "Independent wires may share second edge identity");

    /// Edge方向直接参与Wire规范Forward定义

    const MyVoxel::Topology_Edge edgeCB = makeLineEdge(vertexC, vertexB);
    std::vector<MyVoxel::Topology_Edge> orientedEdges;
    orientedEdges.push_back(edgeAB);
    orientedEdges.push_back(edgeCB.reversed());
    const MyVoxel::Topology_Wire orientedWire(orientedEdges);

    check(orientedWire.isValid(), "Reversed edge may complete wire connectivity");
    check(orientedWire.edge(1).isReversed(), "Wire preserves child edge orientation");
    check(orientedWire.edge(0).endVertex().isSame(orientedWire.edge(1).startVertex()), "Reversed child edge connects by oriented vertices");
    check(orientedWire.endVertex().isSame(vertexC), "Reversed child edge exposes expected wire end vertex");

    /// Wire整体方向属于Topology_Wire句柄

    const MyVoxel::Topology_Wire reversedOpen = openWire.reversed();
    check(reversedOpen.isValid(), "Reversed open wire valid");
    check(reversedOpen.isSame(openWire), "Reversed wire preserves Topology_TWire identity");
    check(reversedOpen.isReversed(), "Reversed wire flips orientation");
    check(!reversedOpen.isClosed(), "Reversed open wire remains open");
    check(reversedOpen.edgeCount() == openWire.edgeCount(), "Reversed wire preserves edge count");
    check(reversedOpen.startVertex().isSame(openWire.endVertex()), "Reversed wire swaps start vertex");
    check(reversedOpen.endVertex().isSame(openWire.startVertex()), "Reversed wire swaps end vertex");
    check(reversedOpen.edge(0).isSame(openWire.edge(1)), "Reversed wire reverses first edge identity");
    check(reversedOpen.edge(1).isSame(openWire.edge(0)), "Reversed wire reverses second edge identity");
    check(reversedOpen.edge(0).orientation() != openWire.edge(1).orientation(), "Reversed wire flips first resulting edge orientation");
    check(reversedOpen.edge(1).orientation() != openWire.edge(0).orientation(), "Reversed wire flips second resulting edge orientation");
    check(reversedOpen.edge(0).startVertex().isSame(openWire.edge(1).endVertex()), "Reversed first edge exposes original end as start");
    check(reversedOpen.edge(1).endVertex().isSame(openWire.edge(0).startVertex()), "Reversed last edge exposes original start as end");

    const std::vector<MyVoxel::Topology_Edge> reversedEdges = reversedOpen.edges();
    check(reversedEdges.size() == 2, "Reversed edges list preserves count");
    check(reversedEdges[0].isSame(edgeBC) && reversedEdges[0].isReversed(), "Reversed edges list exposes reversed last edge");
    check(reversedEdges[1].isSame(edgeAB) && reversedEdges[1].isReversed(), "Reversed edges list exposes reversed first edge");

    const MyVoxel::Topology_Wire restored = reversedOpen.reversed();
    check(restored.isSame(openWire), "Double reversed wire preserves topology identity");
    check(restored.isForward(), "Double reversed wire restores forward orientation");
    check(restored.edge(0).isSame(openWire.edge(0)), "Double reverse restores first edge identity");
    check(restored.edge(0).orientation() == openWire.edge(0).orientation(), "Double reverse restores first edge orientation");
    check(restored.startVertex().isSame(openWire.startVertex()), "Double reverse restores start vertex");
    check(restored.endVertex().isSame(openWire.endVertex()), "Double reverse restores end vertex");

    /// 闭合Wire

    const MyVoxel::Topology_Vertex vertexD = makeVertex(MyMath::Vector3(0.0, 3.0, 1.0));
    const MyVoxel::Topology_Edge edgeCD = makeLineEdge(vertexC, vertexD);
    const MyVoxel::Topology_Edge edgeDA = makeLineEdge(vertexD, vertexA);
    std::vector<MyVoxel::Topology_Edge> closedEdges;
    closedEdges.push_back(edgeAB);
    closedEdges.push_back(edgeBC);
    closedEdges.push_back(edgeCD);
    closedEdges.push_back(edgeDA);
    const MyVoxel::Topology_Wire closedWire(closedEdges);

    check(closedWire.isValid(), "Connected closed wire valid");
    check(closedWire.isClosed(), "Closed wire detected by shared vertex identity");
    check(closedWire.edgeCount() == 4, "Closed wire preserves all edges");
    check(closedWire.startVertex().isSame(closedWire.endVertex()), "Closed wire start and end share topology identity");
    check(closedWire.startVertex().isSame(vertexA), "Closed wire retains canonical start vertex");

    const MyVoxel::Topology_Wire reversedClosed = closedWire.reversed();
    check(reversedClosed.isSame(closedWire), "Reversed closed wire preserves topology identity");
    check(reversedClosed.isClosed(), "Reversing closed wire preserves closure");
    check(reversedClosed.edgeCount() == closedWire.edgeCount(), "Reversing closed wire preserves edge count");

    /// 单Edge完整圆Wire

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> circleGeometry(
        new MyVoxel::Geometry_Arc(MyMath::Vector3(2.0, 3.0, 4.0), 2.0, 0.0, Pi * 2.0));
    const MyVoxel::Topology_Vertex circleVertex = makeVertex(circleGeometry->pointAt(0.0));
    const MyVoxel::Topology_Edge circleEdge(circleGeometry, 0.0, 1.0, circleVertex, circleVertex, Tolerance);
    std::vector<MyVoxel::Topology_Edge> circleEdges;
    circleEdges.push_back(circleEdge);
    const MyVoxel::Topology_Wire circleWire(circleEdges);

    check(circleWire.isValid(), "Single closed edge wire valid");
    check(circleWire.isClosed(), "Single closed edge forms closed wire");
    check(circleWire.edgeCount() == 1, "Single closed edge wire count preserved");
    check(circleWire.startVertex().isSame(circleWire.endVertex()), "Single closed edge wire shares endpoint identity");
    check(circleWire.reversed().edge(0).isSame(circleEdge), "Reversing single-edge wire preserves edge identity");
    check(circleWire.reversed().edge(0).isReversed(), "Reversing single-edge wire flips edge orientation");

#ifdef NDEBUG
    /// Release下非法拓扑序列返回空句柄，Debug通过断言直接暴露调用错误。

    const std::vector<MyVoxel::Topology_Edge> noEdges;
    check(MyVoxel::Topology_Wire(noEdges).isNull(), "Empty edge sequence rejected");

    std::vector<MyVoxel::Topology_Edge> invalidEdges;
    invalidEdges.push_back(MyVoxel::Topology_Edge());
    check(MyVoxel::Topology_Wire(invalidEdges).isNull(), "Invalid edge rejected");

    const MyVoxel::Topology_Vertex independentB = makeVertex(vertexB.geometry().position());
    const MyVoxel::Topology_Edge independentBC = makeLineEdge(independentB, vertexC);
    std::vector<MyVoxel::Topology_Edge> sameCoordinateDisconnected;
    sameCoordinateDisconnected.push_back(edgeAB);
    sameCoordinateDisconnected.push_back(independentBC);
    check(MyVoxel::Topology_Wire(sameCoordinateDisconnected).isNull(), "Same coordinates without shared vertex identity rejected");

    const MyVoxel::Topology_Vertex vertexE = makeVertex(MyMath::Vector3(20.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex vertexF = makeVertex(MyMath::Vector3(21.0, 0.0, 0.0));
    const MyVoxel::Topology_Edge edgeEF = makeLineEdge(vertexE, vertexF);
    std::vector<MyVoxel::Topology_Edge> disconnectedEdges;
    disconnectedEdges.push_back(edgeAB);
    disconnectedEdges.push_back(edgeEF);
    check(MyVoxel::Topology_Wire(disconnectedEdges).isNull(), "Disconnected edge sequence rejected");

    check(empty.reversed().isNull(), "Reversing invalid wire returns null");
    check(empty.edge(0).isNull(), "Invalid wire edge access returns null in Release");
#endif

    std::cout << "Topology_WireV2 Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
