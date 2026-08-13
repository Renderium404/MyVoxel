#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆Feature测试使用的圆周率。
const double Tolerance = 1.0e-12; // 圆弧闭合端点和空间查询使用的小容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 使用共享拓扑端点建立直线Feature Edge。
MyVoxel::Topology_Edge makeLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(
        new MyVoxel::Geometry_Line(startVertex.point(), endVertex.point()));
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, 0.0);
}

// 建立以同一个Topology_Vertex作为首尾身份的完整圆Feature Edge。
MyVoxel::Topology_Edge makeCircleEdge(const MyMath::Vector3& center, double radius)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Arc> arc(new MyVoxel::Geometry_Arc(center, radius, 0.0, Pi * 2.0));
    const MyVoxel::Topology_Vertex seamVertex(arc->startPoint());
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry = arc;
    return MyVoxel::Topology_Edge(seamVertex, seamVertex, geometry, Tolerance);
}

}

int main()
{
    std::cout << "MyVoxel VoxelFeatureSet F1 test" << std::endl << std::endl;

    /// 空集合

    MyVoxel::VoxelFeatureSet features;
    check(features.isValid(), "Empty feature set valid");
    check(features.isEmpty(), "Empty feature set empty");
    check(features.vertexCount() == 0 && features.edgeCount() == 0, "Empty feature counts zero");
    check(!features.bounds().isValid(), "Empty feature bounds invalid");

    /// 独立特征点和Topology身份

    const MyVoxel::Topology_Vertex isolated(MyMath::Vector3(0.0, 0.0, 3.0));
    check(features.addVertex(isolated), "Add isolated feature vertex");
    check(!features.addVertex(isolated), "Duplicate vertex identity rejected");
    check(features.containsVertex(isolated), "Feature set contains vertex identity");
    check(features.vertexCount() == 1 && features.edgeCount() == 0, "Isolated vertex counted");
    check(features.isValid(), "Feature set valid after isolated vertex");

    /// 开放特征链

    const MyVoxel::Topology_Vertex a(MyMath::Vector3(-2.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex b(MyMath::Vector3(0.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex c(MyMath::Vector3(2.0, 1.0, 0.0));
    const MyVoxel::Topology_Edge ab = makeLineEdge(a, b);
    const MyVoxel::Topology_Edge bc = makeLineEdge(b, c);

    check(features.addEdge(ab), "Add first line feature");
    check(features.addEdge(bc), "Add second line feature");
    check(features.containsEdge(ab) && features.containsEdge(bc), "Feature set contains line identities");
    check(features.containsEdge(ab.reversed()), "Reversed Edge is same feature identity");
    check(!features.addEdge(ab.reversed()), "Reversed duplicate Edge rejected");
    check(features.vertexCount() == 4 && features.edgeCount() == 2, "Edge insertion automatically preserves endpoint vertices");
    check(features.incidentEdgeCount(a) == 1, "Open chain endpoint has one incident Edge");
    check(features.incidentEdgeCount(b) == 2, "Open chain junction has two incident Edges");

    std::vector<MyVoxel::Topology_Edge> incident;
    features.incidentEdges(b, incident);
    check(incident.size() == 2 && incident[0].isSame(ab) && incident[1].isSame(bc), "Incident Edge query preserves topology identities");
    check(!features.removeVertex(b), "Referenced feature vertex cannot be removed");

    /// 空间查询

    std::vector<MyVoxel::Topology_Vertex> queriedVertices;
    std::vector<MyVoxel::Topology_Edge> queriedEdges;
    const MyVoxel::Bounds3 leftQuery(MyMath::Vector3(-2.1, -0.1, -0.1), MyMath::Vector3(-0.5, 0.1, 0.1));
    features.queryVertices(leftQuery, queriedVertices);
    features.queryEdges(leftQuery, queriedEdges);
    check(queriedVertices.size() == 1 && queriedVertices[0].isSame(a), "Vertex bounds query finds exact local feature");
    check(queriedEdges.size() == 1 && queriedEdges[0].isSame(ab), "Edge bounds query finds intersecting feature");

    /// 闭合单Edge完整圆

    const MyVoxel::Topology_Edge circle = makeCircleEdge(MyMath::Vector3(5.0, 0.0, 0.0), 1.5);
    check(circle.startVertex().isSame(circle.endVertex()), "Full circle Edge shares one seam Topology_Vertex");
    check(features.addEdge(circle), "Add closed full-circle feature");
    check(features.incidentEdgeCount(circle.startVertex()) == 1, "Closed self-loop counts as one incident Edge");
    check(features.containsEdge(circle.reversed()), "Closed reversed Edge preserves feature identity");
    check(features.isValid(), "Feature set valid with open and closed features");

    /// 删除顺序与缓存范围

    check(features.removeEdge(ab), "Remove first line feature");
    check(features.removeVertex(a), "Unreferenced endpoint can be removed");
    check(!features.containsEdge(ab) && !features.containsVertex(a), "Removed topology identities absent");
    check(features.isValid(), "Feature set remains valid after removals");

    features.clear();
    check(features.isValid() && features.isEmpty(), "Clear restores valid empty feature set");
    check(!features.bounds().isValid(), "Clear resets feature bounds");

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}