#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"
#include "MyVoxel/Modeling/Feature/ShapeFeatureExtractor.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆柱完整圆Feature长度验证使用的圆周率。
const double Tolerance = 1.0e-10; // 解析特征位置、长度和包围盒测试使用的绝对误差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 判断两个double是否在测试容差内一致。
bool nearValue(double first, double second)
{
    return std::fabs(first - second) <= Tolerance;
}

// 使用连续Geometry_Shape建立当前正式Topology_Shape包装。
MyVoxel::Topology_Shape makeTopology(MyVoxel::Geometry_Shape* geometry)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape> resource(geometry);
    return MyVoxel::Topology_Shape(resource);
}

// 判断点是否为指定Box八个理论角点之一。
bool isBoxCorner(const MyMath::Vector3& point, double hx, double hy, double hz)
{
    return nearValue(std::fabs(point.x()), hx) && nearValue(std::fabs(point.y()), hy) && nearValue(std::fabs(point.z()), hz);
}

void testBox()
{
    const double sizeX = 2.0;
    const double sizeY = 4.0;
    const double sizeZ = 6.0;
    const MyVoxel::Topology_Shape shape = makeTopology(new MyVoxel::Geometry_Box(sizeX, sizeY, sizeZ));
    check(MyVoxel::Modeling::ShapeFeatureExtractor::supports(shape), "Box feature extraction supported");

    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);
    check(features.isValid(), "Box feature set valid");
    check(features.vertexCount() == 8, "Box has eight topology feature vertices");
    check(features.edgeCount() == 12, "Box has twelve topology feature edges");
    check(features.bounds().isEqualTo(shape.geometry().localBounds(), 0.0), "Box feature bounds equal analytic Box bounds");

    bool allCorners = true;
    bool allDegreeThree = true;

    for (std::size_t index = 0; index < features.vertices().size(); ++index)
    {
        const MyVoxel::Topology_Vertex& vertex = features.vertices()[index];
        allCorners = allCorners && isBoxCorner(vertex.point(), sizeX * 0.5, sizeY * 0.5, sizeZ * 0.5);
        allDegreeThree = allDegreeThree && features.incidentEdgeCount(vertex) == 3;
    }

    check(allCorners, "Box feature vertices are exact analytic corners");
    check(allDegreeThree, "Every Box feature vertex has three incident edges");

    std::size_t xEdges = 0;
    std::size_t yEdges = 0;
    std::size_t zEdges = 0;
    bool allLines = true;

    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[index];
        allLines = allLines && edge.geometry().kind() == MyVoxel::CurveKind::Line;

        if (nearValue(edge.length(), sizeX)){++xEdges;}
        else if (nearValue(edge.length(), sizeY)){++yEdges;}
        else if (nearValue(edge.length(), sizeZ)){++zEdges;}
    }

    check(allLines, "All Box feature edges use Geometry_Line");
    check(xEdges == 4 && yEdges == 4 && zEdges == 4, "Box has four feature edges along each local axis");
}

void testSphere()
{
    const MyVoxel::Topology_Shape shape = makeTopology(new MyVoxel::Geometry_Sphere(3.0));
    check(MyVoxel::Modeling::ShapeFeatureExtractor::supports(shape), "Sphere feature extraction supported");

    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);
    check(features.isValid(), "Sphere feature set valid");
    check(features.isEmpty(), "Smooth Sphere has no explicit sharp features");
    check(features.vertexCount() == 0 && features.edgeCount() == 0, "Sphere feature counts are zero");
    check(!features.bounds().isValid(), "Sphere empty feature bounds invalid");
}

void testCylinder()
{
    const double radius = 2.5;
    const double height = 8.0;
    const MyVoxel::Topology_Shape shape = makeTopology(new MyVoxel::Geometry_Cylinder(radius, height));
    check(MyVoxel::Modeling::ShapeFeatureExtractor::supports(shape), "Cylinder feature extraction supported");

    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);
    check(features.isValid(), "Cylinder feature set valid");
    check(features.edgeCount() == 2, "Cylinder has two circular sharp feature edges");
    check(features.vertexCount() == 2, "Cylinder full circles keep two topology seam vertices");
    check(features.bounds().isEqualTo(shape.geometry().localBounds(), Tolerance), "Cylinder feature bounds equal analytic Cylinder bounds");

    bool allArcs = true;
    bool allClosedByIdentity = true;
    bool allLengths = true;
    bool allSmoothAtSeam = true;
    bool hasBottom = false;
    bool hasTop = false;

    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[index];
        allArcs = allArcs && edge.geometry().kind() == MyVoxel::CurveKind::Arc;
        allClosedByIdentity = allClosedByIdentity && edge.startVertex().isSame(edge.endVertex());
        allLengths = allLengths && nearValue(edge.length(), 2.0 * Pi * radius);

        const MyMath::Vector3 startTangent = edge.tangentAt(0.0);
        const MyMath::Vector3 endTangent = edge.tangentAt(1.0);
        allSmoothAtSeam = allSmoothAtSeam && startTangent.isEqualTo(endTangent, Tolerance);

        const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());
        hasBottom = hasBottom || nearValue(arc.center().z(), -height * 0.5);
        hasTop = hasTop || nearValue(arc.center().z(), height * 0.5);
    }

    check(allArcs, "Cylinder feature edges use Geometry_Arc");
    check(allClosedByIdentity, "Cylinder circles close through shared seam Topology_Vertex identities");
    check(allLengths, "Cylinder circular feature lengths are exact");
    check(allSmoothAtSeam, "Cylinder seam vertices are topological seams rather than geometric corners");
    check(hasBottom && hasTop, "Cylinder features lie on both end-cap boundaries");
}

void testUnsupportedShape()
{
    const MyVoxel::Topology_Shape frustum = makeTopology(new MyVoxel::Geometry_ConeFrustum(3.0, 1.5, 5.0));
    check(!MyVoxel::Modeling::ShapeFeatureExtractor::supports(frustum), "Unsupported ConeFrustum is not mistaken for featureless geometry");
    const MyVoxel::Topology_Shape invalid;
    check(!MyVoxel::Modeling::ShapeFeatureExtractor::supports(invalid), "Invalid Topology_Shape is not supported");
}

}

int main()
{
    std::cout << "MyVoxel ShapeFeatureExtractor F2 test" << std::endl << std::endl;
    testBox();
    testSphere();
    testCylinder();
    testUnsupportedShape();
    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}