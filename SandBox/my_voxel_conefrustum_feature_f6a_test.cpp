#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Modeling/Feature/ShapeFeatureExtractor.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆Feature长度验证使用的圆周率。
const double Tolerance = 1.0e-10; // 解析Feature位置、半径和变换结果比较使用的绝对容差。
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

// 返回FeatureSet中唯一的孤立Vertex；不存在或不唯一时返回空Vertex。
MyVoxel::Topology_Vertex isolatedVertex(const MyVoxel::VoxelFeatureSet& features)
{
    MyVoxel::Topology_Vertex result;
    std::size_t count = 0;

    for (std::size_t index = 0; index < features.vertices().size(); ++index)
    {
        if (features.incidentEdgeCount(features.vertices()[index]) == 0)
        {
            result = features.vertices()[index];
            ++count;
        }
    }

    return count == 1 ? result : MyVoxel::Topology_Vertex();
}

// 判断全部Edge都是完整圆Arc并保持自环seam身份。
bool allCircularEdgesValid(const MyVoxel::VoxelFeatureSet& features)
{
    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[index];

        if (edge.geometry().kind() != MyVoxel::CurveKind::Arc || !edge.startVertex().isSame(edge.endVertex()))
        {
            return false;
        }

        if (!edge.pointAt(0.0).isEqualTo(edge.pointAt(1.0), Tolerance) ||
            !edge.tangentAt(0.0).isEqualTo(edge.tangentAt(1.0), Tolerance))
        {
            return false;
        }
    }

    return true;
}

void testFrustumLocalFeatures()
{
    const double bottomRadius = 3.0;
    const double topRadius = 1.5;
    const double height = 6.0;
    const MyVoxel::Topology_Shape shape = MyVoxel::Modeling::createConeFrustum(bottomRadius, topRadius, height);

    check(MyVoxel::Modeling::ShapeFeatureExtractor::supports(shape), "ConeFrustum feature extraction supported");
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);

    check(features.isValid(), "ConeFrustum FeatureSet valid");
    check(features.vertexCount() == 2 && features.edgeCount() == 2, "ConeFrustum has two seam vertices and two circular sharp edges");
    check(allCircularEdgesValid(features), "ConeFrustum circular Features preserve smooth self-loop seams");
    check(features.bounds().isEqualTo(shape.geometry().localBounds(), Tolerance), "ConeFrustum Feature bounds equal analytic shape bounds");

    bool bottomCorrect = false;
    bool topCorrect = false;

    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(features.edges()[index].geometry());
        bottomCorrect = bottomCorrect || (nearValue(arc.center().z(), -height * 0.5) && nearValue(arc.radius(), bottomRadius));
        topCorrect = topCorrect || (nearValue(arc.center().z(), height * 0.5) && nearValue(arc.radius(), topRadius));
    }

    check(bottomCorrect && topCorrect, "ConeFrustum circular Features match both analytic end boundaries");
}

void testConeLocalFeatures()
{
    const double radius = 2.5;
    const double height = 5.0;
    const MyVoxel::Topology_Shape shape = MyVoxel::Modeling::createCone(radius, height);
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);

    check(features.isValid(), "Cone FeatureSet valid");
    check(features.vertexCount() == 2 && features.edgeCount() == 1, "Cone has one circular seam vertex plus one isolated Apex");
    check(allCircularEdgesValid(features), "Cone base Feature is a smooth complete circular Edge");

    const MyVoxel::Topology_Vertex apex = isolatedVertex(features);
    check(apex.isValid(), "Cone contains one isolated Apex FeatureVertex");
    check(apex.point().isEqualTo(MyMath::Vector3(0.0, 0.0, height * 0.5), Tolerance), "Cone Apex is exact analytic top point");
    check(features.incidentEdgeCount(apex) == 0, "Cone Apex is not attached to fictitious zero-radius Edge");
    check(features.bounds().isEqualTo(shape.geometry().localBounds(), Tolerance), "Cone Feature bounds include base circle and Apex");

    const MyVoxel::Geometry_Arc& baseArc = static_cast<const MyVoxel::Geometry_Arc&>(features.edges()[0].geometry());
    check(nearValue(baseArc.center().z(), -height * 0.5) && nearValue(baseArc.radius(), radius), "Cone base circular Feature exact");
    check(nearValue(features.edges()[0].length(), 2.0 * Pi * radius), "Cone base circular Feature length exact");
}

void testInvertedConeLocalFeatures()
{
    const double radius = 1.75;
    const double height = 4.0;
    const MyVoxel::Topology_Shape shape = MyVoxel::Modeling::createConeFrustum(0.0, radius, height);
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape);
    const MyVoxel::Topology_Vertex apex = isolatedVertex(features);

    check(features.vertexCount() == 2 && features.edgeCount() == 1, "Inverted Cone has one circular Edge and one isolated Apex");
    check(apex.isValid() && apex.point().isEqualTo(MyMath::Vector3(0.0, 0.0, -height * 0.5), Tolerance), "Inverted Cone Apex is exact analytic bottom point");

    const MyVoxel::Geometry_Arc& topArc = static_cast<const MyVoxel::Geometry_Arc&>(features.edges()[0].geometry());
    check(nearValue(topArc.center().z(), height * 0.5) && nearValue(topArc.radius(), radius), "Inverted Cone top circular Feature exact");
}

void testQuerySpacePreservesApex()
{
    const double radius = 2.0;
    const double height = 6.0;
    const double scale = 2.5;
    const MyMath::Vector3 translation(7.0, -3.0, 4.0);
    const MyMath::Matrix4 placement = MyMath::Matrix4::fromTranslation(translation) * MyMath::Matrix4::fromScale(scale);
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeCone(radius, height, placement);
    const MyVoxel::ShapeQuery query(shape);

    check(query.supportsSignedDistance() && MyVoxel::Modeling::ShapeFeatureExtractor::supports(query), "Uniform-scaled translated Cone query supports Features");
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);
    const MyVoxel::Topology_Vertex apex = isolatedVertex(features);

    check(features.vertexCount() == 2 && features.edgeCount() == 1, "Query-space Cone preserves isolated Apex and circular Edge");
    check(apex.isValid(), "Query-space transformation does not drop isolated Apex Vertex");

    const MyMath::Vector3 expectedApex = placement.transformPoint(MyMath::Vector3(0.0, 0.0, height * 0.5));
    check(apex.point().isEqualTo(expectedApex, Tolerance), "Query-space Cone Apex follows localToQuery transform exactly");

    const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(features.edges()[0].geometry());
    check(nearValue(arc.radius(), radius * scale), "Query-space Cone circular Feature radius follows uniform scale");
    check(features.bounds().isEqualTo(query.queryBounds(), Tolerance), "Query-space Cone Feature bounds equal analytic query bounds");
}

void testVoxelizationMarksConeFeaturesComplete()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyVoxel::VoxelShape cone = MyVoxel::Modeling::voxelize(MyVoxel::Modeling::createCone(2.0, 4.0), grid);
    const MyVoxel::VoxelShape frustum = MyVoxel::Modeling::voxelize(MyVoxel::Modeling::createConeFrustum(2.5, 1.0, 4.0), grid);

    check(cone.isValid() && cone.hasCompleteFeatures(), "Voxelized Cone now carries Complete Feature state");
    check(cone.features().vertexCount() == 2 && cone.features().edgeCount() == 1, "Voxelized Cone stores Apex and base circle Features");
    check(frustum.isValid() && frustum.hasCompleteFeatures(), "Voxelized ConeFrustum now carries Complete Feature state");
    check(frustum.features().vertexCount() == 2 && frustum.features().edgeCount() == 2, "Voxelized ConeFrustum stores two circular Features");
}

}

int main()
{
    std::cout << "MyVoxel ConeFrustum Feature extraction F6A test" << std::endl << std::endl;
    testFrustumLocalFeatures();
    testConeLocalFeatures();
    testInvertedConeLocalFeatures();
    testQuerySpacePreservesApex();
    testVoxelizationMarksConeFeaturesComplete();
    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}