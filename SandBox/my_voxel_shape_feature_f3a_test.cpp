#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Modeling/Feature/ShapeFeatureExtractor.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆柱Feature长度验证使用的圆周率。
const double Tolerance = 1.0e-10; // 查询空间解析Feature位置、长度和方向测试使用的绝对误差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 判断double是否在测试容差内一致。
bool nearValue(double first, double second)
{
    return std::fabs(first - second) <= Tolerance;
}

// 判断全部Box特征点是否经过指定矩阵精确映射，且每个点仍保持三条共享Edge邻接。
bool transformedBoxVerticesCorrect(const MyVoxel::VoxelFeatureSet& source, const MyVoxel::VoxelFeatureSet& target, const MyMath::Matrix4& transform)
{
    if (source.vertexCount() != target.vertexCount())
    {
        return false;
    }

    for (std::size_t sourceIndex = 0; sourceIndex < source.vertices().size(); ++sourceIndex)
    {
        const MyMath::Vector3 expected = transform.transformPoint(source.vertices()[sourceIndex].point());
        bool found = false;

        for (std::size_t targetIndex = 0; targetIndex < target.vertices().size(); ++targetIndex)
        {
            if (target.vertices()[targetIndex].point().isEqualTo(expected, Tolerance))
            {
                found = target.incidentEdgeCount(target.vertices()[targetIndex]) == 3;
                break;
            }
        }

        if (!found)
        {
            return false;
        }
    }

    return true;
}

void testIdentityQuery()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(2.0, 4.0, 6.0);
    const MyVoxel::ShapeQuery query(topology);
    const MyVoxel::VoxelFeatureSet local = MyVoxel::Modeling::ShapeFeatureExtractor::extract(topology);
    const MyVoxel::VoxelFeatureSet queried = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);

    check(MyVoxel::Modeling::ShapeFeatureExtractor::supports(query), "Identity Box ShapeQuery feature extraction supported");
    check(queried.isValid() && queried.vertexCount() == 8 && queried.edgeCount() == 12, "Identity Box query keeps F2 feature topology");
    check(queried.bounds().isEqualTo(local.bounds(), 0.0), "Identity query feature bounds unchanged");
    check(transformedBoxVerticesCorrect(local, queried, MyMath::Matrix4::identity()), "Identity query preserves exact Box corners and degree-three adjacency");
}

void testTranslatedQuery()
{
    const MyMath::Matrix4 placement = MyMath::Matrix4::fromTranslation(MyMath::Vector3(7.0, -3.0, 2.0));
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeBox(2.0, 4.0, 6.0, placement);
    const MyVoxel::ShapeQuery query(shape);
    const MyVoxel::VoxelFeatureSet local = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape.topology());
    const MyVoxel::VoxelFeatureSet queried = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);

    check(query.supportsSignedDistance() && MyVoxel::Modeling::ShapeFeatureExtractor::supports(query), "Translated Box query supports analytic features");
    check(transformedBoxVerticesCorrect(local, queried, placement), "Translated Box feature vertices map into query coordinates");
    check(queried.bounds().isEqualTo(local.bounds().transformed(placement), Tolerance), "Translated Box feature bounds follow placement");
}

void testRelativeQuerySpace()
{
    const MyMath::Matrix4 objectPlacement = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 4.0, -2.0));
    const MyMath::Matrix4 queryToWorld = MyMath::Matrix4::fromTranslation(MyMath::Vector3(8.0, 1.0, -5.0));
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeBox(2.0, 2.0, 2.0, objectPlacement);
    const MyVoxel::ShapeQuery query(shape, queryToWorld);
    const MyVoxel::VoxelFeatureSet local = MyVoxel::Modeling::ShapeFeatureExtractor::extract(shape.topology());
    const MyVoxel::VoxelFeatureSet queried = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);

    check(query.supportsSignedDistance(), "Relative translated query space supports signed distance");
    check(transformedBoxVerticesCorrect(local, queried, query.localToQuery()), "Feature extraction uses ShapeQuery local-to-query transform");
    check(queried.bounds().isEqualTo(query.queryBounds(), Tolerance), "Relative Box feature bounds equal query bounds");
}

void testUniformScaledCylinder()
{
    const double radius = 2.0;
    const double height = 6.0;
    const double scale = 2.5;
    const MyMath::Matrix4 placement = MyMath::Matrix4::fromScale(scale);
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeCylinder(radius, height, placement);
    const MyVoxel::ShapeQuery query(shape);
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);

    check(query.supportsSignedDistance() && MyVoxel::Modeling::ShapeFeatureExtractor::supports(query), "Uniform-scaled Cylinder query supports analytic features");
    check(features.isValid() && features.edgeCount() == 2 && features.vertexCount() == 2, "Uniform-scaled Cylinder keeps two closed circular features");

    bool allCorrect = true;

    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[index];
        const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());
        allCorrect = allCorrect && nearValue(arc.radius(), radius * scale);
        allCorrect = allCorrect && nearValue(edge.length(), 2.0 * Pi * radius * scale);
        allCorrect = allCorrect && edge.startVertex().isSame(edge.endVertex());
        allCorrect = allCorrect && edge.tangentAt(0.0).isEqualTo(edge.tangentAt(1.0), Tolerance);
        allCorrect = allCorrect && nearValue(std::fabs(arc.center().z()), height * scale * 0.5);
    }

    check(allCorrect, "Uniform scaling preserves analytic circular Feature geometry and seam smoothness");
}

void testMirroredCylinder()
{
    const MyMath::Matrix4 placement = MyMath::Matrix4::fromScale(-2.0);
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeCylinder(1.5, 4.0, placement);
    const MyVoxel::ShapeQuery query(shape);
    const MyVoxel::VoxelFeatureSet features = MyVoxel::Modeling::ShapeFeatureExtractor::extract(query);

    check(query.supportsSignedDistance() && features.isValid(), "Mirrored uniform-scale Cylinder features remain valid");

    bool allCorrect = true;

    for (std::size_t index = 0; index < features.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[index];
        const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());
        allCorrect = allCorrect && nearValue(arc.radius(), 3.0);
        allCorrect = allCorrect && nearValue(edge.length(), 6.0 * Pi);
        allCorrect = allCorrect && edge.startVertex().isSame(edge.endVertex());
        allCorrect = allCorrect && edge.pointAt(0.0).isEqualTo(edge.startVertex().point(), Tolerance);
        allCorrect = allCorrect && edge.pointAt(1.0).isEqualTo(edge.endVertex().point(), Tolerance);
    }

    check(allCorrect, "Mirrored similarity transform preserves circular Feature topology and endpoint consistency");
}

void testRejectedNonUniformMetric()
{
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeCylinder(
        2.0, 4.0, MyMath::Matrix4::fromScale(MyMath::Vector3(2.0, 1.0, 1.0)));
    const MyVoxel::ShapeQuery query(shape);

    check(query.isValid(), "Non-uniform ShapeQuery remains valid for ordinary spatial queries");
    check(!query.supportsSignedDistance(), "Non-uniform ShapeQuery rejects exact signed distance");
    check(!MyVoxel::Modeling::ShapeFeatureExtractor::supports(query), "Query-space Feature extraction rejects transforms that would turn circles into ellipses");
}

}

int main()
{
    std::cout << "MyVoxel ShapeFeatureExtractor F3A query-space test" << std::endl << std::endl;
    testIdentityQuery();
    testTranslatedQuery();
    testRelativeQuerySpace();
    testUniformScaledCylinder();
    testMirroredCylinder();
    testRejectedNonUniformMetric();
    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}