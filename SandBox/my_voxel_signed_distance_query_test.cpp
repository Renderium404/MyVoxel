#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace
{

const double DistanceTolerance = 1.0e-10; // 解析几何距离测试使用的绝对误差容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果并累计统计。
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

// 判断两个双精度数是否在测试容差内一致。
bool nearValue(double first, double second, double tolerance = DistanceTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

// 返回指定Topology_Shape的连续几何资源。
const MyVoxel::Geometry_Shape& geometry(const MyVoxel::Topology_Shape& topology)
{
    return topology.geometry();
}

// 创建一个正X半平面的矩形母线，旋转后形成内半径1.5、外半径3.0、高度4.0的空心回转体。
std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > makeRevolvedProfile()
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > curves;
    curves.reserve(4); // 矩形母线固定由四条直线组成。
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, -2.0, 0.0), MyMath::Vector3(3.0, -2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, -2.0, 0.0), MyMath::Vector3(3.0, 2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, 2.0, 0.0), MyMath::Vector3(1.5, 2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, 2.0, 0.0), MyMath::Vector3(1.5, -2.0, 0.0))));
    return curves;
}

// 创建用于回转距离测试的Topology_Shape。
MyVoxel::Topology_Shape makeRevolvedTopology()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry_Revolved> geometryResource = MyVoxel::Foundation::makeRef<MyVoxel::Geometry_Revolved>(makeRevolvedProfile(), 1.0e-12);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape> shapeGeometry = geometryResource;
    return MyVoxel::Topology_Shape(shapeGeometry);
}

/// Geometry_Box

void testBoxSignedDistance()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(4.0, 6.0, 8.0);
    const MyVoxel::Geometry_Shape& shape = geometry(topology);

    check(shape.supportsSignedDistance(), "Box supports signed distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -2.0), "Box center distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(2.0, 1.0, 0.0)), 0.0), "Box face boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), 1.0), "Box outside face distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 4.0, 5.0)), std::sqrt(3.0)), "Box outside corner distance");
    check(shape.signedDistanceLocalPoint(MyMath::Vector3(1.0, 2.0, 3.0)) < 0.0, "Box inside distance is negative");
}

/// Geometry_Sphere

void testSphereSignedDistance()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createSphere(3.0);
    const MyVoxel::Geometry_Shape& shape = geometry(topology);

    check(shape.supportsSignedDistance(), "Sphere supports signed distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -3.0), "Sphere center distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), 0.0), "Sphere boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(5.0, 0.0, 0.0)), 2.0), "Sphere radial outside distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(1.0, 2.0, 2.0)), 0.0), "Sphere non-axis boundary distance");
    check(shape.signedDistanceLocalPoint(MyMath::Vector3(1.0, 1.0, 1.0)) < 0.0, "Sphere inside distance is negative");
}

/// Geometry_Cylinder

void testCylinderSignedDistance()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createCylinder(2.0, 6.0);
    const MyVoxel::Geometry_Shape& shape = geometry(topology);

    check(shape.supportsSignedDistance(), "Cylinder supports signed distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -2.0), "Cylinder center distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(2.0, 0.0, 0.0)), 0.0), "Cylinder side boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 3.0)), 0.0), "Cylinder cap boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 4.0)), 1.0), "Cylinder axial outside distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, 4.0)), std::sqrt(2.0)), "Cylinder outside rim distance");
    check(shape.signedDistanceLocalPoint(MyMath::Vector3(1.0, 0.0, 1.0)) < 0.0, "Cylinder inside distance is negative");
}

/// Geometry_ConeFrustum

void testConeFrustumSignedDistance()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createConeFrustum(3.0, 1.0, 4.0);
    const MyVoxel::Geometry_Shape& shape = geometry(topology);

    check(shape.supportsSignedDistance(), "ConeFrustum supports signed distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, -2.0)), 0.0), "ConeFrustum bottom rim distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(1.0, 0.0, 2.0)), 0.0), "ConeFrustum top rim distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(2.0, 0.0, 0.0)), 0.0), "ConeFrustum side midpoint distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 3.0)), 1.0), "ConeFrustum above top cap distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, -3.0)), 1.0), "ConeFrustum below bottom cap distance");
    check(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) < 0.0, "ConeFrustum center distance is negative");
    check(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)) > 0.0, "ConeFrustum radial outside distance is positive");
}

/// Geometry_Revolved

void testRevolvedSignedDistance()
{
    const MyVoxel::Topology_Shape topology = makeRevolvedTopology();
    const MyVoxel::Geometry_Shape& shape = geometry(topology);

    check(topology.isValid(), "Revolved topology is valid");
    check(shape.supportsSignedDistance(), "Revolved supports signed distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(2.25, 0.0, 0.0)), -0.75), "Revolved material center distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(1.5, 0.0, 0.0)), 0.0), "Revolved inner wall boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), 0.0), "Revolved outer wall boundary distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), 1.5), "Revolved center hole distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(3.5, 0.0, 0.0)), 0.5), "Revolved radial outside distance");
    check(nearValue(shape.signedDistanceLocalPoint(MyMath::Vector3(2.0, 0.0, 3.0)), 1.0), "Revolved axial outside distance");
}

/// ShapeQuery单位空间

void testShapeQueryIdentity()
{
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createSphere(3.0);
    const MyVoxel::ShapeQuery query(topology);

    check(query.isValid(), "Identity ShapeQuery is valid");
    check(query.isIdentityQuery(), "Topology ShapeQuery uses identity query space");
    check(query.supportsSignedDistance(), "Identity ShapeQuery supports signed distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -3.0), "Identity ShapeQuery inside distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(4.0, 0.0, 0.0)), 1.0), "Identity ShapeQuery outside distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(3.0, 0.0, 0.0)), 0.0), "Identity ShapeQuery boundary distance");
}

/// ShapeQuery平移

void testShapeQueryTranslation()
{
    const MyMath::Vector3 translation(10.0, -3.0, 2.0);
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromTranslation(translation));
    const MyVoxel::ShapeQuery query(shape);

    check(query.isValid() && query.supportsSignedDistance(), "Translated ShapeQuery supports signed distance");
    check(nearValue(query.signedDistanceToPoint(translation), -2.0), "Translated ShapeQuery center distance");
    check(nearValue(query.signedDistanceToPoint(translation + MyMath::Vector3(2.0, 0.0, 0.0)), 0.0), "Translated ShapeQuery boundary distance");
    check(nearValue(query.signedDistanceToPoint(translation + MyMath::Vector3(3.5, 0.0, 0.0)), 1.5), "Translated ShapeQuery outside distance");
}

/// ShapeQuery统一缩放

void testShapeQueryUniformScale()
{
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromScale(2.0));
    const MyVoxel::ShapeQuery query(shape);

    check(query.isValid(), "Uniform-scale ShapeQuery is valid");
    check(query.supportsSignedDistance(), "Uniform-scale ShapeQuery supports signed distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -4.0), "Uniform-scale ShapeQuery rescales inside distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(4.0, 0.0, 0.0)), 0.0), "Uniform-scale ShapeQuery rescales boundary");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(5.0, 0.0, 0.0)), 1.0), "Uniform-scale ShapeQuery rescales outside distance");
}

/// ShapeQuery镜像和统一缩放

void testShapeQueryMirroredScale()
{
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromScale(-2.0));
    const MyVoxel::ShapeQuery query(shape);

    check(query.isValid(), "Mirrored uniform-scale ShapeQuery is valid");
    check(query.supportsSignedDistance(), "Mirrored uniform-scale ShapeQuery supports signed distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(5.0, 0.0, 0.0)), 1.0), "Mirrored uniform-scale ShapeQuery preserves Euclidean distance");
}

/// ShapeQuery拒绝非均匀度量

void testShapeQueryNonUniformScale()
{
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromScale(MyMath::Vector3(2.0, 1.0, 1.0)));
    const MyVoxel::ShapeQuery query(shape);

    check(query.isValid(), "Non-uniform-scale ShapeQuery remains valid for ordinary spatial queries");
    check(!query.supportsSignedDistance(), "Non-uniform-scale ShapeQuery rejects exact signed distance");
}

/// ShapeQuery查询坐标系换算

void testShapeQueryRelativeQuerySpace()
{
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromScale(4.0));
    const MyMath::Matrix4 queryToWorld = MyMath::Matrix4::fromScale(2.0);
    const MyVoxel::ShapeQuery query(shape, queryToWorld);

    check(query.isValid() && query.supportsSignedDistance(), "Relative uniform-scale query space supports signed distance");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(0.0, 0.0, 0.0)), -4.0), "Relative query-space inside distance uses query units");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(4.0, 0.0, 0.0)), 0.0), "Relative query-space boundary uses query units");
    check(nearValue(query.signedDistanceToPoint(MyMath::Vector3(5.0, 0.0, 0.0)), 1.0), "Relative query-space outside distance uses query units");
}

}

int main()
{
    std::cout << "MyVoxel signed-distance geometry / ShapeQuery regression test" << std::endl << std::endl;

    testBoxSignedDistance();
    testSphereSignedDistance();
    testCylinderSignedDistance();
    testConeFrustumSignedDistance();
    testRevolvedSignedDistance();
    testShapeQueryIdentity();
    testShapeQueryTranslation();
    testShapeQueryUniformScale();
    testShapeQueryMirroredScale();
    testShapeQueryNonUniformScale();
    testShapeQueryRelativeQuerySpace();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}