#include <cmath>
#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Geometry/Curve/CurveKind.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Modeling/Shape/MeshModeling.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/RevolvedModeling.h"
#include "MyVoxel/Modeling/Wire/WireModeling.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧测试使用的圆周率。
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

bool nearValue(double first, double second, double tolerance)
{
    return std::fabs(first - second) <= tolerance;
}

}

int main()
{
    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, -3.0, 2.0));

    const MyVoxel::Topology_Curve lineTopology =
        MyVoxel::Modeling::createLine(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(3.0, 4.0, 0.0));
    check(lineTopology.isValid() && lineTopology.kind() == MyVoxel::CurveKind::Line, "createLine returns Topology_Curve");

    const MyVoxel::Curve line =
        MyVoxel::Modeling::makeLine(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(3.0, 4.0, 0.0), transform);
    check(line.isValid() && line.kind() == MyVoxel::CurveKind::Line, "makeLine returns Curve");
    check(line.localToWorld().isEqualTo(transform, 0.0), "makeLine preserves transform");
    check(line.worldStartPoint().isEqualTo(MyMath::Vector3(10.0, -3.0, 2.0), 0.0), "Curve world placement");

    const MyVoxel::Topology_Curve arcTopology =
        MyVoxel::Modeling::createArc(MyMath::Vector3(0.0, 0.0, 0.0), 2.0, 0.0, Pi * 0.5);
    check(arcTopology.isValid() && nearValue(arcTopology.length(), Pi, 1.0e-12), "createArc returns Topology_Curve");

    const MyVoxel::Curve arc =
        MyVoxel::Modeling::makeArc(MyMath::Vector3(0.0, 0.0, 0.0), 2.0, 0.0, Pi * 0.5);
    check(arc.isValid() && arc.localToWorld().isIdentity(0.0), "makeArc uses identity transform");

    const MyVoxel::Topology_Wire rectangleTopology = MyVoxel::Modeling::createRectangle(6.0, 4.0);
    check(rectangleTopology.isValid() && rectangleTopology.isClosed() && rectangleTopology.isCounterClockwise(), "createRectangle returns Topology_Wire");
    check(nearValue(rectangleTopology.signedArea(), 24.0, 1.0e-12), "createRectangle area");

    const MyVoxel::Wire rectangle = MyVoxel::Modeling::makeRectangle(6.0, 4.0, transform);
    check(rectangle.isValid() && rectangle.isClosed(), "makeRectangle returns Wire");
    check(rectangle.localToWorld().isEqualTo(transform, 0.0), "makeRectangle preserves transform");
    check(rectangle.worldStartPoint().isEqualTo(transform.transformPoint(rectangle.localStartPoint()), 0.0), "Wire world placement");

    const MyVoxel::Topology_Wire circleTopology = MyVoxel::Modeling::createCircle(3.0);
    check(circleTopology.isValid() && circleTopology.isClosed() && circleTopology.curveCount() == 1, "createCircle returns Topology_Wire");

    std::vector<MyMath::Vector3> polygonPoints;
    polygonPoints.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    polygonPoints.push_back(MyMath::Vector3(4.0, 0.0, 0.0));
    polygonPoints.push_back(MyMath::Vector3(4.0, 3.0, 0.0));
    polygonPoints.push_back(MyMath::Vector3(0.0, 3.0, 0.0));
    check(MyVoxel::Modeling::createPolygon(polygonPoints).isValid(), "createPolygon");
    check(MyVoxel::Modeling::makePolygon(polygonPoints, transform).isValid(), "makePolygon");

    const MyVoxel::Topology_Shape boxTopology = MyVoxel::Modeling::createBox(4.0, 5.0, 6.0);
    check(boxTopology.isValid() && boxTopology.kind() == MyVoxel::ShapeKind::Box, "createBox returns Topology_Shape");

    const MyVoxel::Shape box = MyVoxel::Modeling::makeBox(4.0, 5.0, 6.0, transform);
    check(box.isValid() && box.kind() == MyVoxel::ShapeKind::Box, "makeBox returns Shape");
    check(box.localToWorld().isEqualTo(transform, 0.0), "makeBox preserves transform");

    check(MyVoxel::Modeling::createSphere(3.0).isValid(), "createSphere");
    check(MyVoxel::Modeling::makeSphere(3.0).isValid(), "makeSphere");
    check(MyVoxel::Modeling::createCylinder(2.0, 5.0).isValid(), "createCylinder");
    check(MyVoxel::Modeling::makeCylinder(2.0, 5.0, transform).isValid(), "makeCylinder");
    check(MyVoxel::Modeling::createCone(3.0, 5.0).isValid(), "createCone");
    check(MyVoxel::Modeling::makeCone(3.0, 5.0).isValid(), "makeCone");
    check(MyVoxel::Modeling::createConeFrustum(3.0, 1.5, 5.0).isValid(), "createConeFrustum");
    check(MyVoxel::Modeling::makeConeFrustum(3.0, 1.5, 5.0, transform).isValid(), "makeConeFrustum");

    const MyVoxel::Topology_Wire profile =
        MyVoxel::Modeling::createRectangle(MyMath::Vector3(2.5, 0.0, 0.0), 2.0, 4.0);
    const MyVoxel::Topology_Shape revolvedTopology = MyVoxel::Modeling::createRevolved(profile);
    const MyVoxel::Shape revolved = MyVoxel::Modeling::makeRevolved(profile, transform);
    check(revolvedTopology.isValid() && revolvedTopology.kind() == MyVoxel::ShapeKind::Revolved, "createRevolved returns Topology_Shape");
    check(revolved.isValid() && revolved.kind() == MyVoxel::ShapeKind::Revolved, "makeRevolved returns Shape");
    check(revolved.localToWorld().isEqualTo(transform, 0.0), "makeRevolved preserves transform");

    const MyVoxel::Mesh sourceMesh =
        MyVoxel::ShapeMesher::build(boxTopology, MyVoxel::Display_Color::gray());
    const MyVoxel::Topology_Shape meshTopology = MyVoxel::Modeling::createMesh(sourceMesh);
    check(meshTopology.isValid() && meshTopology.kind() == MyVoxel::ShapeKind::Mesh, "createMesh returns Topology_Shape");

    MyVoxel::Mesh movableMesh = sourceMesh;
    const MyVoxel::Shape meshShape = MyVoxel::Modeling::makeMesh(std::move(movableMesh), transform);
    check(meshShape.isValid() && meshShape.kind() == MyVoxel::ShapeKind::Mesh, "makeMesh returns Shape");
    check(meshShape.localToWorld().isEqualTo(transform, 0.0), "makeMesh preserves transform");

    std::cout << "Modeling create/make Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
