#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Geometry/Curve/CurveKind.h"
#include "MyVoxel/Geometry/Shape/ShapeKind.h"
#include "MyVoxel/Instance/Curve.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Instance/Wire.h"
#include "MyVoxel/Modeling/Curve/CurveModeling.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/RevolvedModeling.h"
#include "MyVoxel/Modeling/Wire/WireModeling.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"

namespace
{

const double TestTolerance = 1.0e-10; // 综合测试中普通几何数值比较使用的统一容差。
const double ProfileTolerance = 1.0e-10; // 回转体二维母线规范化和区域判断使用的测试容差。

class TestRunner
{
public:
    TestRunner()
        : m_passed(0)
        , m_failed(0)
    {
    }

    void check(bool condition, const char* name)
    {
        if (condition)
        {
            ++m_passed;
            std::cout << "[PASS] " << name << std::endl;
        }
        else
        {
            ++m_failed;
            std::cout << "[FAIL] " << name << std::endl;
        }
    }

    int passed() const
    {
        return m_passed;
    }

    int failed() const
    {
        return m_failed;
    }

private:
    int m_passed;
    int m_failed;
};

// 判断两个双精度数是否在测试容差内相等。
bool nearlyEqual(double first, double second, double tolerance = TestTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

// 判断两个三维点是否在测试容差内相等。
bool pointEqual(const MyMath::Vector3& first, const MyMath::Vector3& second, double tolerance = TestTolerance)
{
    return first.isEqualTo(second, tolerance);
}

/// Curve与Topology_Edge

void testCurveModeling(TestRunner& runner)
{
    const MyMath::Vector3 startPoint(1.0, 2.0, 0.0);
    const MyMath::Vector3 endPoint(5.0, 2.0, 0.0);
    const MyVoxel::Topology_Edge edge = MyVoxel::Modeling::createLine(startPoint, endPoint);

    runner.check(edge.isValid(), "Curve.createLine.valid");
    runner.check(edge.geometry().kind() == MyVoxel::CurveKind::Line, "Curve.createLine.kind");
    runner.check(pointEqual(edge.startVertex().point(), startPoint), "Curve.createLine.startVertex");
    runner.check(pointEqual(edge.endVertex().point(), endPoint), "Curve.createLine.endVertex");
    runner.check(pointEqual(edge.pointAt(0.5), MyMath::Vector3(3.0, 2.0, 0.0)), "Curve.createLine.midPoint");
    runner.check(nearlyEqual(edge.length(), 4.0), "Curve.createLine.length");

    const MyVoxel::Topology_Edge reversed = edge.reversed();
    runner.check(reversed.isValid(), "Curve.reversed.valid");
    runner.check(edge.isSame(reversed), "Curve.reversed.sameTopology");
    runner.check(pointEqual(reversed.startVertex().point(), endPoint), "Curve.reversed.startVertex");
    runner.check(pointEqual(reversed.endVertex().point(), startPoint), "Curve.reversed.endVertex");
    runner.check(pointEqual(reversed.pointAt(0.25), edge.pointAt(0.75)), "Curve.reversed.parameter");

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));
    const MyVoxel::Curve curve(edge, transform);

    runner.check(curve.isValid(), "Curve.instance.valid");
    runner.check(curve.topology().isSame(edge), "Curve.instance.topology");
    runner.check(pointEqual(curve.startPoint(), startPoint), "Curve.instance.localStart");
    runner.check(pointEqual(curve.worldStartPoint(), MyMath::Vector3(11.0, 22.0, 30.0)), "Curve.instance.worldStart");
    runner.check(pointEqual(curve.worldEndPoint(), MyMath::Vector3(15.0, 22.0, 30.0)), "Curve.instance.worldEnd");
    runner.check(pointEqual(curve.worldPointAt(0.5), MyMath::Vector3(13.0, 22.0, 30.0)), "Curve.instance.worldMidPoint");
}

/// 开放Topology_Wire

void testOpenWire(TestRunner& runner)
{
    std::vector<MyMath::Vector3> points;
    points.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(4.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(4.0, 3.0, 0.0));

    const MyVoxel::Topology_Wire wire = MyVoxel::Modeling::createPolyline(points);

    runner.check(wire.isValid(), "Wire.polyline.valid");
    runner.check(!wire.isClosed(), "Wire.polyline.open");
    runner.check(wire.edgeCount() == 2U, "Wire.polyline.edgeCount");

    const MyVoxel::Topology_Edge first = wire.edge(0);
    const MyVoxel::Topology_Edge second = wire.edge(1);

    runner.check(first.endVertex().isSame(second.startVertex()), "Wire.polyline.sharedVertex");
    runner.check(pointEqual(first.startVertex().point(), points[0]), "Wire.polyline.firstPoint");
    runner.check(pointEqual(second.endVertex().point(), points[2]), "Wire.polyline.lastPoint");

    const MyVoxel::Topology_Wire reversed = wire.reversed();

    runner.check(reversed.isValid(), "Wire.polyline.reversedValid");
    runner.check(wire.isSame(reversed), "Wire.polyline.reversedSameTopology");
    runner.check(!reversed.isClosed(), "Wire.polyline.reversedOpen");
    runner.check(pointEqual(reversed.edge(0).startVertex().point(), points[2]), "Wire.polyline.reversedStart");
    runner.check(pointEqual(reversed.edge(1).endVertex().point(), points[0]), "Wire.polyline.reversedEnd");
    runner.check(reversed.edge(0).endVertex().isSame(reversed.edge(1).startVertex()), "Wire.polyline.reversedConnectivity");
}

/// 闭合Topology_Wire

void testClosedWire(TestRunner& runner)
{
    std::vector<MyMath::Vector3> points;
    points.push_back(MyMath::Vector3(1.0, 1.0, 0.0));
    points.push_back(MyMath::Vector3(5.0, 1.0, 0.0));
    points.push_back(MyMath::Vector3(5.0, 4.0, 0.0));
    points.push_back(MyMath::Vector3(1.0, 4.0, 0.0));

    const MyVoxel::Topology_Wire polygon = MyVoxel::Modeling::createPolygon(points);

    runner.check(polygon.isValid(), "Wire.polygon.valid");
    runner.check(polygon.isClosed(), "Wire.polygon.closed");
    runner.check(polygon.edgeCount() == 4U, "Wire.polygon.edgeCount");
    runner.check(polygon.edge(3).endVertex().isSame(polygon.edge(0).startVertex()), "Wire.polygon.closure");

    const MyVoxel::Topology_Wire rectangle = MyVoxel::Modeling::createRectangle(MyMath::Vector3(3.0, 2.0, 0.0), 4.0, 2.0);

    runner.check(rectangle.isValid(), "Wire.rectangle.valid");
    runner.check(rectangle.isClosed(), "Wire.rectangle.closed");
    runner.check(rectangle.edgeCount() == 4U, "Wire.rectangle.edgeCount");

    const MyVoxel::Topology_Wire circle = MyVoxel::Modeling::createCircle(MyMath::Vector3(2.0, 3.0, 0.0), 5.0);

    runner.check(circle.isValid(), "Wire.circle.valid");
    runner.check(circle.isClosed(), "Wire.circle.closed");
    runner.check(circle.edgeCount() == 1U, "Wire.circle.singleEdge");
    runner.check(circle.edge(0).startVertex().isSame(circle.edge(0).endVertex()), "Wire.circle.sharedClosureVertex");
    runner.check(circle.edge(0).geometry().kind() == MyVoxel::CurveKind::Arc, "Wire.circle.arcGeometry");
}

/// Wire实例

void testWireInstance(TestRunner& runner)
{
    std::vector<MyMath::Vector3> points;
    points.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(4.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(4.0, 3.0, 0.0));

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, -5.0, 2.0));
    const MyVoxel::Wire wire = MyVoxel::Modeling::makePolyline(points, transform);

    runner.check(wire.isValid(), "Wire.instance.valid");
    runner.check(!wire.isClosed(), "Wire.instance.open");
    runner.check(wire.edgeCount() == 2U, "Wire.instance.edgeCount");
    runner.check(pointEqual(wire.localStartPoint(), points[0]), "Wire.instance.localStart");
    runner.check(pointEqual(wire.localEndPoint(), points[2]), "Wire.instance.localEnd");
    runner.check(pointEqual(wire.worldStartPoint(), MyMath::Vector3(10.0, -5.0, 2.0)), "Wire.instance.worldStart");
    runner.check(pointEqual(wire.worldEndPoint(), MyMath::Vector3(14.0, -2.0, 2.0)), "Wire.instance.worldEnd");

    const MyVoxel::Curve firstCurve = wire.curve(0);

    runner.check(firstCurve.isValid(), "Wire.instance.curveValid");
    runner.check(pointEqual(firstCurve.worldStartPoint(), wire.worldStartPoint()), "Wire.instance.curveWorldStart");
    runner.check(pointEqual(firstCurve.worldEndPoint(), MyMath::Vector3(14.0, -5.0, 2.0)), "Wire.instance.curveWorldEnd");

    const MyVoxel::Wire reversed = wire.reversed();

    runner.check(reversed.isValid(), "Wire.instance.reversedValid");
    runner.check(reversed.topology().isSame(wire.topology()), "Wire.instance.reversedSameTopology");
    runner.check(pointEqual(reversed.worldStartPoint(), wire.worldEndPoint()), "Wire.instance.reversedWorldStart");
    runner.check(pointEqual(reversed.worldEndPoint(), wire.worldStartPoint()), "Wire.instance.reversedWorldEnd");
}

/// 基础Shape建模

void testPrimitiveShapeModeling(TestRunner& runner)
{
    const MyVoxel::Topology_Shape boxTopology = MyVoxel::Modeling::createBox(10.0, 8.0, 6.0);

    runner.check(boxTopology.isValid(), "Shape.boxTopology.valid");
    runner.check(boxTopology.geometry().kind() == MyVoxel::ShapeKind::Box, "Shape.boxTopology.kind");

    const MyVoxel::Shape localBox(boxTopology);

    runner.check(localBox.isValid(), "Shape.box.valid");
    runner.check(localBox.kind() == MyVoxel::ShapeKind::Box, "Shape.box.kind");
    runner.check(localBox.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), "Shape.box.containsCenter");
    runner.check(!localBox.containsLocalPoint(MyMath::Vector3(6.0, 0.0, 0.0)), "Shape.box.rejectOutside");

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(20.0, 30.0, 40.0));
    const MyVoxel::Shape worldBox(boxTopology, transform);

    runner.check(worldBox.isValid(), "Shape.boxWorld.valid");
    runner.check(worldBox.sharesGeometryWith(localBox), "Shape.boxWorld.sharedGeometry");
    runner.check(worldBox.containsWorldPoint(MyMath::Vector3(20.0, 30.0, 40.0)), "Shape.boxWorld.containsCenter");
    runner.check(!worldBox.containsWorldPoint(MyMath::Vector3(30.0, 30.0, 40.0)), "Shape.boxWorld.rejectOutside");

    const MyVoxel::Shape sphere = MyVoxel::Modeling::makeSphere(5.0);
    const MyVoxel::Shape cylinder = MyVoxel::Modeling::makeCylinder(4.0, 10.0);
    const MyVoxel::Shape cone = MyVoxel::Modeling::makeCone(5.0, 10.0);
    const MyVoxel::Shape frustum = MyVoxel::Modeling::makeConeFrustum(5.0, 3.0, 10.0);

    runner.check(sphere.isValid() && sphere.kind() == MyVoxel::ShapeKind::Sphere, "Shape.sphere.valid");
    runner.check(cylinder.isValid() && cylinder.kind() == MyVoxel::ShapeKind::Cylinder, "Shape.cylinder.valid");
    runner.check(cone.isValid() && cone.kind() == MyVoxel::ShapeKind::ConeFrustum, "Shape.cone.valid");
    runner.check(frustum.isValid() && frustum.kind() == MyVoxel::ShapeKind::ConeFrustum, "Shape.frustum.valid");
}

/// 回转体建模

void testRevolvedModeling(TestRunner& runner)
{
    std::vector<MyMath::Vector3> profilePoints;
    profilePoints.push_back(MyMath::Vector3(2.0, -3.0, 0.0));
    profilePoints.push_back(MyMath::Vector3(5.0, -3.0, 0.0));
    profilePoints.push_back(MyMath::Vector3(5.0, 3.0, 0.0));
    profilePoints.push_back(MyMath::Vector3(2.0, 3.0, 0.0));

    const MyVoxel::Topology_Wire profile = MyVoxel::Modeling::createPolygon(profilePoints);

    runner.check(profile.isValid(), "Revolved.profile.valid");
    runner.check(profile.isClosed(), "Revolved.profile.closed");
    runner.check(profile.edgeCount() == 4U, "Revolved.profile.edgeCount");

    const MyVoxel::Topology_Shape revolvedTopology = MyVoxel::Modeling::createRevolved(profile, ProfileTolerance);

    runner.check(revolvedTopology.isValid(), "Revolved.topology.valid");
    runner.check(revolvedTopology.geometry().kind() == MyVoxel::ShapeKind::Revolved, "Revolved.topology.kind");

    const MyVoxel::Shape revolved(revolvedTopology);

    runner.check(revolved.isValid(), "Revolved.shape.valid");
    runner.check(revolved.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "Revolved.containsRadialMaterial");
    runner.check(revolved.containsLocalPoint(MyMath::Vector3(0.0, 3.0, 0.0)), "Revolved.containsRotatedRadialMaterial");
    runner.check(!revolved.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), "Revolved.rejectInnerHole");
    runner.check(!revolved.containsLocalPoint(MyMath::Vector3(6.0, 0.0, 0.0)), "Revolved.rejectOuterRadius");
    runner.check(!revolved.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 4.0)), "Revolved.rejectOutsideHeight");

    const MyVoxel::Topology_Wire reversedProfile = profile.reversed();
    const MyVoxel::Topology_Shape reversedTopology = MyVoxel::Modeling::createRevolved(reversedProfile, ProfileTolerance);

    runner.check(reversedTopology.isValid(), "Revolved.reversedProfile.valid");

    if (reversedTopology.isValid())
    {
        const MyVoxel::Shape reversedShape(reversedTopology);
        runner.check(reversedShape.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "Revolved.reversedProfile.sameInside");
        runner.check(!reversedShape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), "Revolved.reversedProfile.sameHole");
    }

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));
    const MyVoxel::Shape worldRevolved = MyVoxel::Modeling::makeRevolved(profile, ProfileTolerance, transform);

    runner.check(worldRevolved.isValid(), "Revolved.world.valid");
    runner.check(worldRevolved.containsWorldPoint(MyMath::Vector3(13.0, 20.0, 30.0)), "Revolved.world.containsMaterial");
    runner.check(!worldRevolved.containsWorldPoint(MyMath::Vector3(10.0, 20.0, 30.0)), "Revolved.world.rejectHole");

    const MyVoxel::Wire profileInstance(profile, transform);
    const MyVoxel::Shape fromWireInstance = MyVoxel::Modeling::makeRevolved(profileInstance, ProfileTolerance);

    runner.check(fromWireInstance.isValid(), "Revolved.fromWire.valid");
    runner.check(fromWireInstance.containsWorldPoint(MyMath::Vector3(13.0, 20.0, 30.0)), "Revolved.fromWire.transformPreserved");
}

/// 包围盒一致性

void testBounds(TestRunner& runner)
{
    const MyVoxel::Wire rectangle = MyVoxel::Modeling::makeRectangle(MyMath::Vector3(3.0, 4.0, 0.0), 6.0, 8.0);

    runner.check(rectangle.isValid(), "Bounds.wire.valid");

    if (rectangle.isValid())
    {
        const MyVoxel::Bounds3& bounds = rectangle.localBounds();
        runner.check(pointEqual(bounds.minimum(), MyMath::Vector3(0.0, 0.0, 0.0)), "Bounds.wire.minimum");
        runner.check(pointEqual(bounds.maximum(), MyMath::Vector3(6.0, 8.0, 0.0)), "Bounds.wire.maximum");
    }

    const MyVoxel::Shape box = MyVoxel::Modeling::makeBox(10.0, 8.0, 6.0);

    runner.check(box.isValid(), "Bounds.shape.valid");

    if (box.isValid())
    {
        runner.check(pointEqual(box.localBounds().minimum(), MyMath::Vector3(-5.0, -4.0, -3.0)), "Bounds.shape.minimum");
        runner.check(pointEqual(box.localBounds().maximum(), MyMath::Vector3(5.0, 4.0, 3.0)), "Bounds.shape.maximum");
    }
}

}

int main()
{
    std::cout << "============================================================" << std::endl;
    std::cout << "MyVoxel modeling core test" << std::endl;
    std::cout << "============================================================" << std::endl;

    TestRunner runner;

    testCurveModeling(runner);
    testOpenWire(runner);
    testClosedWire(runner);
    testWireInstance(runner);
    testPrimitiveShapeModeling(runner);
    testRevolvedModeling(runner);
    testBounds(runner);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << runner.passed() << std::endl;
    std::cout << "Failed: " << runner.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return runner.failed() == 0 ? 0 : 1;
}