#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Builder/Display_CurveBuilder.h"
#include "MyVoxel/Display/Builder/Display_CurveBuildOptions.h"
#include "MyVoxel/Display/Builder/Display_ShapeBuilder.h"
#include "MyVoxel/Display/Builder/Display_WireBuilder.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Object/Display_Object.h"
#include "MyVoxel/Display/Object/Display_ObjectManager.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"

#include "MyVoxel/Foundation/RefPtr.h"

#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"

#include "MyVoxel/Instance/Curve.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Instance/Wire.h"

#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/RevolvedModeling.h"

#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧离散测试统一使用弧度制。
const double GeometryTolerance = 1.0e-9; // Topology_Edge端点与曲线连接检查使用的统一测试容差。
const double DisplayTolerance = 1.0e-5; // Display顶点坐标使用float保存，因此显示数据比较使用1e-5容差。

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

    std::size_t passed() const
    {
        return m_passed;
    }

    std::size_t failed() const
    {
        return m_failed;
    }

private:
    std::size_t m_passed;
    std::size_t m_failed;
};

// 判断两个标量是否在显示数据比较容差内相等。
bool nearValue(double first, double second)
{
    return std::fabs(first - second) <= DisplayTolerance;
}

// 判断两个三维点是否在显示数据比较容差内相等。
bool nearPoint(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.isEqualTo(second, DisplayTolerance);
}

// 返回Display_LineVertex对应的三维位置。
MyMath::Vector3 lineVertexPoint(const MyVoxel::Display_LineVertex& vertex)
{
    return MyMath::Vector3(vertex.x, vertex.y, vertex.z);
}

// 返回测试使用的红色。
MyVoxel::Display_Color redColor()
{
    return MyVoxel::Display_Color(1.0, 0.0, 0.0, 1.0);
}

// 返回测试使用的绿色。
MyVoxel::Display_Color greenColor()
{
    return MyVoxel::Display_Color(0.0, 1.0, 0.0, 1.0);
}

// 返回测试使用的蓝色。
MyVoxel::Display_Color blueColor()
{
    return MyVoxel::Display_Color(0.0, 0.0, 1.0, 1.0);
}

// 返回测试使用的白色。
MyVoxel::Display_Color whiteColor()
{
    return MyVoxel::Display_Color(1.0, 1.0, 1.0, 1.0);
}

// 返回测试使用的灰色。
MyVoxel::Display_Color grayColor()
{
    return MyVoxel::Display_Color(0.5, 0.5, 0.5, 1.0);
}

// 使用共享Topology_Vertex创建完整直线Edge。
MyVoxel::Topology_Edge makeLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(
        new MyVoxel::Geometry_Line(startVertex.point(), endVertex.point()));
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, GeometryTolerance);
}

// 使用指定局部XY平面端点创建完整直线Edge。
MyVoxel::Topology_Edge makeLineEdge(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
{
    const MyVoxel::Topology_Vertex startVertex(startPoint);
    const MyVoxel::Topology_Vertex endVertex(endPoint);
    return makeLineEdge(startVertex, endVertex);
}

// 创建世界XY平面中的圆弧Edge。
MyVoxel::Topology_Edge makeArcEdge(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(
        new MyVoxel::Geometry_Arc(center, radius, startAngle, sweepAngle));
    const MyVoxel::Topology_Vertex startVertex(geometry->startPoint());
    const MyVoxel::Topology_Vertex endVertex(geometry->endPoint());
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, GeometryTolerance);
}

// 创建包含两条共享中间Topology_Vertex的开放Wire。
MyVoxel::Topology_Wire makeOpenWire()
{
    const MyVoxel::Topology_Vertex vertex0(MyMath::Vector3(0.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex vertex1(MyMath::Vector3(3.0, 0.0, 0.0));
    const MyVoxel::Topology_Vertex vertex2(MyMath::Vector3(3.0, 2.0, 0.0));
    std::vector<MyVoxel::Topology_Edge> edges;
    edges.push_back(makeLineEdge(vertex0, vertex1));
    edges.push_back(makeLineEdge(vertex1, vertex2));
    return MyVoxel::Topology_Wire(edges);
}

// 创建位于局部Y轴正侧、绕Z轴旋转后形成空心圆柱体的矩形闭合母线。
MyVoxel::Topology_Wire makeClosedRevolvedProfile()
{
    const MyVoxel::Topology_Vertex vertex0(MyMath::Vector3(1.0, -1.0, 0.0));
    const MyVoxel::Topology_Vertex vertex1(MyMath::Vector3(2.0, -1.0, 0.0));
    const MyVoxel::Topology_Vertex vertex2(MyMath::Vector3(2.0, 1.0, 0.0));
    const MyVoxel::Topology_Vertex vertex3(MyMath::Vector3(1.0, 1.0, 0.0));
    std::vector<MyVoxel::Topology_Edge> edges;
    edges.push_back(makeLineEdge(vertex0, vertex1));
    edges.push_back(makeLineEdge(vertex1, vertex2));
    edges.push_back(makeLineEdge(vertex2, vertex3));
    edges.push_back(makeLineEdge(vertex3, vertex0));
    return MyVoxel::Topology_Wire(edges);
}

/// Curve离散参数

void testCurveBuildOptions(TestRunner& runner)
{
    MyVoxel::Display_CurveBuildOptions options;

    runner.check(options.isValid(), "CurveOptions.defaultValid");
    runner.check(options.fullCircleSegmentCount == 64U, "CurveOptions.defaultSegmentCount");

    options.fullCircleSegmentCount = 3U;
    runner.check(options.isValid(), "CurveOptions.minimumValid");

    options.fullCircleSegmentCount = 2U;
    runner.check(!options.isValid(), "CurveOptions.rejectTooFewSegments");
}

/// 直线Edge到Line Resource

void testLineResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Edge edge =
        makeLineEdge(MyMath::Vector3(1.0, 2.0, 0.0), MyMath::Vector3(5.0, 6.0, 0.0));

    runner.check(edge.isValid(), "LineResource.edgeValid");
    runner.check(edge.startVertex().point().isEqualTo(MyMath::Vector3(1.0, 2.0, 0.0), 0.0), "LineResource.startVertex");
    runner.check(edge.endVertex().point().isEqualTo(MyMath::Vector3(5.0, 6.0, 0.0), 0.0), "LineResource.endVertex");
    runner.check(MyVoxel::Display_CurveBuilder::supports(edge), "LineResource.supported");

    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_CurveBuilder::createResource(edge, resourceManager, redColor());

    runner.check(resourceId != 0, "LineResource.created");
    runner.check(resourceManager.contains(resourceId), "LineResource.registered");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource =
        resourceManager.lineResource(resourceId);

    runner.check(resource && resource->isValid(), "LineResource.valid");
    runner.check(resource && resource->segmentCount() == 1U, "LineResource.segmentCount");
    runner.check(resource && resource->vertexCount() == 2U, "LineResource.vertexCount");

    if (resource && resource->vertexCount() == 2U)
    {
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[0]), edge.pointAt(0.0)), "LineResource.startPoint");
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[1]), edge.pointAt(1.0)), "LineResource.endPoint");
    }

    if (resource)
    {
        runner.check(nearPoint(resource->localBounds().minimum(), MyMath::Vector3(1.0, 2.0, 0.0)), "LineResource.boundsMinimum");
        runner.check(nearPoint(resource->localBounds().maximum(), MyMath::Vector3(5.0, 6.0, 0.0)), "LineResource.boundsMaximum");
    }
}

/// Edge反向必须反转显示点顺序

void testReversedLineResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Edge forward =
        makeLineEdge(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(10.0, 0.0, 0.0));
    const MyVoxel::Topology_Edge reversed = forward.reversed();

    runner.check(forward.isValid() && reversed.isValid(), "ReversedLine.edgesValid");
    runner.check(reversed.startVertex().isSame(forward.endVertex()), "ReversedLine.startVertexIdentity");
    runner.check(reversed.endVertex().isSame(forward.startVertex()), "ReversedLine.endVertexIdentity");
    runner.check(nearPoint(reversed.pointAt(0.0), MyMath::Vector3(10.0, 0.0, 0.0)), "ReversedLine.topologyStart");
    runner.check(nearPoint(reversed.pointAt(1.0), MyMath::Vector3(0.0, 0.0, 0.0)), "ReversedLine.topologyEnd");

    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_CurveBuilder::createResource(reversed, resourceManager, greenColor());
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource =
        resourceManager.lineResource(resourceId);

    runner.check(resource && resource->isValid(), "ReversedLine.resourceValid");

    if (resource && resource->vertexCount() == 2U)
    {
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[0]), MyMath::Vector3(10.0, 0.0, 0.0)), "ReversedLine.displayStart");
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[1]), MyMath::Vector3(0.0, 0.0, 0.0)), "ReversedLine.displayEnd");
    }
}

/// 圆弧Edge离散

void testArcResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_CurveBuildOptions options;
    options.fullCircleSegmentCount = 8U; // 完整圆使用8段时，90度圆弧应使用2段。

    const MyVoxel::Topology_Edge edge =
        makeArcEdge(MyMath::Vector3(0.0, 0.0, 0.0), 2.0, 0.0, Pi * 0.5);

    runner.check(edge.isValid(), "ArcResource.edgeValid");
    runner.check(edge.geometry().kind() == MyVoxel::CurveKind::Arc, "ArcResource.geometryKind");
    runner.check(MyVoxel::Display_CurveBuilder::supports(edge), "ArcResource.supported");

    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_CurveBuilder::createResource(edge, resourceManager, blueColor(), options);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource =
        resourceManager.lineResource(resourceId);

    runner.check(resourceId != 0, "ArcResource.created");
    runner.check(resource && resource->isValid(), "ArcResource.valid");
    runner.check(resource && resource->segmentCount() == 2U, "ArcResource.segmentCount");
    runner.check(resource && resource->vertexCount() == 4U, "ArcResource.vertexCount");

    if (resource && resource->vertexCount() == 4U)
    {
        const double diagonal = std::sqrt(2.0);
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[0]), MyMath::Vector3(2.0, 0.0, 0.0)), "ArcResource.start");
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[1]), MyMath::Vector3(diagonal, diagonal, 0.0)), "ArcResource.segment0End");
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[2]), MyMath::Vector3(diagonal, diagonal, 0.0)), "ArcResource.segment1Start");
        runner.check(nearPoint(lineVertexPoint(resource->vertices()[3]), MyMath::Vector3(0.0, 2.0, 0.0)), "ArcResource.end");
    }

    const MyVoxel::Topology_Edge reversed = edge.reversed();
    const MyVoxel::Display_ResourceId reversedId =
        MyVoxel::Display_CurveBuilder::createResource(reversed, resourceManager, blueColor(), options);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> reversedResource =
        resourceManager.lineResource(reversedId);

    runner.check(reversedResource && reversedResource->segmentCount() == 2U, "ArcResource.reversedValid");

    if (reversedResource && reversedResource->vertexCount() == 4U)
    {
        runner.check(nearPoint(lineVertexPoint(reversedResource->vertices()[0]), MyMath::Vector3(0.0, 2.0, 0.0)), "ArcResource.reversedStart");
        runner.check(nearPoint(lineVertexPoint(reversedResource->vertices()[3]), MyMath::Vector3(2.0, 0.0, 0.0)), "ArcResource.reversedEnd");
    }
}

/// Curve实例到Display_Object

void testCurveObject(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Edge edge =
        makeLineEdge(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(4.0, 2.0, 0.0));
    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_CurveBuilder::createResource(edge, resourceManager, redColor());

    const MyMath::Matrix4 transform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));
    const MyVoxel::Curve curve(edge, transform);

    runner.check(curve.isValid(), "CurveObject.curveValid");

    const MyVoxel::Display_ObjectId objectId =
        MyVoxel::Display_CurveBuilder::createObject(curve, resourceId, objectManager, MyVoxel::Display_ObjectUsage::Dynamic, 2.5);

    runner.check(objectId != 0, "CurveObject.created");

    const MyVoxel::Display_Object object = objectManager.object(objectId);

    runner.check(object.isValid(), "CurveObject.valid");
    runner.check(object.resourceKind() == MyVoxel::Display_ResourceKind::Line, "CurveObject.kind");
    runner.check(object.usage() == MyVoxel::Display_ObjectUsage::Dynamic, "CurveObject.usage");
    runner.check(object.partCount() == 1U, "CurveObject.partCount");
    runner.check(object.hasPart(0), "CurveObject.hasPart0");
    runner.check(nearValue(object.lineWidth(), 2.5), "CurveObject.lineWidth");
    runner.check(object.localToWorld().isEqualTo(transform, 0.0), "CurveObject.transform");

    if (object.hasPart(0))
    {
        runner.check(object.part(0).resourceId == resourceId, "CurveObject.resourceId");
    }

    runner.check(nearPoint(object.localBounds().minimum(), MyMath::Vector3(0.0, 0.0, 0.0)), "CurveObject.localMinimum");
    runner.check(nearPoint(object.localBounds().maximum(), MyMath::Vector3(4.0, 2.0, 0.0)), "CurveObject.localMaximum");
    runner.check(nearPoint(object.worldBounds().minimum(), MyMath::Vector3(10.0, 20.0, 30.0)), "CurveObject.worldMinimum");
    runner.check(nearPoint(object.worldBounds().maximum(), MyMath::Vector3(14.0, 22.0, 30.0)), "CurveObject.worldMaximum");
}

/// 一个Curve Resource可以复用于多个Curve实例

void testCurveResourceSharing(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Edge edge =
        makeLineEdge(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(5.0, 0.0, 0.0));
    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_CurveBuilder::createResource(edge, resourceManager, greenColor());

    const MyMath::Matrix4 transformA =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 0.0, 0.0));
    const MyMath::Matrix4 transformB =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(-10.0, 5.0, 0.0));
    const MyVoxel::Curve curveA(edge, transformA);
    const MyVoxel::Curve curveB(edge, transformB);

    const MyVoxel::Display_ObjectId objectIdA =
        MyVoxel::Display_CurveBuilder::createObject(curveA, resourceId, objectManager);
    const MyVoxel::Display_ObjectId objectIdB =
        MyVoxel::Display_CurveBuilder::createObject(curveB, resourceId, objectManager);

    runner.check(objectIdA != 0 && objectIdB != 0, "CurveSharing.objectsCreated");
    runner.check(objectIdA != objectIdB, "CurveSharing.objectIdsDistinct");
    runner.check(resourceManager.resourceCount() == 1U, "CurveSharing.singleResource");
    runner.check(objectManager.objectCount() == 2U, "CurveSharing.twoObjects");

    const MyVoxel::Display_Object objectA = objectManager.object(objectIdA);
    const MyVoxel::Display_Object objectB = objectManager.object(objectIdB);

    runner.check(objectA.part(0).resourceId == resourceId, "CurveSharing.objectAResource");
    runner.check(objectB.part(0).resourceId == resourceId, "CurveSharing.objectBResource");
    runner.check(objectA.part(0).resource.get() == objectB.part(0).resource.get(), "CurveSharing.sameResourcePointer");
    runner.check(objectA.localToWorld().isEqualTo(transformA, 0.0), "CurveSharing.transformA");
    runner.check(objectB.localToWorld().isEqualTo(transformB, 0.0), "CurveSharing.transformB");
}

/// Curve Builder必须拒绝Mesh Resource

void testCurveRejectMeshResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Edge edge =
        makeLineEdge(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(3.0, 0.0, 0.0));
    const MyVoxel::Curve curve(edge);

    const MyVoxel::Topology_Shape box = MyVoxel::Modeling::createBox(2.0, 2.0, 2.0);
    const MyVoxel::Display_ResourceId meshResourceId =
        MyVoxel::Display_ShapeBuilder::createResource(box, resourceManager, grayColor());

    runner.check(meshResourceId != 0, "CurveRejectMesh.meshResourceCreated");
    runner.check(MyVoxel::Display_CurveBuilder::createObject(curve, meshResourceId, objectManager) == 0, "CurveRejectMesh.rejected");
    runner.check(objectManager.objectCount() == 0U, "CurveRejectMesh.noObjectLeak");
}

/// Wire资源创建

void testWireResources(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Wire wire = makeOpenWire();

    runner.check(wire.isValid(), "WireResources.wireValid");
    runner.check(!wire.isClosed(), "WireResources.open");
    runner.check(wire.edgeCount() == 2U, "WireResources.edgeCount");
    runner.check(wire.edge(0).endVertex().isSame(wire.edge(1).startVertex()), "WireResources.sharedMiddleVertex");
    runner.check(MyVoxel::Display_WireBuilder::supports(wire), "WireResources.supported");

    const std::vector<MyVoxel::Display_ResourceId> resourceIds =
        MyVoxel::Display_WireBuilder::createResources(wire, resourceManager, whiteColor());

    runner.check(resourceIds.size() == 2U, "WireResources.resourceCount");
    runner.check(resourceManager.resourceCount() == 2U, "WireResources.managerCount");

    if (resourceIds.size() == 2U)
    {
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource0 =
            resourceManager.lineResource(resourceIds[0]);
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource1 =
            resourceManager.lineResource(resourceIds[1]);

        runner.check(resource0 && resource0->segmentCount() == 1U, "WireResources.resource0Valid");
        runner.check(resource1 && resource1->segmentCount() == 1U, "WireResources.resource1Valid");

        if (resource0 && resource0->vertexCount() == 2U)
        {
            runner.check(nearPoint(lineVertexPoint(resource0->vertices()[0]), MyMath::Vector3(0.0, 0.0, 0.0)), "WireResources.edge0Start");
            runner.check(nearPoint(lineVertexPoint(resource0->vertices()[1]), MyMath::Vector3(3.0, 0.0, 0.0)), "WireResources.edge0End");
        }

        if (resource1 && resource1->vertexCount() == 2U)
        {
            runner.check(nearPoint(lineVertexPoint(resource1->vertices()[0]), MyMath::Vector3(3.0, 0.0, 0.0)), "WireResources.edge1Start");
            runner.check(nearPoint(lineVertexPoint(resource1->vertices()[1]), MyMath::Vector3(3.0, 2.0, 0.0)), "WireResources.edge1End");
        }
    }
}

/// Reversed Wire必须反序并反转每条Edge

void testReversedWireResources(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Wire forward = makeOpenWire();
    const MyVoxel::Topology_Wire reversed = forward.reversed();

    runner.check(reversed.isValid(), "ReversedWire.valid");
    runner.check(reversed.edgeCount() == 2U, "ReversedWire.edgeCount");
    runner.check(nearPoint(reversed.startVertex().point(), MyMath::Vector3(3.0, 2.0, 0.0)), "ReversedWire.startVertex");
    runner.check(nearPoint(reversed.endVertex().point(), MyMath::Vector3(0.0, 0.0, 0.0)), "ReversedWire.endVertex");

    const std::vector<MyVoxel::Display_ResourceId> resourceIds =
        MyVoxel::Display_WireBuilder::createResources(reversed, resourceManager, greenColor());

    runner.check(resourceIds.size() == 2U, "ReversedWire.resourceCount");

    if (resourceIds.size() == 2U)
    {
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource0 =
            resourceManager.lineResource(resourceIds[0]);
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> resource1 =
            resourceManager.lineResource(resourceIds[1]);

        if (resource0 && resource0->vertexCount() == 2U)
        {
            runner.check(nearPoint(lineVertexPoint(resource0->vertices()[0]), MyMath::Vector3(3.0, 2.0, 0.0)), "ReversedWire.edge0Start");
            runner.check(nearPoint(lineVertexPoint(resource0->vertices()[1]), MyMath::Vector3(3.0, 0.0, 0.0)), "ReversedWire.edge0End");
        }

        if (resource1 && resource1->vertexCount() == 2U)
        {
            runner.check(nearPoint(lineVertexPoint(resource1->vertices()[0]), MyMath::Vector3(3.0, 0.0, 0.0)), "ReversedWire.edge1Start");
            runner.check(nearPoint(lineVertexPoint(resource1->vertices()[1]), MyMath::Vector3(0.0, 0.0, 0.0)), "ReversedWire.edge1End");
        }
    }
}

/// Wire实例到多Part Display_Object

void testWireObject(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);
    const MyVoxel::Topology_Wire topology = makeOpenWire();
    const std::vector<MyVoxel::Display_ResourceId> resourceIds =
        MyVoxel::Display_WireBuilder::createResources(topology, resourceManager, whiteColor());

    const MyMath::Matrix4 transform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, 7.0, 11.0));
    const MyVoxel::Wire wire(topology, transform);

    runner.check(wire.isValid(), "WireObject.wireValid");

    const MyVoxel::Display_ObjectId objectId =
        MyVoxel::Display_WireBuilder::createObject(wire, resourceIds, objectManager, MyVoxel::Display_ObjectUsage::Dynamic, 1.5);

    runner.check(objectId != 0, "WireObject.created");

    const MyVoxel::Display_Object object = objectManager.object(objectId);

    runner.check(object.isValid(), "WireObject.valid");
    runner.check(object.resourceKind() == MyVoxel::Display_ResourceKind::Line, "WireObject.kind");
    runner.check(object.usage() == MyVoxel::Display_ObjectUsage::Dynamic, "WireObject.usage");
    runner.check(object.partCount() == 2U, "WireObject.partCount");
    runner.check(object.hasPart(0) && object.hasPart(1), "WireObject.partsExist");
    runner.check(nearValue(object.lineWidth(), 1.5), "WireObject.lineWidth");
    runner.check(object.localToWorld().isEqualTo(transform, 0.0), "WireObject.transform");

    if (resourceIds.size() == 2U && object.hasPart(0) && object.hasPart(1))
    {
        runner.check(object.part(0).resourceId == resourceIds[0], "WireObject.part0Resource");
        runner.check(object.part(1).resourceId == resourceIds[1], "WireObject.part1Resource");
    }

    runner.check(nearPoint(object.localBounds().minimum(), MyMath::Vector3(0.0, 0.0, 0.0)), "WireObject.localMinimum");
    runner.check(nearPoint(object.localBounds().maximum(), MyMath::Vector3(3.0, 2.0, 0.0)), "WireObject.localMaximum");
    runner.check(nearPoint(object.worldBounds().minimum(), MyMath::Vector3(5.0, 7.0, 11.0)), "WireObject.worldMinimum");
    runner.check(nearPoint(object.worldBounds().maximum(), MyMath::Vector3(8.0, 9.0, 11.0)), "WireObject.worldMaximum");
}

/// Wire Builder必须拒绝Mesh Resource Part

void testWireRejectMeshResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);
    const MyVoxel::Topology_Wire topology = makeOpenWire();
    const MyVoxel::Wire wire(topology);

    const MyVoxel::Topology_Shape box = MyVoxel::Modeling::createBox(2.0, 2.0, 2.0);
    const MyVoxel::Display_ResourceId meshResourceId =
        MyVoxel::Display_ShapeBuilder::createResource(box, resourceManager, grayColor());

    std::vector<MyVoxel::Display_ResourceId> resourceIds;
    resourceIds.push_back(meshResourceId);
    resourceIds.push_back(meshResourceId);

    runner.check(meshResourceId != 0, "WireRejectMesh.meshResourceCreated");
    runner.check(MyVoxel::Display_WireBuilder::createObject(wire, resourceIds, objectManager) == 0, "WireRejectMesh.rejected");
    runner.check(objectManager.objectCount() == 0U, "WireRejectMesh.noObjectLeak");
}

/// Box Shape到Mesh Resource

void testShapeResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(2.0, 4.0, 6.0);

    runner.check(topology.isValid(), "ShapeResource.topologyValid");
    runner.check(MyVoxel::Display_ShapeBuilder::supports(topology), "ShapeResource.supported");

    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_ShapeBuilder::createResource(topology, resourceManager, grayColor());

    runner.check(resourceId != 0, "ShapeResource.created");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> resource =
        resourceManager.meshResource(resourceId);

    runner.check(resource && resource->isValid(), "ShapeResource.valid");
    runner.check(resource && resource->triangleCount() == 12U, "ShapeResource.triangleCount");
    runner.check(resource && resource->vertexCount() == 36U, "ShapeResource.vertexCount");

    if (resource)
    {
        runner.check(nearPoint(resource->localBounds().minimum(), MyMath::Vector3(-1.0, -2.0, -3.0)), "ShapeResource.boundsMinimum");
        runner.check(nearPoint(resource->localBounds().maximum(), MyMath::Vector3(1.0, 2.0, 3.0)), "ShapeResource.boundsMaximum");
    }
}

/// Shape实例到Mesh Display_Object

void testShapeObject(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(2.0, 4.0, 6.0);
    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_ShapeBuilder::createResource(topology, resourceManager, blueColor());

    const MyMath::Matrix4 transform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));
    const MyVoxel::Shape shape(topology, transform);

    runner.check(shape.isValid(), "ShapeObject.shapeValid");

    const MyVoxel::Display_ObjectId objectId =
        MyVoxel::Display_ShapeBuilder::createObject(shape, resourceId, objectManager, MyVoxel::Display_ObjectUsage::Static);

    runner.check(objectId != 0, "ShapeObject.created");

    const MyVoxel::Display_Object object = objectManager.object(objectId);

    runner.check(object.isValid(), "ShapeObject.valid");
    runner.check(object.resourceKind() == MyVoxel::Display_ResourceKind::Mesh, "ShapeObject.kind");
    runner.check(object.partCount() == 1U, "ShapeObject.partCount");
    runner.check(object.hasPart(0), "ShapeObject.hasPart0");
    runner.check(object.localToWorld().isEqualTo(transform, 0.0), "ShapeObject.transform");

    if (object.hasPart(0))
    {
        runner.check(object.part(0).resourceId == resourceId, "ShapeObject.resourceId");
    }

    runner.check(nearPoint(object.localBounds().minimum(), MyMath::Vector3(-1.0, -2.0, -3.0)), "ShapeObject.localMinimum");
    runner.check(nearPoint(object.localBounds().maximum(), MyMath::Vector3(1.0, 2.0, 3.0)), "ShapeObject.localMaximum");
    runner.check(nearPoint(object.worldBounds().minimum(), MyMath::Vector3(9.0, 18.0, 27.0)), "ShapeObject.worldMinimum");
    runner.check(nearPoint(object.worldBounds().maximum(), MyMath::Vector3(11.0, 22.0, 33.0)), "ShapeObject.worldMaximum");
}

/// Shape Builder必须拒绝Line Resource

void testShapeRejectLineResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(2.0, 2.0, 2.0);
    const MyVoxel::Shape shape(topology);

    const MyVoxel::Topology_Edge edge =
        makeLineEdge(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(1.0, 0.0, 0.0));
    const MyVoxel::Display_ResourceId lineResourceId =
        MyVoxel::Display_CurveBuilder::createResource(edge, resourceManager, whiteColor());

    runner.check(lineResourceId != 0, "ShapeRejectLine.lineResourceCreated");
    runner.check(MyVoxel::Display_ShapeBuilder::createObject(shape, lineResourceId, objectManager) == 0, "ShapeRejectLine.rejected");
    runner.check(objectManager.objectCount() == 0U, "ShapeRejectLine.noObjectLeak");
}

/// 一个Shape Resource可以复用于多个Shape实例

void testShapeResourceSharing(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(4.0, 6.0, 8.0);
    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_ShapeBuilder::createResource(topology, resourceManager, grayColor());

    const MyMath::Matrix4 transformA =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 0.0, 0.0));
    const MyMath::Matrix4 transformB =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(-10.0, 5.0, 2.0));
    const MyVoxel::Shape shapeA(topology, transformA);
    const MyVoxel::Shape shapeB(topology, transformB);

    const MyVoxel::Display_ObjectId objectIdA =
        MyVoxel::Display_ShapeBuilder::createObject(shapeA, resourceId, objectManager);
    const MyVoxel::Display_ObjectId objectIdB =
        MyVoxel::Display_ShapeBuilder::createObject(shapeB, resourceId, objectManager);

    runner.check(resourceId != 0, "ShapeSharing.resourceCreated");
    runner.check(objectIdA != 0 && objectIdB != 0, "ShapeSharing.objectsCreated");
    runner.check(objectIdA != objectIdB, "ShapeSharing.objectIdsDistinct");
    runner.check(resourceManager.resourceCount() == 1U, "ShapeSharing.singleResource");
    runner.check(objectManager.objectCount() == 2U, "ShapeSharing.twoObjects");

    const MyVoxel::Display_Object objectA = objectManager.object(objectIdA);
    const MyVoxel::Display_Object objectB = objectManager.object(objectIdB);

    runner.check(objectA.part(0).resourceId == resourceId, "ShapeSharing.objectAResource");
    runner.check(objectB.part(0).resourceId == resourceId, "ShapeSharing.objectBResource");
    runner.check(objectA.part(0).resource.get() == objectB.part(0).resource.get(), "ShapeSharing.sameResourcePointer");
    runner.check(objectA.localToWorld().isEqualTo(transformA, 0.0), "ShapeSharing.transformA");
    runner.check(objectB.localToWorld().isEqualTo(transformB, 0.0), "ShapeSharing.transformB");

    runner.check(resourceManager.remove(resourceId), "ShapeSharing.removeDirectoryEntry");
    runner.check(!resourceManager.contains(resourceId), "ShapeSharing.directoryEntryGone");

    const MyVoxel::Display_Object retainedA = objectManager.object(objectIdA);
    const MyVoxel::Display_Object retainedB = objectManager.object(objectIdB);

    runner.check(retainedA.isValid() && retainedB.isValid(), "ShapeSharing.objectsSurviveRemoval");
    runner.check(retainedA.part(0).resource.get() == retainedB.part(0).resource.get(), "ShapeSharing.resourceStillShared");
}

/// Revolved Shape路径

void testRevolvedShapeResource(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    const MyVoxel::Topology_Wire profile = makeClosedRevolvedProfile();

    runner.check(profile.isValid(), "Revolved.profileValid");
    runner.check(profile.isClosed(), "Revolved.profileClosed");
    runner.check(profile.edgeCount() == 4U, "Revolved.profileEdgeCount");

    const MyVoxel::Topology_Shape topology =
        MyVoxel::Modeling::createRevolved(profile, GeometryTolerance);

    runner.check(topology.isValid(), "Revolved.topologyValid");
    runner.check(MyVoxel::Display_ShapeBuilder::supports(topology), "Revolved.builderSupports");

    const MyVoxel::Display_ResourceId resourceId =
        MyVoxel::Display_ShapeBuilder::createResource(topology, resourceManager, redColor());

    runner.check(resourceId != 0, "Revolved.resourceCreated");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> resource =
        resourceManager.meshResource(resourceId);

    runner.check(resource && resource->isValid(), "Revolved.resourceValid");
    runner.check(resource && resource->triangleCount() > 0U, "Revolved.hasTriangles");
    runner.check(resource && resource->vertexCount() == resource->triangleCount() * 3U, "Revolved.vertexTriangleConsistency");

    if (resource)
    {
        runner.check(nearPoint(resource->localBounds().minimum(), MyMath::Vector3(-2.0, -2.0, -1.0)), "Revolved.boundsMinimum");
        runner.check(nearPoint(resource->localBounds().maximum(), MyMath::Vector3(2.0, 2.0, 1.0)), "Revolved.boundsMaximum");
    }
}

}

int main()
{
    std::cout << "============================================================" << std::endl;
    std::cout << "MyVoxel display builder test" << std::endl;
    std::cout << "============================================================" << std::endl;

    TestRunner runner;

    testCurveBuildOptions(runner);
    testLineResource(runner);
    testReversedLineResource(runner);
    testArcResource(runner);
    testCurveObject(runner);
    testCurveResourceSharing(runner);
    testCurveRejectMeshResource(runner);
    testWireResources(runner);
    testReversedWireResources(runner);
    testWireObject(runner);
    testWireRejectMeshResource(runner);
    testShapeResource(runner);
    testShapeObject(runner);
    testShapeRejectLineResource(runner);
    testShapeResourceSharing(runner);
    testRevolvedShapeResource(runner);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << runner.passed() << std::endl;
    std::cout << "Failed: " << runner.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return runner.failed() == 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}