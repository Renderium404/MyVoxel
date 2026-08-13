#include "ShapeFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace
{

const double HalfScale = 0.5; // 标准Box、Cylinder和ConeFrustum完整尺寸转换为半尺寸使用的固定比例。
const double Pi = 3.1415926535897932384626433832795; // 完整圆特征Geometry_Arc使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆特征的扫掠角。
const double CircleClosureToleranceScale = 64.0; // 完整圆首尾三角函数舍入误差相对double epsilon的安全放大系数。
const double TransformedArcToleranceScale = 128.0; // 解析Arc经过刚体或统一缩放重建后端点舍入误差使用的安全放大系数。

struct MappedVertex
{
    MappedVertex(const MyVoxel::Topology_Vertex& sourceValue, const MyVoxel::Topology_Vertex& targetValue)
        : source(sourceValue)
        , target(targetValue)
    {
    }

    MyVoxel::Topology_Vertex source; // 原始FeatureSet中的共享拓扑点身份。
    MyVoxel::Topology_Vertex target; // 查询坐标系中重新建立的共享拓扑点身份。
};

// 使用两个共享Topology_Vertex建立新的直线特征Edge。
MyVoxel::Topology_Edge makeLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(new MyVoxel::Geometry_Line(startVertex.point(), endVertex.point()));
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, 0.0);
}

// 返回完整圆Geometry_Arc首尾匹配使用的尺度自适应容差。
double circleClosureTolerance(const MyMath::Vector3& center, double radius)
{
    const double scale = (std::max)(1.0, (std::max)(std::fabs(radius),
        (std::max)(std::fabs(center.x()), (std::max)(std::fabs(center.y()), std::fabs(center.z())))));
    return scale * (std::numeric_limits<double>::epsilon)() * CircleClosureToleranceScale;
}

// 返回变换后Arc拓扑端点与重新参数化Geometry_Arc端点匹配使用的尺度自适应容差。
double transformedArcTolerance(const MyVoxel::Geometry_Arc& arc)
{
    const MyVoxel::Bounds3& bounds = arc.bounds();
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();
    const double scale = (std::max)(1.0, (std::max)(arc.radius(),
        (std::max)(std::fabs(minimum.x()), (std::max)(std::fabs(minimum.y()), (std::max)(std::fabs(minimum.z()),
        (std::max)(std::fabs(maximum.x()), (std::max)(std::fabs(maximum.y()), std::fabs(maximum.z()))))))));
    return scale * (std::numeric_limits<double>::epsilon)() * TransformedArcToleranceScale;
}

// 建立位于世界XY平面且使用一个共享seam Vertex闭合的完整圆特征Edge。
MyVoxel::Topology_Edge makeCircleEdge(const MyMath::Vector3& center, double radius)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Arc> arc(new MyVoxel::Geometry_Arc(center, radius, 0.0, TwoPi));
    const MyVoxel::Topology_Vertex seamVertex(arc->startPoint());
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry = arc;
    return MyVoxel::Topology_Edge(seamVertex, seamVertex, geometry, circleClosureTolerance(center, radius));
}

// 提取标准轴对齐Box的8个共享角点和12条直线棱。
MyVoxel::VoxelFeatureSet extractBox(const MyVoxel::Geometry_Box& box)
{
    const double hx = box.sizeX() * HalfScale;
    const double hy = box.sizeY() * HalfScale;
    const double hz = box.sizeZ() * HalfScale;
    const MyVoxel::Topology_Vertex v000(MyMath::Vector3(-hx, -hy, -hz));
    const MyVoxel::Topology_Vertex v100(MyMath::Vector3( hx, -hy, -hz));
    const MyVoxel::Topology_Vertex v110(MyMath::Vector3( hx,  hy, -hz));
    const MyVoxel::Topology_Vertex v010(MyMath::Vector3(-hx,  hy, -hz));
    const MyVoxel::Topology_Vertex v001(MyMath::Vector3(-hx, -hy,  hz));
    const MyVoxel::Topology_Vertex v101(MyMath::Vector3( hx, -hy,  hz));
    const MyVoxel::Topology_Vertex v111(MyMath::Vector3( hx,  hy,  hz));
    const MyVoxel::Topology_Vertex v011(MyMath::Vector3(-hx,  hy,  hz));
    MyVoxel::VoxelFeatureSet features;

    features.addEdge(makeLineEdge(v000, v100)); features.addEdge(makeLineEdge(v100, v110));
    features.addEdge(makeLineEdge(v110, v010)); features.addEdge(makeLineEdge(v010, v000));
    features.addEdge(makeLineEdge(v001, v101)); features.addEdge(makeLineEdge(v101, v111));
    features.addEdge(makeLineEdge(v111, v011)); features.addEdge(makeLineEdge(v011, v001));
    features.addEdge(makeLineEdge(v000, v001)); features.addEdge(makeLineEdge(v100, v101));
    features.addEdge(makeLineEdge(v110, v111)); features.addEdge(makeLineEdge(v010, v011));
    return features;
}

// Sphere边界处处光滑，不产生显式锐特征。
MyVoxel::VoxelFeatureSet extractSphere(const MyVoxel::Geometry_Sphere& sphere)
{
    (void)sphere;
    return MyVoxel::VoxelFeatureSet();
}

// 提取标准Z轴Cylinder上下端面与侧面的两条完整圆锐边。
MyVoxel::VoxelFeatureSet extractCylinder(const MyVoxel::Geometry_Cylinder& cylinder)
{
    const double halfHeight = cylinder.height() * HalfScale;
    MyVoxel::VoxelFeatureSet features;
    features.addEdge(makeCircleEdge(MyMath::Vector3(0.0, 0.0, -halfHeight), cylinder.radius()));
    features.addEdge(makeCircleEdge(MyMath::Vector3(0.0, 0.0,  halfHeight), cylinder.radius()));
    return features;
}

// 提取标准Z轴ConeFrustum的端面圆锐边；零半径端作为真实孤立Apex FeatureVertex保留。
MyVoxel::VoxelFeatureSet extractConeFrustum(const MyVoxel::Geometry_ConeFrustum& cone)
{
    const double halfHeight = cone.height() * HalfScale;
    MyVoxel::VoxelFeatureSet features;

    if (cone.bottomRadius() > 0.0)
    {
        features.addEdge(makeCircleEdge(MyMath::Vector3(0.0, 0.0, -halfHeight), cone.bottomRadius()));
    }
    else
    {
        features.addVertex(MyVoxel::Topology_Vertex(MyMath::Vector3(0.0, 0.0, -halfHeight)));
    }

    if (cone.topRadius() > 0.0)
    {
        features.addEdge(makeCircleEdge(MyMath::Vector3(0.0, 0.0, halfHeight), cone.topRadius()));
    }
    else
    {
        features.addVertex(MyVoxel::Topology_Vertex(MyMath::Vector3(0.0, 0.0, halfHeight)));
    }

    return features;
}

// 返回原始Topology_Vertex身份对应的查询空间Topology_Vertex。
const MyVoxel::Topology_Vertex& mappedVertex(const std::vector<MappedVertex>& vertices, const MyVoxel::Topology_Vertex& source)
{
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (vertices[index].source.isSame(source))
        {
            return vertices[index].target;
        }
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Transformed Feature Edge references a Vertex outside the source FeatureSet.");
    return vertices.front().target;
}

// 将Line几何变换到查询坐标系；任意仿射均保持有限直线段类型。
MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> transformLine(const MyVoxel::Geometry_Line& line, const MyMath::Matrix4& localToQuery)
{
    return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(localToQuery.transformPoint(line.startPoint()), localToQuery.transformPoint(line.endPoint())));
}

// 将Arc几何通过刚体或统一缩放变换到查询坐标系，保持圆弧参数方向和解析类型。
MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> transformArc(const MyVoxel::Geometry_Arc& arc, const MyMath::Matrix4& localToQuery)
{
    const MyMath::Vector3 center = localToQuery.transformPoint(arc.center());
    MyMath::Vector3 xAxis = localToQuery.transformVector(arc.xAxis());
    MyMath::Vector3 yAxis = localToQuery.transformVector(arc.yAxis());
    MyMath::Vector3 zAxis = localToQuery.transformVector(arc.normal());
    const double scale = xAxis.length();

    MYVOXEL_ASSERT_MESSAGE(scale > 0.0, "Feature Arc similarity transform must preserve a non-zero metric scale.");
    const bool normalizedX = xAxis.normalize(0.0);
    const bool normalizedY = yAxis.normalize(0.0);
    const bool normalizedZ = zAxis.normalize(0.0);
    MYVOXEL_ASSERT_MESSAGE(normalizedX && normalizedY && normalizedZ, "Feature Arc transformed coordinate axes must remain non-zero.");

    const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(center, xAxis, yAxis, zAxis);
    return MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Arc(coordinateSystem, arc.radius() * scale, arc.startAngle(), arc.sweepAngle()));
}

// 将局部FeatureSet重新建立到ShapeQuery查询坐标系，并保持Edge共享Vertex身份和孤立FeatureVertex。
MyVoxel::VoxelFeatureSet transformFeatures(const MyVoxel::VoxelFeatureSet& source, const MyMath::Matrix4& localToQuery)
{
    if (source.isEmpty())
    {
        return MyVoxel::VoxelFeatureSet();
    }

    std::vector<MappedVertex> mappedVertices;
    mappedVertices.reserve(source.vertexCount());

    for (std::size_t index = 0; index < source.vertices().size(); ++index)
    {
        const MyVoxel::Topology_Vertex& sourceVertex = source.vertices()[index];
        mappedVertices.push_back(MappedVertex(sourceVertex, MyVoxel::Topology_Vertex(localToQuery.transformPoint(sourceVertex.point()))));
    }

    MyVoxel::VoxelFeatureSet result;

    // 必须先加入全部映射Vertex，不能只依赖addEdge自动加入端点，否则Cone Apex等孤立FeatureVertex会在查询空间变换后丢失。
    for (std::size_t index = 0; index < mappedVertices.size(); ++index)
    {
        result.addVertex(mappedVertices[index].target);
    }

    for (std::size_t index = 0; index < source.edges().size(); ++index)
    {
        const MyVoxel::Topology_Edge& sourceEdge = source.edges()[index];
        const MyVoxel::Topology_Vertex& startVertex = mappedVertex(mappedVertices, sourceEdge.startVertex());
        const MyVoxel::Topology_Vertex& endVertex = mappedVertex(mappedVertices, sourceEdge.endVertex());
        MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry;
        double connectionTolerance = 0.0;

        if (sourceEdge.geometry().kind() == MyVoxel::CurveKind::Line)
        {
            geometry = transformLine(static_cast<const MyVoxel::Geometry_Line&>(sourceEdge.geometry()), localToQuery);
        }
        else if (sourceEdge.geometry().kind() == MyVoxel::CurveKind::Arc)
        {
            geometry = transformArc(static_cast<const MyVoxel::Geometry_Arc&>(sourceEdge.geometry()), localToQuery);
            connectionTolerance = transformedArcTolerance(static_cast<const MyVoxel::Geometry_Arc&>(*geometry));
        }

        MYVOXEL_ASSERT_MESSAGE(geometry, "ShapeFeatureExtractor encountered an unsupported Feature Curve kind during query-space transformation.");
        if (geometry)
        {
            result.addEdge(MyVoxel::Topology_Edge(startVertex, endVertex, geometry, connectionTolerance));
        }
    }

    return result;
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 能力查询

bool ShapeFeatureExtractor::supports(const Topology_Shape& shape)
{
    if (!shape.isValid())
    {
        return false;
    }

    const ShapeKind kind = shape.geometry().kind();
    return kind == ShapeKind::Box || kind == ShapeKind::Sphere || kind == ShapeKind::Cylinder || kind == ShapeKind::ConeFrustum;
}

bool ShapeFeatureExtractor::supports(const ShapeQuery& query)
{
    return query.isValid() && query.supportsSignedDistance() && supports(query.topology());
}

/// 特征提取

VoxelFeatureSet ShapeFeatureExtractor::extract(const Topology_Shape& shape)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeFeatureExtractor requires a valid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(supports(shape), "ShapeFeatureExtractor does not support the current Geometry_Shape kind.");

    if (!shape.isValid() || !supports(shape))
    {
        return VoxelFeatureSet();
    }

    const Geometry_Shape& geometry = shape.geometry();

    switch (geometry.kind())
    {
    case ShapeKind::Box:
        return extractBox(static_cast<const Geometry_Box&>(geometry));
    case ShapeKind::Sphere:
        return extractSphere(static_cast<const Geometry_Sphere&>(geometry));
    case ShapeKind::Cylinder:
        return extractCylinder(static_cast<const Geometry_Cylinder&>(geometry));
    case ShapeKind::ConeFrustum:
        return extractConeFrustum(static_cast<const Geometry_ConeFrustum&>(geometry));
    default:
        break;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "ShapeFeatureExtractor reached an unsupported Geometry_Shape kind.");
    return VoxelFeatureSet();
}

VoxelFeatureSet ShapeFeatureExtractor::extract(const ShapeQuery& query)
{
    MYVOXEL_ASSERT_MESSAGE(query.isValid(), "ShapeFeatureExtractor requires a valid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(supports(query), "ShapeFeatureExtractor query-space extraction requires supported geometry and a rigid or uniform-scale query transform.");

    if (!supports(query))
    {
        return VoxelFeatureSet();
    }

    return transformFeatures(extract(query.topology()), query.localToQuery());
}

}
}