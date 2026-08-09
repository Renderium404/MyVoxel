#include "Topology_Edge.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Topology/Topology_TEdge.h"

namespace
{

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断拓扑顶点几何位置是否在给定容差内匹配曲线参数点。
bool vertexMatchesCurvePoint(const MyVoxel::Topology_Vertex& vertex, const MyVoxel::Geometry_Curve& geometry, double parameter, double tolerance)
{
    if (!vertex.isValid())
    {
        return false;
    }

    const MyMath::Vector3 curvePoint = geometry.pointAt(parameter);
    return curvePoint.isFinite() && vertex.geometry().position().distanceTo(curvePoint) <= tolerance;
}

// 验证拓扑边定义并创建新的共享Topology_TEdge实体。
MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject> createTEdge(
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>& geometry, double firstParameter, double lastParameter,
    const MyVoxel::Topology_Vertex& firstVertex, const MyVoxel::Topology_Vertex& lastVertex, double geometryTolerance)
{
    const bool validParameters = isFiniteValue(firstParameter) && isFiniteValue(lastParameter) &&
                                 firstParameter >= 0.0 && firstParameter < lastParameter && lastParameter <= 1.0;
    const bool validTolerance = isFiniteValue(geometryTolerance) && geometryTolerance >= 0.0;
    const bool validBaseData = geometry && firstVertex.isValid() && lastVertex.isValid();

    MYVOXEL_ASSERT_MESSAGE(validParameters, "Topology_Edge parameters must satisfy 0 <= firstParameter < lastParameter <= 1.");
    MYVOXEL_ASSERT_MESSAGE(validTolerance, "Topology_Edge geometry tolerance must be finite and non-negative.");
    MYVOXEL_ASSERT_MESSAGE(validBaseData, "Topology_Edge requires non-null geometry and valid topology vertices.");

    if (!validParameters || !validTolerance || !validBaseData)
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>();
    }

    const bool firstMatches = vertexMatchesCurvePoint(firstVertex, *geometry, firstParameter, geometryTolerance);
    const bool lastMatches = vertexMatchesCurvePoint(lastVertex, *geometry, lastParameter, geometryTolerance);

    MYVOXEL_ASSERT_MESSAGE(firstMatches, "Topology_Edge first vertex must match geometry at firstParameter within geometryTolerance.");
    MYVOXEL_ASSERT_MESSAGE(lastMatches, "Topology_Edge last vertex must match geometry at lastParameter within geometryTolerance.");

    if (!firstMatches || !lastMatches)
    {
        return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>();
    }

    return MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>(
        new MyVoxel::Topology_TEdge(geometry, firstParameter, lastParameter, firstVertex, lastVertex));
}

}

namespace MyVoxel
{

Topology_Edge::Topology_Edge()
{
}

Topology_Edge::Topology_Edge(const Foundation::RefPtr<const Geometry_Curve>& geometry, double firstParameter, double lastParameter,
                             const Topology_Vertex& firstVertex, const Topology_Vertex& lastVertex, double geometryTolerance)
    : Topology_Object(createTEdge(geometry, firstParameter, lastParameter, firstVertex, lastVertex, geometryTolerance), Topology_Orientation::Forward)
{
}

Topology_Edge::Topology_Edge(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 几何支撑

const Geometry_Curve& Topology_Edge::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Topology_Edge.");
    return *tEdge().geometry();
}

double Topology_Edge::parameterStart() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access parameters of an invalid Topology_Edge.");
    return isForward() ? tEdge().firstParameter() : tEdge().lastParameter();
}

double Topology_Edge::parameterEnd() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access parameters of an invalid Topology_Edge.");
    return isForward() ? tEdge().lastParameter() : tEdge().firstParameter();
}

/// 有向拓扑端点

const Topology_Vertex& Topology_Edge::startVertex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the start vertex of an invalid Topology_Edge.");
    return isForward() ? tEdge().firstVertex() : tEdge().lastVertex();
}

const Topology_Vertex& Topology_Edge::endVertex() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the end vertex of an invalid Topology_Edge.");
    return isForward() ? tEdge().lastVertex() : tEdge().firstVertex();
}

/// 有向几何查询

MyMath::Vector3 Topology_Edge::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Edge.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(t) && t >= 0.0 && t <= 1.0, "Topology_Edge parameter must be finite and lie in [0,1].");

    if (!isValid() || !isFiniteValue(t) || t < 0.0 || t > 1.0)
    {
        return MyMath::Vector3();
    }

    const double curveParameter = parameterStart() + (parameterEnd() - parameterStart()) * t;
    return geometry().pointAt(curveParameter);
}

MyMath::Vector3 Topology_Edge::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Edge.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(t) && t >= 0.0 && t <= 1.0, "Topology_Edge parameter must be finite and lie in [0,1].");

    if (!isValid() || !isFiniteValue(t) || t < 0.0 || t > 1.0)
    {
        return MyMath::Vector3();
    }

    const double curveParameter = parameterStart() + (parameterEnd() - parameterStart()) * t;
    const MyMath::Vector3 tangent = geometry().tangentAt(curveParameter);
    return isForward() ? tangent : -tangent;
}

/// 方向操作

Topology_Edge Topology_Edge::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Topology_Edge.");

    if (!isValid())
    {
        return Topology_Edge();
    }

    return Topology_Edge(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TEdge& Topology_Edge::tEdge() const
{
    return *static_cast<const Topology_TEdge*>(tObject().get());
}

}
