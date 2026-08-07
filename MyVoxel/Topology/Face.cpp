#include "Face.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断点是否为曲面UV参数平面中的有限参数点。
bool isFiniteParameterPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

}

namespace MyVoxel
{

Face::Face()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_normalToWorld(MyMath::Matrix4::identity())
    , m_valid(false)
{
}

Face::Face(const Topology_Face& topology)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_normalToWorld(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, MyMath::Matrix4::identity());
}

Face::Face(const Topology_Face& topology, const MyMath::Matrix4& localToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_normalToWorld(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, localToWorld);
}

/// 状态判断

bool Face::isValid() const
{
    return m_valid;
}

bool Face::isNull() const
{
    return m_topology.isNull();
}

Face::operator bool() const
{
    return isValid();
}

bool Face::sharesSurfaceWith(const Face& other) const
{
    return m_topology.sharesSurfaceWith(other.m_topology);
}

bool Face::isReversed() const
{
    return isValid() && m_topology.isReversed();
}

/// 局部拓扑与曲面资源

const Topology_Face& Face::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Face.");
    return m_topology;
}

const Geometry_Surface& Face::surface() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the surface of an invalid Face.");
    return m_topology.surface();
}

const Geometry_Surface* Face::surfacePointer() const
{
    return m_topology.surfacePointer();
}

SurfaceKind Face::surfaceKind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the surface kind of an invalid Face.");
    return m_topology.surfaceKind();
}

/// 参数域边界

const Topology_Wire& Face::outerParameterWire() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the outer parameter Wire of an invalid Face.");
    return m_topology.outerWire();
}

std::size_t Face::innerParameterWireCount() const
{
    return m_topology.innerWireCount();
}

const Topology_Wire& Face::innerParameterWire(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an inner parameter Wire of an invalid Face.");
    return m_topology.innerWire(index);
}

const Bounds3& Face::parameterBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the parameter bounds of an invalid Face.");
    return m_topology.parameterBounds();
}

/// 空间数据

const Bounds3& Face::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Face.");
    return m_topology.localBounds();
}

const MyMath::Matrix4& Face::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Face.");
    return m_localToWorld;
}

const MyMath::Matrix4& Face::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Face.");
    return m_worldToLocal;
}

const MyMath::Matrix4& Face::normalToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the normal transform of an invalid Face.");
    return m_normalToWorld;
}

const Bounds3& Face::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Face.");
    return m_worldBounds;
}

/// 参数曲面查询

MyMath::Vector3 Face::localPointAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_topology.pointAt(parameterPoint);
}

MyMath::Vector3 Face::worldPointAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_localToWorld.transformPoint(m_topology.pointAt(parameterPoint));
}

MyMath::Vector3 Face::localNormalAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_topology.normalAt(parameterPoint);
}

MyMath::Vector3 Face::worldNormalAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    const MyMath::Vector3 normal = m_normalToWorld.transformVector(m_topology.normalAt(parameterPoint));
    MYVOXEL_ASSERT_MESSAGE(normal.isFinite() && normal.lengthSquared() > 0.0, "Face transformed normal must be finite and non-zero.");
    return normal.normalized(0.0);
}

MyMath::Vector3 Face::localDerivativeUAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteParameterPoint(parameterPoint), "Face parameter point must be finite and lie in the local UV plane.");
    return m_topology.surface().derivativeUAt(parameterPoint.x(), parameterPoint.y());
}

MyMath::Vector3 Face::localDerivativeVAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteParameterPoint(parameterPoint), "Face parameter point must be finite and lie in the local UV plane.");
    return m_topology.surface().derivativeVAt(parameterPoint.x(), parameterPoint.y());
}

MyMath::Vector3 Face::worldDerivativeUAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_localToWorld.transformVector(localDerivativeUAt(parameterPoint));
}

MyMath::Vector3 Face::worldDerivativeVAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_localToWorld.transformVector(localDerivativeVAt(parameterPoint));
}

/// 参数域查询

bool Face::containsParameterPoint(const MyMath::Vector3& parameterPoint, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_topology.containsParameterPoint(parameterPoint, tolerance);
}

ShapeRelation Face::classifyParameterBounds(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Face.");
    return m_topology.classifyParameterBounds(bounds, tolerance);
}

/// Face创建

Face Face::reversed() const
{
    return isValid() ? Face(m_topology.reversed(), m_localToWorld) : Face();
}

/// 初始化

void Face::initialize(const Topology_Face& topology, const MyMath::Matrix4& localToWorld)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "Face topology must be valid.");
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "Face transform must be affine.");

    if (!topology.isValid() || !localToWorld.isAffine())
    {
        return;
    }

    MyMath::Matrix4 worldToLocal;
    const bool inverted = localToWorld.inverted(worldToLocal);

    MYVOXEL_ASSERT_MESSAGE(inverted, "Face transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    const Bounds3 worldBounds = topology.localBounds().transformed(localToWorld);

    MYVOXEL_ASSERT_MESSAGE(worldBounds.isValid(), "Face transformed world bounds must be valid.");

    if (!worldBounds.isValid())
    {
        return;
    }

    m_topology = topology;
    m_localToWorld = localToWorld;
    m_worldToLocal = worldToLocal;
    m_normalToWorld = worldToLocal.transposed();
    m_worldBounds = worldBounds;
    m_valid = true;
}

}