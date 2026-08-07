#include "Topology_Face.h"
#include <cmath>
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断标量是否为有限非负值。
bool isFiniteNonNegativeValue(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

// 判断点是否为局部UV平面中的有限参数点。
bool isFiniteParameterPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

// 判断包围盒是否为局部UV平面中的有效参数范围。
bool isParameterBounds(const MyVoxel::Bounds3& bounds)
{
    return bounds.isValid() && bounds.minimum().z() == 0.0 && bounds.maximum().z() == 0.0;
}

}

namespace MyVoxel
{

Topology_Face::Topology_Face()
    : m_reversed(false)
    , m_valid(false)
{
}

Topology_Face::Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& surface, const Topology_Wire& outerWire)
    : m_surface(surface)
    , m_outerWire(outerWire)
    , m_reversed(false)
    , m_valid(false)
{
    rebuild();
}

Topology_Face::Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& surface, const Topology_Wire& outerWire, const std::vector<Topology_Wire>& innerWires)
    : m_surface(surface)
    , m_outerWire(outerWire)
    , m_innerWires(innerWires)
    , m_reversed(false)
    , m_valid(false)
{
    rebuild();
}

/// 状态判断

bool Topology_Face::isValid() const
{
    return m_valid;
}

bool Topology_Face::isNull() const
{
    return m_surface.isNull();
}

Topology_Face::operator bool() const
{
    return isValid();
}

bool Topology_Face::sharesSurfaceWith(const Topology_Face& other) const
{
    return m_surface == other.m_surface;
}

bool Topology_Face::isReversed() const
{
    return isValid() && m_reversed;
}

/// 曲面几何

const Geometry_Surface& Topology_Face::surface() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the surface of an invalid Topology_Face.");
    return *m_surface;
}

const Geometry_Surface* Topology_Face::surfacePointer() const
{
    return m_surface.get();
}

const Foundation::RefPtr<const Geometry_Surface>& Topology_Face::surfaceResource() const
{
    return m_surface;
}

SurfaceKind Topology_Face::surfaceKind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the surface kind of an invalid Topology_Face.");
    return m_surface->kind();
}

/// 边界拓扑

const Topology_Wire& Topology_Face::outerWire() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the outer Wire of an invalid Topology_Face.");
    return m_outerWire;
}

std::size_t Topology_Face::innerWireCount() const
{
    return m_innerWires.size();
}

const Topology_Wire& Topology_Face::innerWire(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an inner Wire of an invalid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(index < m_innerWires.size(), "Topology_Face inner Wire index is out of range.");
    return m_innerWires[index];
}

const std::vector<Topology_Wire>& Topology_Face::innerWires() const
{
    return m_innerWires;
}

/// 参数域与局部空间

const Bounds3& Topology_Face::parameterBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the parameter bounds of an invalid Topology_Face.");
    return m_parameterBounds;
}

const Bounds3& Topology_Face::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Topology_Face.");
    return m_localBounds;
}

MyMath::Vector3 Topology_Face::pointAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteParameterPoint(parameterPoint), "Topology_Face parameter point must be finite and lie in the local UV plane.");
    return m_surface->pointAt(parameterPoint.x(), parameterPoint.y());
}

MyMath::Vector3 Topology_Face::normalAt(const MyMath::Vector3& parameterPoint) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteParameterPoint(parameterPoint), "Topology_Face parameter point must be finite and lie in the local UV plane.");
    const MyMath::Vector3 normal = m_surface->normalAt(parameterPoint.x(), parameterPoint.y());
    return m_reversed ? normal * -1.0 : normal;
}

/// 参数域查询

bool Topology_Face::containsParameterPoint(const MyMath::Vector3& parameterPoint, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteParameterPoint(parameterPoint), "Topology_Face parameter point must be finite and lie in the local UV plane.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(tolerance), "Topology_Face query tolerance must be finite and non-negative.");

    const ShapeRelation outerRelation = classifyWirePoint(m_outerWire, parameterPoint, tolerance);

    if (outerRelation == ShapeRelation::Outside)
    {
        return false;
    }

    if (outerRelation == ShapeRelation::Intersecting)
    {
        return true;
    }

    for (std::size_t index = 0; index < m_innerWires.size(); ++index)
    {
        const ShapeRelation innerRelation = classifyWirePoint(m_innerWires[index], parameterPoint, tolerance);

        if (innerRelation == ShapeRelation::Intersecting)
        {
            return true;
        }

        if (innerRelation == ShapeRelation::Inside)
        {
            return false;
        }
    }

    return true;
}

ShapeRelation Topology_Face::classifyParameterBounds(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(isParameterBounds(bounds), "Topology_Face query bounds must be valid and lie in the local UV plane.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(tolerance), "Topology_Face query tolerance must be finite and non-negative.");

    const ShapeRelation outerRelation = m_outerWire.classifyBounds(bounds, tolerance);

    if (outerRelation != ShapeRelation::Inside)
    {
        return outerRelation;
    }

    for (std::size_t index = 0; index < m_innerWires.size(); ++index)
    {
        const ShapeRelation innerRelation = m_innerWires[index].classifyBounds(bounds, tolerance);

        if (innerRelation == ShapeRelation::Intersecting)
        {
            return ShapeRelation::Intersecting;
        }

        if (innerRelation == ShapeRelation::Inside)
        {
            return ShapeRelation::Outside;
        }
    }

    return ShapeRelation::Inside;
}

/// 拓扑创建

Topology_Face Topology_Face::reversed() const
{
    if (!isValid())
    {
        return Topology_Face();
    }

    Topology_Face result = *this;
    result.m_outerWire = result.m_outerWire.reversed();

    for (std::size_t index = 0; index < result.m_innerWires.size(); ++index)
    {
        result.m_innerWires[index] = result.m_innerWires[index].reversed();
    }

    result.m_reversed = !result.m_reversed;
    return result;
}

/// 缓存建立

void Topology_Face::rebuild()
{
    m_parameterBounds.clear();
    m_localBounds.clear();
    m_reversed = false;
    m_valid = false;

    if (m_surface.isNull() || !m_outerWire.isValid() || !m_outerWire.isClosed() || !m_outerWire.hasArea())
    {
        return;
    }

    for (std::size_t index = 0; index < m_innerWires.size(); ++index)
    {
        if (!m_innerWires[index].isValid() || !m_innerWires[index].isClosed() || !m_innerWires[index].hasArea())
        {
            return;
        }
    }

    if (!m_outerWire.isCounterClockwise())
    {
        m_outerWire = m_outerWire.reversed();
    }

    for (std::size_t index = 0; index < m_innerWires.size(); ++index)
    {
        if (m_innerWires[index].isCounterClockwise())
        {
            m_innerWires[index] = m_innerWires[index].reversed();
        }
    }

    m_parameterBounds = m_outerWire.bounds();
    m_localBounds = m_surface->localBounds(m_parameterBounds);
    m_valid = m_parameterBounds.isValid() && m_localBounds.isValid();

    MYVOXEL_ASSERT_MESSAGE(m_valid, "Topology_Face construction produced invalid parameter or local bounds.");
}

ShapeRelation Topology_Face::classifyWirePoint(const Topology_Wire& wire, const MyMath::Vector3& point, double tolerance)
{
    return wire.classifyBounds(Bounds3(point, point), tolerance);
}

}