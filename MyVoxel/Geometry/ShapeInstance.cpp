#include "ShapeInstance.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{
namespace Geometry
{

ShapeInstance::ShapeInstance()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
}

ShapeInstance::ShapeInstance(const Shape& shape)
    : m_shape(shape)
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(shape.isValid())
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeInstance requires a valid Shape.");

    if (m_valid)
    {
        m_worldBounds = shape.localBounds();
    }
}

ShapeInstance::ShapeInstance(const Shape& shape, const MyMath::Matrix4& localToWorld)
    : m_shape(shape)
    , m_localToWorld(localToWorld)
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeInstance requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "ShapeInstance transform must be affine.");

    if (!shape.isValid() || !localToWorld.isAffine())
    {
        return;
    }

    const bool inverted = localToWorld.inverted(m_worldToLocal);

    MYVOXEL_ASSERT_MESSAGE(inverted, "ShapeInstance transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    m_worldBounds = shape.localBounds().transformed(localToWorld);
    m_valid = m_worldBounds.isValid();
}

/// 状态判断

bool ShapeInstance::isValid() const
{
    return m_valid;
}

/// 实例数据

const Shape& ShapeInstance::shape() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the Shape of an invalid ShapeInstance.");
    return m_shape;
}

const MyMath::Matrix4& ShapeInstance::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid ShapeInstance.");
    return m_localToWorld;
}

const MyMath::Matrix4& ShapeInstance::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid ShapeInstance.");
    return m_worldToLocal;
}

const Bounds3& ShapeInstance::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid ShapeInstance.");
    return m_worldBounds;
}

/// 空间查询

bool ShapeInstance::containsWorldPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "ShapeInstance query point must be finite.");

    const MyMath::Vector3 localPoint = m_worldToLocal.transformPoint(point);
    return m_shape.containsLocalPoint(localPoint);
}

ShapeRelation ShapeInstance::classifyWorldBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "ShapeInstance query bounds must be valid.");

    const Bounds3 localBounds = bounds.transformed(m_worldToLocal);
    return m_shape.classifyLocalBounds(localBounds);
}

}
}