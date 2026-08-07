#include "Geometry_Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Geometry_Shape::Geometry_Shape()
{
}

Geometry_Shape::Geometry_Shape(const Bounds3& localBounds)
    : m_localBounds(localBounds)
{
}

/// 几何属性

const Bounds3& Geometry_Shape::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(m_localBounds.isValid(), "Geometry_Shape local bounds have not been initialized.");
    return m_localBounds;
}

/// 派生类缓存维护

void Geometry_Shape::setLocalBounds(const Bounds3& localBounds)
{
    MYVOXEL_ASSERT_MESSAGE(localBounds.isValid(), "Geometry_Shape local bounds must be valid.");
    m_localBounds = localBounds;
}

void Geometry_Shape::clearLocalBounds()
{
    m_localBounds.clear();
}

}
