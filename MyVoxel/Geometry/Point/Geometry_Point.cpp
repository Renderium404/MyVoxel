#include "Geometry_Point.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Geometry_Point::Geometry_Point(const MyMath::Vector3& position)
    : m_position(position)
{
    MYVOXEL_ASSERT_MESSAGE(position.isFinite(), "Geometry_Point position must be finite.");
}

/// 几何数据

const MyMath::Vector3& Geometry_Point::position() const
{
    return m_position;
}

}