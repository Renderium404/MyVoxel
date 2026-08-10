#include "Topology_TVertex.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_TVertex::Topology_TVertex(const MyMath::Vector3& point)
    : m_point(point)
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Topology_TVertex point must be finite.");
}

/// 几何数据

const MyMath::Vector3& Topology_TVertex::point() const
{
    return m_point;
}

}