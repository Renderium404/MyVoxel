#include "VoxelShape.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{

// 将连续坐标转换为指定体素边长下的空间索引。
MyVoxel::VoxelIndex coordinateIndex(double coordinate, double edgeLength)
{
    assert(std::isfinite(coordinate));
    assert(std::isfinite(edgeLength) && edgeLength > 0.0);

    const double value = std::floor(coordinate / edgeLength);
    const double minimum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(value >= minimum && value <= maximum);
    return static_cast<MyVoxel::VoxelIndex>(value);
}

}

namespace MyVoxel
{

VoxelShape::VoxelShape()
    : m_data(std::make_shared<SharedData>())
    , m_transform(MyMath::Matrix4::identity())
{
}

double VoxelShape::baseVoxelEdgeLength() const
{
    return m_data->baseVoxelEdgeLength;
}

void VoxelShape::setBaseVoxelEdgeLength(double edgeLength)
{
    assert(std::isfinite(edgeLength) && edgeLength > 0.0);

    if (m_data->baseVoxelEdgeLength == edgeLength)
    {
        return;
    }

    detach();
    m_data->baseVoxelEdgeLength = edgeLength;
    m_data->forest.clear();
}

VoxelLevel VoxelShape::maximumLevel() const
{
    return m_data->maximumLevel;
}

void VoxelShape::setMaximumLevel(VoxelLevel level)
{
    if (m_data->maximumLevel == level)
    {
        return;
    }

    detach();
    m_data->maximumLevel = level;
    m_data->forest.clear();
}

double VoxelShape::voxelEdgeLength(VoxelLevel level) const
{
    assert(level <= m_data->maximumLevel);
    return std::ldexp(m_data->baseVoxelEdgeLength, -static_cast<int>(level));
}

VoxelNodeForest& VoxelShape::forest()
{
    detach();
    return m_data->forest;
}

const VoxelNodeForest& VoxelShape::forest() const
{
    return m_data->forest;
}

void VoxelShape::clear()
{
    if (m_data->forest.rootCount() == 0)
    {
        return;
    }

    detach();
    m_data->forest.clear();
}

bool VoxelShape::sharesDataWith(const VoxelShape& other) const
{
    return m_data == other.m_data;
}

VoxelCellAddress VoxelShape::localAddressAt(const MyMath::Vector3& point, VoxelLevel level) const
{
    assert(point.isFinite());
    assert(level <= m_data->maximumLevel);

    const double edgeLength = voxelEdgeLength(level);
    const VoxelIndex x = coordinateIndex(point.x(), edgeLength);
    const VoxelIndex y = coordinateIndex(point.y(), edgeLength);
    const VoxelIndex z = coordinateIndex(point.z(), edgeLength);

    return VoxelCellAddress(VoxelCellIndex(x, y, z), level);
}

VoxelState VoxelShape::stateAtLocalPoint(const MyMath::Vector3& point) const
{
    return m_data->forest.state(localAddressAt(point, m_data->maximumLevel));
}

bool VoxelShape::containsLocalPoint(const MyMath::Vector3& point) const
{
    return stateAtLocalPoint(point) == VoxelState::Material;
}

const MyMath::Matrix4& VoxelShape::transform() const
{
    return m_transform;
}

void VoxelShape::setTransform(const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());
    m_transform = transform;
}

void VoxelShape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

void VoxelShape::detach()
{
    assert(m_data);

    if (!m_data.unique())
    {
        m_data = std::make_shared<SharedData>(*m_data);
    }
}

}