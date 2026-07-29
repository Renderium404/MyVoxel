#include "VoxelShape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelShape::Data::Data(const VoxelGrid& gridValue)
    : grid(gridValue)
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "VoxelShape requires a valid VoxelGrid.");
}

VoxelShape::Data::Data(const Data& other)
    : grid(other.grid)
    , forest(other.forest)
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Copied VoxelShape data must contain a valid VoxelGrid.");
}

VoxelShape::VoxelShape()
    : m_data(Foundation::makeRef<Data>(VoxelGrid(1.0, BaseVoxelLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(const VoxelGrid& grid)
    : m_data(Foundation::makeRef<Data>(grid))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(double baseVoxelEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<Data>(VoxelGrid(baseVoxelEdgeLength, maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(const MyMath::Vector3& origin, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<Data>(VoxelGrid(origin, baseVoxelEdgeLength, maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
}

/// 体素空间

const VoxelGrid& VoxelShape::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->grid;
}

/// 体素数据

const VoxelForest& VoxelShape::forest() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->forest;
}

VoxelForest& VoxelShape::editForest()
{
    detach();
    return m_data->forest;
}

bool VoxelShape::isEmpty() const
{
    return forest().isEmpty();
}

std::size_t VoxelShape::rootCount() const
{
    return forest().rootCount();
}

void VoxelShape::clear()
{
    if (isEmpty())
    {
        return;
    }

    detach();
    m_data->forest.clear();
}

bool VoxelShape::sharesDataWith(const VoxelShape& other) const
{
    return m_data.get() == other.m_data.get();
}

/// 空间变换

const MyMath::Matrix4& VoxelShape::transform() const
{
    return m_transform;
}

void VoxelShape::setTransform(const MyMath::Matrix4& transform)
{
    MYVOXEL_ASSERT_MESSAGE(transform.isAffine(), "VoxelShape transform must be affine.");
    m_transform = transform;
}

void VoxelShape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

void VoxelShape::detach()
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");

    if (m_data->referenceCount() > 1)
    {
        m_data = Foundation::makeRef<Data>(*m_data);
    }
}

}