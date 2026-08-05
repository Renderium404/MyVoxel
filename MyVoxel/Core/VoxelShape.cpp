#include "VoxelShape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelShape::SharedData::SharedData(
    const VoxelGrid& gridValue)
    : grid(gridValue)
{
    MYVOXEL_ASSERT_MESSAGE(
        grid.isValid(),
        "VoxelShape requires a valid VoxelGrid.");
}

VoxelShape::SharedData::SharedData(
    const SharedData& other)
    : grid(other.grid)
    , forest(other.forest)
{
    MYVOXEL_ASSERT_MESSAGE(
        grid.isValid(),
        "Copied VoxelShape data must contain a valid VoxelGrid.");
}

VoxelShape::VoxelShape()
    : m_data(
          Foundation::makeRef<SharedData>(
              VoxelGrid(1.0, BaseVoxelLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(const VoxelGrid& grid)
    : m_data(Foundation::makeRef<SharedData>(grid))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(
    double baseCellEdgeLength,
    VoxelLevel maximumLevel)
    : m_data(
          Foundation::makeRef<SharedData>(
              VoxelGrid(
                  baseCellEdgeLength,
                  maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(
    const MyMath::Vector3& origin,
    double baseCellEdgeLength,
    VoxelLevel maximumLevel)
    : m_data(
          Foundation::makeRef<SharedData>(
              VoxelGrid(
                  origin,
                  baseCellEdgeLength,
                  maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "VoxelShape parameters must define a valid voxel shape.");
}

/// 状态判断

bool VoxelShape::isValid() const
{
    return m_data &&
           m_data->grid.isValid() &&
           m_transform.isAffine() &&
           m_transform.isInvertible();
}

bool VoxelShape::isEmpty() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "Cannot query an invalid VoxelShape.");

    return m_data->forest.isEmpty();
}

bool VoxelShape::isDataShared() const
{
    MYVOXEL_ASSERT_MESSAGE(
        m_data,
        "VoxelShape shared data must not be null.");

    return m_data->referenceCount() > 1;
}

bool VoxelShape::sharesDataWith(
    const VoxelShape& other) const
{
    MYVOXEL_ASSERT_MESSAGE(
        m_data,
        "VoxelShape shared data must not be null.");

    MYVOXEL_ASSERT_MESSAGE(
        other.m_data,
        "Other VoxelShape shared data must not be null.");

    return m_data.get() == other.m_data.get();
}

/// 体素空间

const VoxelGrid& VoxelShape::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(
        m_data,
        "VoxelShape shared data must not be null.");

    return m_data->grid;
}

bool VoxelShape::supportsAddress(
    const VoxelCellAddress& address) const
{
    return m_data &&
           address.level <= m_data->grid.maximumLevel();
}

/// 体素状态

VoxelState VoxelShape::state(
    const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "Cannot query an invalid VoxelShape.");

    MYVOXEL_ASSERT_MESSAGE(
        supportsAddress(address),
        "Voxel address exceeds the shape maximum level.");

    return m_data->forest.state(address);
}

bool VoxelShape::hasNode(
    const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "Cannot query an invalid VoxelShape.");

    MYVOXEL_ASSERT_MESSAGE(
        supportsAddress(address),
        "Voxel address exceeds the shape maximum level.");

    return m_data->forest.hasNode(address);
}

/// 修改入口

VoxelShapeSession VoxelShape::session(
    VoxelLevel changeTrackingLevel)
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "Cannot create a session for an invalid VoxelShape.");

    MYVOXEL_ASSERT_MESSAGE(
        changeTrackingLevel <= grid().maximumLevel(),
        "Voxel change tracking level exceeds the shape maximum level.");

    return VoxelShapeSession(
        *this,
        changeTrackingLevel);
}

/// 体素数据

const VoxelForest& VoxelShape::forest() const
{
    MYVOXEL_ASSERT_MESSAGE(
        m_data,
        "VoxelShape shared data must not be null.");

    return m_data->forest;
}

std::size_t VoxelShape::rootCount() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValid(),
        "Cannot query an invalid VoxelShape.");

    return m_data->forest.rootCount();
}

/// 空间变换

const MyMath::Matrix4& VoxelShape::transform() const
{
    return m_transform;
}

void VoxelShape::setTransform(
    const MyMath::Matrix4& transform)
{
    MYVOXEL_ASSERT_MESSAGE(
        transform.isAffine(),
        "VoxelShape transform must be affine.");

    MYVOXEL_ASSERT_MESSAGE(
        transform.isInvertible(),
        "VoxelShape transform must be invertible.");

    m_transform = transform;
}

void VoxelShape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

/// 内部辅助

void VoxelShape::detach()
{
    MYVOXEL_ASSERT_MESSAGE(
        m_data,
        "VoxelShape shared data must not be null.");

    if (m_data->referenceCount() <= 1)
    {
        return;
    }

    m_data =
        Foundation::makeRef<SharedData>(*m_data);

    MYVOXEL_ASSERT_MESSAGE(
        m_data->referenceCount() == 1,
        "Detached VoxelShape data must be exclusively owned.");
}

}