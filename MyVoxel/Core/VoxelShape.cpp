#include "VoxelShape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const MyVoxel::VoxelLevel MinimumVolumeFieldSampleLevel = static_cast<MyVoxel::VoxelLevel>(2); // 一个64样本VolumeBlock覆盖连续两级细分，距离场采样层级至少为第2层。

// 判断指定体素网格是否能够建立两级展开的64样本距离场。
bool gridSupportsVolumeField(const MyVoxel::VoxelGrid& grid)
{
    return grid.isValid() && grid.maximumLevel() >= MinimumVolumeFieldSampleLevel;
}

// 为支持距离场的体素网格创建与最高层对齐的全失效距离场。
std::unique_ptr<MyVoxel::VolumeField> createVolumeField(const MyVoxel::VoxelGrid& grid)
{
    if (!gridSupportsVolumeField(grid))
    {
        return std::unique_ptr<MyVoxel::VolumeField>();
    }

    return std::unique_ptr<MyVoxel::VolumeField>(new MyVoxel::VolumeField(grid.maximumLevel()));
}

}

namespace MyVoxel
{

VoxelShape::SharedData::SharedData(const VoxelGrid& gridValue)
    : grid(gridValue)
    , forest()
    , volumeField()
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "VoxelShape requires a valid VoxelGrid.");
    volumeField = createVolumeField(grid);
}

VoxelShape::SharedData::SharedData(const SharedData& other)
    : grid(other.grid)
    , forest(other.forest)
    , volumeField()
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Copied VoxelShape data must contain a valid VoxelGrid.");
    volumeField = createVolumeField(grid);
}

VoxelShape::VoxelShape()
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(1.0, BaseVoxelLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(const VoxelGrid& grid)
    : m_data(Foundation::makeRef<SharedData>(grid))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(baseCellEdgeLength, maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(origin, baseCellEdgeLength, maximumLevel)))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

/// 状态判断

bool VoxelShape::isValid() const
{
    if (!m_data || !m_data->grid.isValid() || !m_transform.isAffine() || !m_transform.isInvertible())
    {
        return false;
    }

    const bool fieldSupported = gridSupportsVolumeField(m_data->grid);

    if (fieldSupported != static_cast<bool>(m_data->volumeField))
    {
        return false;
    }

    return !m_data->volumeField ||
           (m_data->volumeField->isValid() && m_data->volumeField->sampleLevel() == m_data->grid.maximumLevel());
}

bool VoxelShape::isEmpty() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    return m_data->forest.isEmpty();
}

bool VoxelShape::isDataShared() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->referenceCount() > 1;
}

bool VoxelShape::sharesDataWith(const VoxelShape& other) const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    MYVOXEL_ASSERT_MESSAGE(other.m_data, "Other VoxelShape shared data must not be null.");
    return m_data.get() == other.m_data.get();
}

/// 体素空间

const VoxelGrid& VoxelShape::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->grid;
}

bool VoxelShape::supportsAddress(const VoxelCellAddress& address) const
{
    return m_data && address.level <= m_data->grid.maximumLevel();
}

/// 体素状态

VoxelState VoxelShape::state(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_data->forest.state(address);
}

bool VoxelShape::hasNode(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_data->forest.hasNode(address);
}

/// 修改入口

VoxelShapeSession VoxelShape::session(VoxelLevel changeTrackingLevel)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot create a session for an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(changeTrackingLevel <= grid().maximumLevel(), "Voxel change tracking level exceeds the shape maximum level.");
    return VoxelShapeSession(*this, changeTrackingLevel);
}

/// 体素数据

const VoxelForest& VoxelShape::forest() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->forest;
}

std::size_t VoxelShape::rootCount() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    return m_data->forest.rootCount();
}

/// 距离场资源

bool VoxelShape::supportsVolumeField() const
{
    return m_data && gridSupportsVolumeField(m_data->grid);
}

const VolumeField* VoxelShape::volumeField() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query the distance field of an invalid VoxelShape.");
    return m_data->volumeField.get();
}

VolumeField* VoxelShape::editVolumeField()
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot edit the distance field of an invalid VoxelShape.");

    if (!supportsVolumeField())
    {
        return nullptr;
    }

    detach();

    MYVOXEL_ASSERT_MESSAGE(m_data->volumeField.get() != nullptr, "Supported VoxelShape must contain a VolumeField resource.");
    return m_data->volumeField.get();
}

/// 空间变换

const MyMath::Matrix4& VoxelShape::transform() const
{
    return m_transform;
}

void VoxelShape::setTransform(const MyMath::Matrix4& transform)
{
    MYVOXEL_ASSERT_MESSAGE(transform.isAffine(), "VoxelShape transform must be affine.");
    MYVOXEL_ASSERT_MESSAGE(transform.isInvertible(), "VoxelShape transform must be invertible.");
    m_transform = transform;
}

void VoxelShape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

/// 内部辅助

void VoxelShape::detach()
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");

    if (m_data->referenceCount() <= 1)
    {
        return;
    }

    m_data = Foundation::makeRef<SharedData>(*m_data);

    MYVOXEL_ASSERT_MESSAGE(m_data->referenceCount() == 1, "Detached VoxelShape data must be exclusively owned.");
    MYVOXEL_ASSERT_MESSAGE(!m_data->volumeField || m_data->volumeField->isCompletelyDirty(),
                           "Detached VoxelShape distance field must require a complete rebuild.");
}

void VoxelShape::invalidateVolumeField()
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    MYVOXEL_ASSERT_MESSAGE(m_data->referenceCount() == 1, "VoxelShape distance invalidation requires exclusively owned shared data.");

    if (m_data->volumeField)
    {
        m_data->volumeField->markAllDirty();
    }
}

}