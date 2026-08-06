


#include "VolumeFieldView.h"

#include "MyVoxel/Core/Storage/VolumeBlock.h"
#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VolumeFieldView::VolumeFieldView(const VoxelShape& shape)
    : m_shape(&shape)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "VolumeFieldView requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(shape.supportsVolumeField(), "VolumeFieldView requires a Shape with maximum level at least 2.");
    MYVOXEL_ASSERT_MESSAGE(shape.volumeField() != nullptr, "VolumeFieldView requires an existing VolumeField resource.");
}

/// 状态判断

bool VolumeFieldView::isValid() const
{
    if (!m_shape || !m_shape->isValid() || !m_shape->supportsVolumeField()) return false;

    const VolumeField* volume = m_shape->volumeField();
    return volume && volume->isValid() && volume->sampleLevel() == m_shape->grid().maximumLevel();
}

bool VolumeFieldView::isCurrent() const
{
    return isValid() && field().isCurrent();
}

bool VolumeFieldView::isSampleReadable(const VoxelCellAddress& sampleAddress) const
{
    if (!isValid() || !supportsSampleAddress(sampleAddress)) return false;
    return !field().isBlockDirty(field().blockAddress(sampleAddress));
}

bool VolumeFieldView::hasStoredSample(const VoxelCellAddress& sampleAddress) const
{
    if (!isValid() || !supportsSampleAddress(sampleAddress)) return false;
    return field().containsBlock(field().blockAddress(sampleAddress));
}

/// 绑定资源

const VoxelShape& VolumeFieldView::shape() const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape != nullptr, "VolumeFieldView Shape pointer must not be null.");
    return *m_shape;
}

const VoxelGrid& VolumeFieldView::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query the grid of an invalid VolumeFieldView.");
    return shape().grid();
}

const VolumeField& VolumeFieldView::field() const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape != nullptr && m_shape->isValid() && m_shape->supportsVolumeField(),
                           "Cannot query the field of an invalid VolumeFieldView.");
    const VolumeField* volume = m_shape->volumeField();
    MYVOXEL_ASSERT_MESSAGE(volume != nullptr, "Supported VoxelShape must contain a VolumeField resource.");
    return *volume;
}

VoxelLevel VolumeFieldView::sampleLevel() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query the sample level of an invalid VolumeFieldView.");
    return field().sampleLevel();
}

bool VolumeFieldView::supportsSampleAddress(const VoxelCellAddress& sampleAddress) const
{
    return isValid() && field().supportsSampleAddress(sampleAddress);
}

/// 距离采样

float VolumeFieldView::value(const VoxelCellAddress& sampleAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot read an invalid VolumeFieldView.");
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(sampleAddress), "VolumeFieldView sample address must use the configured sample level.");

    const VolumeField& volume = field();
    const VoxelCellAddress blockAddress = volume.blockAddress(sampleAddress);
    MYVOXEL_ASSERT_MESSAGE(!volume.isBlockDirty(blockAddress), "VolumeFieldView cannot read a dirty distance block.");

    const VolumeBlock* block = volume.findBlock(blockAddress);
    return block ? volumeDistance(*block, volume.sampleIndex(sampleAddress)) : backgroundValue(sampleAddress);
}

float VolumeFieldView::value(const VoxelCellIndex& sampleIndex) const
{
    return value(VoxelCellAddress(sampleIndex, sampleLevel()));
}

MyMath::Vector3 VolumeFieldView::samplePosition(const VoxelCellAddress& sampleAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query a position from an invalid VolumeFieldView.");
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(sampleAddress), "VolumeFieldView sample address must use the configured sample level.");
    return grid().cellCenter(sampleAddress);
}

MyMath::Vector3 VolumeFieldView::samplePosition(const VoxelCellIndex& sampleIndex) const
{
    return samplePosition(VoxelCellAddress(sampleIndex, sampleLevel()));
}

/// 内部辅助

float VolumeFieldView::backgroundValue(const VoxelCellAddress& sampleAddress) const
{
    const VoxelState state = shape().state(sampleAddress);
    MYVOXEL_ASSERT_MESSAGE(state != VoxelState::Subdivided, "Maximum-level VolumeField sample cannot remain subdivided.");
    return state == VoxelState::Material ? field().interiorBackgroundDistance() : field().exteriorBackgroundDistance();
}

}