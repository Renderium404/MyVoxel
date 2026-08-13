#include "VoxelTsdfAccessor.h"

#include <cstdint>
#include <limits>

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 返回带整数偏移的最高层采样索引，并检查VoxelIndex数值范围。
MyVoxel::VoxelCellIndex offsetIndex(const MyVoxel::VoxelCellIndex& index, int offsetX, int offsetY, int offsetZ)
{
    const std::int64_t x = static_cast<std::int64_t>(index.x) + static_cast<std::int64_t>(offsetX);
    const std::int64_t y = static_cast<std::int64_t>(index.y) + static_cast<std::int64_t>(offsetY);
    const std::int64_t z = static_cast<std::int64_t>(index.z) + static_cast<std::int64_t>(offsetZ);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(x >= minimum && x <= maximum && y >= minimum && y <= maximum && z >= minimum && z <= maximum,
                           "VoxelTsdfAccessor neighbor sample exceeds VoxelIndex range.");
    return MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y), static_cast<MyVoxel::VoxelIndex>(z));
}

}

namespace MyVoxel
{
namespace Meshing
{

VoxelTsdfAccessor::VoxelTsdfAccessor(const VoxelShape& shape)
    : m_shape(&shape)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "VoxelTsdfAccessor requires a valid VoxelShape.");
}

/// 状态与绑定对象

bool VoxelTsdfAccessor::isValid() const
{
    return m_shape && m_shape->isValid();
}

const VoxelShape& VoxelTsdfAccessor::shape() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid VoxelTsdfAccessor.");
    return *m_shape;
}

const VoxelGrid& VoxelTsdfAccessor::grid() const
{
    return shape().grid();
}

VoxelLevel VoxelTsdfAccessor::sampleLevel() const
{
    return grid().maximumLevel();
}

double VoxelTsdfAccessor::sampleSpacing() const
{
    return grid().minimumCellEdgeLength();
}

float VoxelTsdfAccessor::backgroundDistance() const
{
    return shape().backgroundDistance();
}

bool VoxelTsdfAccessor::supportsSampleAddress(const VoxelCellAddress& address) const
{
    return isValid() && address.level == sampleLevel();
}

/// TSDF采样

float VoxelTsdfAccessor::value(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot read an invalid VoxelTsdfAccessor.");
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(address), "VoxelTsdfAccessor only supports highest-level TSDF sample addresses.");
    return shape().distance(address);
}

float VoxelTsdfAccessor::value(const VoxelCellIndex& index) const
{
    return value(VoxelCellAddress(index, sampleLevel()));
}

MyMath::Vector3 VoxelTsdfAccessor::samplePosition(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query position from an invalid VoxelTsdfAccessor.");
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(address), "VoxelTsdfAccessor only supports highest-level TSDF sample addresses.");
    return grid().cellCenter(address);
}

MyMath::Vector3 VoxelTsdfAccessor::samplePosition(const VoxelCellIndex& index) const
{
    return samplePosition(VoxelCellAddress(index, sampleLevel()));
}

/// 一阶微分

MyMath::Vector3 VoxelTsdfAccessor::gradient(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot compute gradient from an invalid VoxelTsdfAccessor.");
    const double spacing = sampleSpacing();
    MYVOXEL_ASSERT_MESSAGE(spacing > 0.0, "VoxelTsdfAccessor sample spacing must be positive.");
    const double inverseDoubleSpacing = 0.5 / spacing; // 二阶中心差分统一使用1/(2h)尺度。

    const double gradientX = (static_cast<double>(value(offsetIndex(index, 1, 0, 0))) - static_cast<double>(value(offsetIndex(index, -1, 0, 0)))) * inverseDoubleSpacing;
    const double gradientY = (static_cast<double>(value(offsetIndex(index, 0, 1, 0))) - static_cast<double>(value(offsetIndex(index, 0, -1, 0)))) * inverseDoubleSpacing;
    const double gradientZ = (static_cast<double>(value(offsetIndex(index, 0, 0, 1))) - static_cast<double>(value(offsetIndex(index, 0, 0, -1)))) * inverseDoubleSpacing;
    return MyMath::Vector3(gradientX, gradientY, gradientZ);
}

}
}