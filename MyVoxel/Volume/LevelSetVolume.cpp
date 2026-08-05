#include "LevelSetVolume.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 将无符号64位数量转换为size_t。
std::size_t checkedSize(std::uint64_t value)
{
    const std::uint64_t maximum = static_cast<std::uint64_t>((std::numeric_limits<std::size_t>::max)());

    MYVOXEL_ASSERT_MESSAGE(value <= maximum, "LevelSetVolume sample dimension exceeds size_t range.");
    return static_cast<std::size_t>(value);
}

// 执行size_t乘法并检查溢出。
std::size_t checkedMultiply(std::size_t first, std::size_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    MYVOXEL_ASSERT_MESSAGE(first <= (std::numeric_limits<std::size_t>::max)() / second,
                           "LevelSetVolume sample count exceeds size_t range.");

    return first * second;
}

}

namespace MyVoxel
{

LevelSetVolume::LevelSetVolume(const VoxelGrid& grid, const VoxelCellRange& sampleRange, float backgroundValue)
    : m_grid(grid)
    , m_sampleRange(sampleRange)
    , m_backgroundValue(backgroundValue)
    , m_countX(checkedSize(sampleRange.countX()))
    , m_countY(checkedSize(sampleRange.countY()))
    , m_countZ(checkedSize(sampleRange.countZ()))
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "LevelSetVolume requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(sampleRange.isValid(), "LevelSetVolume requires a valid sample range.");
    MYVOXEL_ASSERT_MESSAGE(sampleRange.level == grid.maximumLevel(),
                           "LevelSetVolume samples must use the VoxelGrid maximum level.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(static_cast<double>(backgroundValue)),
                           "LevelSetVolume background value must be finite.");

    const std::size_t xyCount = checkedMultiply(m_countX, m_countY);
    const std::size_t sampleCountValue = checkedMultiply(xyCount, m_countZ);

    m_values.assign(sampleCountValue, backgroundValue);

    MYVOXEL_ASSERT_MESSAGE(isValid(), "LevelSetVolume construction produced invalid storage.");
}

/// 状态判断

bool LevelSetVolume::isValid() const
{
    if (!m_grid.isValid() || !m_sampleRange.isValid() || m_sampleRange.level != m_grid.maximumLevel())
    {
        return false;
    }

    if (!std::isfinite(static_cast<double>(m_backgroundValue)))
    {
        return false;
    }

    if (m_countX != checkedSize(m_sampleRange.countX()) ||
        m_countY != checkedSize(m_sampleRange.countY()) ||
        m_countZ != checkedSize(m_sampleRange.countZ()))
    {
        return false;
    }

    return m_values.size() == checkedMultiply(checkedMultiply(m_countX, m_countY), m_countZ);
}

bool LevelSetVolume::isEmpty() const
{
    return m_values.empty();
}

/// 距离场属性

const VoxelGrid& LevelSetVolume::grid() const
{
    return m_grid;
}

const VoxelCellRange& LevelSetVolume::sampleRange() const
{
    return m_sampleRange;
}

float LevelSetVolume::backgroundValue() const
{
    return m_backgroundValue;
}

std::size_t LevelSetVolume::sampleCount() const
{
    return m_values.size();
}

bool LevelSetVolume::contains(const VoxelCellIndex& index) const
{
    return m_sampleRange.contains(index);
}

/// 采样访问

float LevelSetVolume::value(const VoxelCellIndex& index) const
{
    if (!contains(index))
    {
        return m_backgroundValue;
    }

    return m_values[linearIndex(index)];
}

void LevelSetVolume::setValue(const VoxelCellIndex& index, float value)
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "LevelSetVolume target sample lies outside the finite sample range.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(static_cast<double>(value)), "LevelSetVolume sample value must be finite.");

    m_values[linearIndex(index)] = value;
}

MyMath::Vector3 LevelSetVolume::samplePosition(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "LevelSetVolume sample position lies outside the finite sample range.");
    return m_grid.cellCenter(VoxelCellAddress(index, m_sampleRange.level));
}

/// 内部辅助

std::size_t LevelSetVolume::linearIndex(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(contains(index), "LevelSetVolume linear index lies outside the finite sample range.");

    const std::int64_t localX =
        static_cast<std::int64_t>(index.x) -
        static_cast<std::int64_t>(m_sampleRange.minimum.x);

    const std::int64_t localY =
        static_cast<std::int64_t>(index.y) -
        static_cast<std::int64_t>(m_sampleRange.minimum.y);

    const std::int64_t localZ =
        static_cast<std::int64_t>(index.z) -
        static_cast<std::int64_t>(m_sampleRange.minimum.z);

    MYVOXEL_ASSERT_MESSAGE(localX >= 0 && localY >= 0 && localZ >= 0,
                           "LevelSetVolume local sample index must be non-negative.");

    return
        (static_cast<std::size_t>(localZ) * m_countY +
         static_cast<std::size_t>(localY)) *
            m_countX +
        static_cast<std::size_t>(localX);
}

}