#include "VolumeMeshingWorkspace.h"

#include <algorithm>
#include <cstdint>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Volume/VolumeFieldView.h"

namespace
{

// 将无符号64位数量转换为size_t。
std::size_t checkedSize(std::uint64_t value)
{
    const std::uint64_t maximum = static_cast<std::uint64_t>((std::numeric_limits<std::size_t>::max)());

    MYVOXEL_ASSERT_MESSAGE(value <= maximum, "VolumeMeshingWorkspace dimension exceeds size_t range.");
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
                           "VolumeMeshingWorkspace element count exceeds size_t range.");

    return first * second;
}

// 根据采样范围生成低一格的网格单元范围。
MyVoxel::VoxelCellRange makeCellRange(const MyVoxel::VoxelCellRange& sampleRange)
{
    MYVOXEL_ASSERT_MESSAGE(sampleRange.isValid(), "VolumeMeshingWorkspace requires a valid sample range.");
    MYVOXEL_ASSERT_MESSAGE(sampleRange.countX() >= 2 && sampleRange.countY() >= 2 && sampleRange.countZ() >= 2,
                           "VolumeMeshingWorkspace requires at least two samples in every direction.");

    const MyVoxel::VoxelCellIndex maximum(
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(sampleRange.maximum.x) - 1),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(sampleRange.maximum.y) - 1),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(sampleRange.maximum.z) - 1));

    return MyVoxel::VoxelCellRange(sampleRange.minimum, maximum, sampleRange.level);
}

// 返回带整数偏移的体素索引。
MyVoxel::VoxelCellIndex offsetIndex(const MyVoxel::VoxelCellIndex& index, int offsetX, int offsetY, int offsetZ)
{
    return MyVoxel::VoxelCellIndex(
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.x) + offsetX),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.y) + offsetY),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.z) + offsetZ));
}

}

namespace MyVoxel
{
namespace Meshing
{

VolumeMeshingWorkspace::VolumeMeshingWorkspace(const VoxelCellRange& sampleRange)
    : m_sampleRange(sampleRange)
    , m_cellRange(makeCellRange(sampleRange))
    , m_sampleCountX(checkedSize(m_sampleRange.countX()))
    , m_sampleCountY(checkedSize(m_sampleRange.countY()))
    , m_sampleCountZ(checkedSize(m_sampleRange.countZ()))
    , m_cellCountX(checkedSize(m_cellRange.countX()))
    , m_cellCountY(checkedSize(m_cellRange.countY()))
    , m_cellCountZ(checkedSize(m_cellRange.countZ()))
    , m_activeCellCount(0)
    , m_precomputedSampleSignCount(0)
    , m_precomputedCellSignMaskCount(0)
    , m_computedGradientCount(0)
{
    const std::size_t sampleXYCount = checkedMultiply(m_sampleCountX, m_sampleCountY);
    const std::size_t sampleCountValue = checkedMultiply(sampleXYCount, m_sampleCountZ);
    const std::size_t cellXYCount = checkedMultiply(m_cellCountX, m_cellCountY);
    const std::size_t cellCountValue = checkedMultiply(cellXYCount, m_cellCountZ);

    m_sampleInside.resize(sampleCountValue);
    m_sampleInsideReady.assign(sampleCountValue, static_cast<unsigned char>(0));

    m_cellSignMasks.resize(cellCountValue);
    m_cellSignMaskReady.assign(cellCountValue, static_cast<unsigned char>(0));

    m_states.resize(cellCountValue);

    m_gradientX.resize(sampleCountValue);
    m_gradientY.resize(sampleCountValue);
    m_gradientZ.resize(sampleCountValue);
    m_gradientReady.assign(sampleCountValue, static_cast<unsigned char>(0));

    MYVOXEL_ASSERT_MESSAGE(isValid(), "VolumeMeshingWorkspace construction produced invalid storage.");
}

/// 状态判断

bool VolumeMeshingWorkspace::isValid() const
{
    if (!m_sampleRange.isValid() || !m_cellRange.isValid())
    {
        return false;
    }

    if (m_sampleRange.level != m_cellRange.level)
    {
        return false;
    }

    if (m_sampleCountX == 0 || m_sampleCountY == 0 || m_sampleCountZ == 0 ||
        m_cellCountX == 0 || m_cellCountY == 0 || m_cellCountZ == 0)
    {
        return false;
    }

    if (m_sampleCountX > (std::numeric_limits<std::size_t>::max)() / m_sampleCountY)
    {
        return false;
    }

    const std::size_t sampleXYCount = m_sampleCountX * m_sampleCountY;

    if (sampleXYCount > (std::numeric_limits<std::size_t>::max)() / m_sampleCountZ)
    {
        return false;
    }

    if (m_cellCountX > (std::numeric_limits<std::size_t>::max)() / m_cellCountY)
    {
        return false;
    }

    const std::size_t cellXYCount = m_cellCountX * m_cellCountY;

    if (cellXYCount > (std::numeric_limits<std::size_t>::max)() / m_cellCountZ)
    {
        return false;
    }

    const std::size_t expectedSampleCount = sampleXYCount * m_sampleCountZ;
    const std::size_t expectedCellCount = cellXYCount * m_cellCountZ;

    return
        m_sampleInside.size() == expectedSampleCount &&
        m_sampleInsideReady.size() == expectedSampleCount &&
        m_cellSignMasks.size() == expectedCellCount &&
        m_cellSignMaskReady.size() == expectedCellCount &&
        m_states.size() == expectedCellCount &&
        m_gradientX.size() == expectedSampleCount &&
        m_gradientY.size() == expectedSampleCount &&
        m_gradientZ.size() == expectedSampleCount &&
        m_gradientReady.size() == expectedSampleCount &&
        m_activeCellCount <= expectedCellCount &&
        m_precomputedSampleSignCount <= expectedSampleCount &&
        m_precomputedCellSignMaskCount <= expectedCellCount &&
        m_computedGradientCount <= expectedSampleCount;
}

bool VolumeMeshingWorkspace::isEmpty() const
{
    return m_activeCellCount == 0;
}

bool VolumeMeshingWorkspace::hasCompleteSampleSigns() const
{
    return m_precomputedSampleSignCount == sampleCount();
}

bool VolumeMeshingWorkspace::hasCompleteCellSignMasks() const
{
    return m_precomputedCellSignMaskCount == cellCount();
}

/// 工作区属性

const VoxelCellRange& VolumeMeshingWorkspace::sampleRange() const
{
    return m_sampleRange;
}

const VoxelCellRange& VolumeMeshingWorkspace::cellRange() const
{
    return m_cellRange;
}

std::size_t VolumeMeshingWorkspace::sampleCount() const
{
    return m_sampleInside.size();
}

std::size_t VolumeMeshingWorkspace::cellCount() const
{
    return m_states.size();
}

std::size_t VolumeMeshingWorkspace::activeCellCount() const
{
    return m_activeCellCount;
}

std::size_t VolumeMeshingWorkspace::precomputedSampleSignCount() const
{
    return m_precomputedSampleSignCount;
}

std::size_t VolumeMeshingWorkspace::precomputedCellSignMaskCount() const
{
    return m_precomputedCellSignMaskCount;
}

std::size_t VolumeMeshingWorkspace::computedGradientCount() const
{
    return m_computedGradientCount;
}

bool VolumeMeshingWorkspace::containsSample(const VoxelCellIndex& index) const
{
    return m_sampleRange.contains(index);
}

bool VolumeMeshingWorkspace::containsCell(const VoxelCellIndex& index) const
{
    return m_cellRange.contains(index);
}

/// 采样符号访问

void VolumeMeshingWorkspace::setSampleInside(const VoxelCellIndex& index, bool inside)
{
    MYVOXEL_ASSERT_MESSAGE(containsSample(index), "VolumeMeshingWorkspace sample sign index lies outside the sample range.");

    const std::size_t linearIndexValue = sampleLinearIndex(index);

    if (m_sampleInsideReady[linearIndexValue] == 0)
    {
        m_sampleInsideReady[linearIndexValue] = static_cast<unsigned char>(1);
        ++m_precomputedSampleSignCount;
    }

    m_sampleInside[linearIndexValue] = inside ? static_cast<unsigned char>(1) : static_cast<unsigned char>(0);
}

bool VolumeMeshingWorkspace::sampleInside(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(containsSample(index), "VolumeMeshingWorkspace sample sign index lies outside the sample range.");

    const std::size_t linearIndexValue = sampleLinearIndex(index);

    MYVOXEL_ASSERT_MESSAGE(m_sampleInsideReady[linearIndexValue] != 0,
                           "VolumeMeshingWorkspace sample sign has not been precomputed.");

    return m_sampleInside[linearIndexValue] != 0;
}

/// 单元符号访问

void VolumeMeshingWorkspace::setCellSignMask(const VoxelCellIndex& index, std::uint8_t signMask)
{
    MYVOXEL_ASSERT_MESSAGE(containsCell(index), "VolumeMeshingWorkspace cell sign index lies outside the cell range.");

    const std::size_t linearIndexValue = cellLinearIndex(index);

    if (m_cellSignMaskReady[linearIndexValue] == 0)
    {
        m_cellSignMaskReady[linearIndexValue] = static_cast<unsigned char>(1);
        ++m_precomputedCellSignMaskCount;
    }

    m_cellSignMasks[linearIndexValue] = static_cast<unsigned char>(signMask);
}

std::uint8_t VolumeMeshingWorkspace::cellSignMask(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(containsCell(index), "VolumeMeshingWorkspace cell sign index lies outside the cell range.");

    const std::size_t linearIndexValue = cellLinearIndex(index);

    MYVOXEL_ASSERT_MESSAGE(m_cellSignMaskReady[linearIndexValue] != 0,
                           "VolumeMeshingWorkspace cell sign mask has not been precomputed.");

    return static_cast<std::uint8_t>(m_cellSignMasks[linearIndexValue]);
}

/// 单元状态访问

const CellMeshingState* VolumeMeshingWorkspace::find(const VoxelCellIndex& index) const
{
    if (!containsCell(index))
    {
        return nullptr;
    }

    const CellMeshingState& state = m_states[cellLinearIndex(index)];
    return state.edgeGroupCount > 0 ? &state : nullptr;
}

CellMeshingState* VolumeMeshingWorkspace::find(const VoxelCellIndex& index)
{
    if (!containsCell(index))
    {
        return nullptr;
    }

    CellMeshingState& state = m_states[cellLinearIndex(index)];
    return state.edgeGroupCount > 0 ? &state : nullptr;
}

void VolumeMeshingWorkspace::setState(const VoxelCellIndex& index, const CellMeshingState& state)
{
    MYVOXEL_ASSERT_MESSAGE(containsCell(index), "VolumeMeshingWorkspace target cell lies outside the workspace range.");
    MYVOXEL_ASSERT_MESSAGE(state.edgeGroupCount > 0, "VolumeMeshingWorkspace can only store active surface cells.");

    CellMeshingState& target = m_states[cellLinearIndex(index)];

    if (target.edgeGroupCount == 0)
    {
        ++m_activeCellCount;
    }

    target = state;
}

/// 梯度访问

MyMath::Vector3 VolumeMeshingWorkspace::gradient(const VolumeFieldView& view, const VoxelCellIndex& index)
{
    MYVOXEL_ASSERT_MESSAGE(view.isValid() && view.isCurrent(), "VolumeMeshingWorkspace gradient requires a valid current VolumeFieldView.");
    MYVOXEL_ASSERT_MESSAGE(m_sampleRange.level == view.sampleLevel(), "VolumeMeshingWorkspace gradient range must use the VolumeField sample level.");
    MYVOXEL_ASSERT_MESSAGE(containsSample(index), "VolumeMeshingWorkspace gradient index lies outside the sample range.");

    const std::size_t linearIndexValue = sampleLinearIndex(index);

    if (m_gradientReady[linearIndexValue] == 0)
    {
        m_gradientX[linearIndexValue] =
            static_cast<double>(view.value(offsetIndex(index, 1, 0, 0))) -
            static_cast<double>(view.value(offsetIndex(index, -1, 0, 0)));

        m_gradientY[linearIndexValue] =
            static_cast<double>(view.value(offsetIndex(index, 0, 1, 0))) -
            static_cast<double>(view.value(offsetIndex(index, 0, -1, 0)));

        m_gradientZ[linearIndexValue] =
            static_cast<double>(view.value(offsetIndex(index, 0, 0, 1))) -
            static_cast<double>(view.value(offsetIndex(index, 0, 0, -1)));

        m_gradientReady[linearIndexValue] = static_cast<unsigned char>(1);
        ++m_computedGradientCount;
    }

    return MyMath::Vector3(
        m_gradientX[linearIndexValue],
        m_gradientY[linearIndexValue],
        m_gradientZ[linearIndexValue]);
}

/// 生命周期

void VolumeMeshingWorkspace::clear()
{
    std::fill(m_sampleInsideReady.begin(), m_sampleInsideReady.end(), static_cast<unsigned char>(0));
    std::fill(m_cellSignMaskReady.begin(), m_cellSignMaskReady.end(), static_cast<unsigned char>(0));
    std::fill(m_gradientReady.begin(), m_gradientReady.end(), static_cast<unsigned char>(0));

    for (std::size_t stateIndex = 0; stateIndex < m_states.size(); ++stateIndex)
    {
        m_states[stateIndex] = CellMeshingState();
    }

    m_activeCellCount = 0;
    m_precomputedSampleSignCount = 0;
    m_precomputedCellSignMaskCount = 0;
    m_computedGradientCount = 0;
}

/// 内部辅助

std::size_t VolumeMeshingWorkspace::sampleLinearIndex(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(containsSample(index), "VolumeMeshingWorkspace sample index lies outside the sample range.");

    const std::int64_t localX = static_cast<std::int64_t>(index.x) - static_cast<std::int64_t>(m_sampleRange.minimum.x);
    const std::int64_t localY = static_cast<std::int64_t>(index.y) - static_cast<std::int64_t>(m_sampleRange.minimum.y);
    const std::int64_t localZ = static_cast<std::int64_t>(index.z) - static_cast<std::int64_t>(m_sampleRange.minimum.z);

    MYVOXEL_ASSERT_MESSAGE(localX >= 0 && localY >= 0 && localZ >= 0,
                           "VolumeMeshingWorkspace local sample index must be non-negative.");

    return (static_cast<std::size_t>(localZ) * m_sampleCountY + static_cast<std::size_t>(localY)) * m_sampleCountX +
           static_cast<std::size_t>(localX);
}

std::size_t VolumeMeshingWorkspace::cellLinearIndex(const VoxelCellIndex& index) const
{
    MYVOXEL_ASSERT_MESSAGE(containsCell(index), "VolumeMeshingWorkspace cell index lies outside the cell range.");

    const std::int64_t localX = static_cast<std::int64_t>(index.x) - static_cast<std::int64_t>(m_cellRange.minimum.x);
    const std::int64_t localY = static_cast<std::int64_t>(index.y) - static_cast<std::int64_t>(m_cellRange.minimum.y);
    const std::int64_t localZ = static_cast<std::int64_t>(index.z) - static_cast<std::int64_t>(m_cellRange.minimum.z);

    MYVOXEL_ASSERT_MESSAGE(localX >= 0 && localY >= 0 && localZ >= 0,
                           "VolumeMeshingWorkspace local cell index must be non-negative.");

    return (static_cast<std::size_t>(localZ) * m_cellCountY + static_cast<std::size_t>(localY)) * m_cellCountX +
           static_cast<std::size_t>(localX);
}

}
}