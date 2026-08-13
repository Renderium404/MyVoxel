#ifndef MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H
#define MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyVoxel/Core/VoxelGrid.h"
#include "CellMeshingState.h"

namespace MyVoxel
{
namespace Meshing
{

// 保存一次有限TSDF区域Surface Nets提取过程中的连续临时状态。
//
// 工作区只保存采样点符号、单元符号掩码和活动单元状态，不再拥有独立VolumeField或梯度缓存。
// 同一工作区可以重复用于采样范围完全一致的VoxelShape TSDF网格提取。
class VolumeMeshingWorkspace
{
public:
    // 根据最高层TSDF采样范围创建对应的采样与单元工作区。
    explicit VolumeMeshingWorkspace(const VoxelCellRange& sampleRange);

    /// 状态判断

    // 判断采样范围、单元范围和连续存储是否保持一致。
    bool isValid() const;
    // 判断工作区中是否没有任何活动表面单元。
    bool isEmpty() const;
    // 判断全部采样点符号是否已经完成预计算。
    bool hasCompleteSampleSigns() const;
    // 判断全部网格单元符号掩码是否已经完成预计算。
    bool hasCompleteCellSignMasks() const;

    /// 工作区属性

    // 返回工作区覆盖的最高层TSDF采样点范围。
    const VoxelCellRange& sampleRange() const;
    // 返回由相邻八个采样点定义的Surface Nets网格单元范围。
    const VoxelCellRange& cellRange() const;
    // 返回工作区包含的采样点数量。
    std::size_t sampleCount() const;
    // 返回工作区包含的网格单元数量。
    std::size_t cellCount() const;
    // 返回当前已经写入的活动表面单元数量。
    std::size_t activeCellCount() const;
    // 返回当前已经预计算的采样点符号数量。
    std::size_t precomputedSampleSignCount() const;
    // 返回当前已经预计算的网格单元符号掩码数量。
    std::size_t precomputedCellSignMaskCount() const;
    // 保留旧统计接口；Uniform Surface Nets不再缓存采样梯度，因此始终返回零。
    std::size_t computedGradientCount() const;
    // 判断指定索引是否位于采样点范围中。
    bool containsSample(const VoxelCellIndex& index) const;
    // 判断指定索引是否位于网格单元范围中。
    bool containsCell(const VoxelCellIndex& index) const;

    /// 采样符号访问

    // 写入指定采样点相对于当前等值面的内外符号。
    void setSampleInside(const VoxelCellIndex& index, bool inside);
    // 返回指定采样点是否位于当前等值面内部。
    bool sampleInside(const VoxelCellIndex& index) const;

    /// 单元符号访问

    // 写入指定网格单元的八位原始符号掩码。
    void setCellSignMask(const VoxelCellIndex& index, std::uint8_t signMask);
    // 返回指定网格单元的八位原始符号掩码。
    std::uint8_t cellSignMask(const VoxelCellIndex& index) const;

    /// 单元状态访问

    // 返回指定网格单元的只读状态，范围外或非活动单元返回空指针。
    const CellMeshingState* find(const VoxelCellIndex& index) const;
    // 返回指定网格单元的可写状态，范围外或非活动单元返回空指针。
    CellMeshingState* find(const VoxelCellIndex& index);
    // 写入指定活动表面单元状态。
    void setState(const VoxelCellIndex& index, const CellMeshingState& state);

    /// 生命周期

    // 清空全部预计算数据和活动单元状态，但保留连续数组容量。
    void clear();

private:
    // 返回指定采样点对应的连续数组位置。
    std::size_t sampleLinearIndex(const VoxelCellIndex& index) const;
    // 返回指定网格单元对应的连续数组位置。
    std::size_t cellLinearIndex(const VoxelCellIndex& index) const;

private:
    VoxelCellRange m_sampleRange; // 工作区覆盖的最高层TSDF采样点范围。
    VoxelCellRange m_cellRange; // 工作区覆盖的Surface Nets网格单元范围。
    std::size_t m_sampleCountX; // X方向采样点数量。
    std::size_t m_sampleCountY; // Y方向采样点数量。
    std::size_t m_sampleCountZ; // Z方向采样点数量。
    std::size_t m_cellCountX; // X方向网格单元数量。
    std::size_t m_cellCountY; // Y方向网格单元数量。
    std::size_t m_cellCountZ; // Z方向网格单元数量。
    std::size_t m_activeCellCount; // 当前活动表面单元数量。
    std::size_t m_precomputedSampleSignCount; // 当前已经预计算的采样点符号数量。
    std::size_t m_precomputedCellSignMaskCount; // 当前已经预计算的单元符号掩码数量。
    std::vector<unsigned char> m_sampleInside; // 每个采样点的内外符号。
    std::vector<unsigned char> m_sampleInsideReady; // 每个采样点符号是否已经写入。
    std::vector<unsigned char> m_cellSignMasks; // 每个网格单元的八位原始符号掩码。
    std::vector<unsigned char> m_cellSignMaskReady; // 每个单元符号掩码是否已经写入。
    std::vector<CellMeshingState> m_states; // 每个Surface Nets单元的临时边组与顶点状态。
};

}
}

#endif // MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H