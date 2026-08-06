#ifndef MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H
#define MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "CellMeshingState.h"

namespace MyVoxel
{

class VolumeFieldView;

namespace Meshing
{

// 保存一次有限距离场区域网格提取过程中的连续临时状态。
//
// 工作区保存采样点符号、单元符号掩码、活动单元状态和懒计算梯度。
// 同一个工作区可以重复用于范围一致的VoxelShape距离场网格提取。
class VolumeMeshingWorkspace
{
public:
    // 根据距离场最高层采样范围创建对应的采样与单元工作区。
    explicit VolumeMeshingWorkspace(const VoxelCellRange& sampleRange);

    /// 状态判断

    // 判断采样范围、单元范围和连续存储是否一致。
    bool isValid() const;

    // 判断工作区中是否没有任何活动表面单元。
    bool isEmpty() const;

    // 判断全部采样点符号是否已经完成预计算。
    bool hasCompleteSampleSigns() const;

    // 判断全部网格单元符号掩码是否已经完成预计算。
    bool hasCompleteCellSignMasks() const;

    /// 工作区属性

    // 返回工作区覆盖的最高层采样点范围。
    const VoxelCellRange& sampleRange() const;

    // 返回工作区覆盖的网格单元范围。
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

    // 返回当前已经计算并缓存的采样梯度数量。
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

    /// 梯度访问

    // 返回指定采样点的中心差分梯度，第一次访问时从VolumeFieldView计算并缓存。
    MyMath::Vector3 gradient(const VolumeFieldView& view, const VoxelCellIndex& index);

    /// 生命周期

    // 清空全部预计算数据、单元状态和梯度缓存，但保留连续数组容量。
    void clear();

private:
    // 返回指定采样点对应的连续数组位置。
    std::size_t sampleLinearIndex(const VoxelCellIndex& index) const;

    // 返回指定网格单元对应的连续数组位置。
    std::size_t cellLinearIndex(const VoxelCellIndex& index) const;

private:
    VoxelCellRange m_sampleRange; // 工作区覆盖的最高层采样点范围。
    VoxelCellRange m_cellRange; // 工作区覆盖的最高层网格单元范围。

    std::size_t m_sampleCountX; // X方向采样点数量。
    std::size_t m_sampleCountY; // Y方向采样点数量。
    std::size_t m_sampleCountZ; // Z方向采样点数量。

    std::size_t m_cellCountX; // X方向网格单元数量。
    std::size_t m_cellCountY; // Y方向网格单元数量。
    std::size_t m_cellCountZ; // Z方向网格单元数量。

    std::size_t m_activeCellCount; // 当前活动表面单元数量。
    std::size_t m_precomputedSampleSignCount; // 当前已经预计算的采样点符号数量。
    std::size_t m_precomputedCellSignMaskCount; // 当前已经预计算的单元符号掩码数量。
    std::size_t m_computedGradientCount; // 当前已经计算的采样梯度数量。

    std::vector<unsigned char> m_sampleInside; // 每个采样点相对于当前等值面的内外符号。
    std::vector<unsigned char> m_sampleInsideReady; // 标记对应采样点符号是否已经写入。

    std::vector<unsigned char> m_cellSignMasks; // 每个网格单元的八位原始符号掩码。
    std::vector<unsigned char> m_cellSignMaskReady; // 标记对应单元符号掩码是否已经写入。

    std::vector<CellMeshingState> m_states; // 按Z、Y、X顺序排列的连续单元状态。

    std::vector<double> m_gradientX; // 与采样点一一对应的X方向梯度缓存。
    std::vector<double> m_gradientY; // 与采样点一一对应的Y方向梯度缓存。
    std::vector<double> m_gradientZ; // 与采样点一一对应的Z方向梯度缓存。
    std::vector<unsigned char> m_gradientReady; // 标记对应采样梯度是否已经计算。
};

}
}

#endif // MYVOXEL_MESHING_VOLUMEMESHINGWORKSPACE_H