#ifndef MYVOXEL_VOLUME_LEVELSETVOLUME_H
#define MYVOXEL_VOLUME_LEVELSETVOLUME_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Core/VoxelGrid.h"

namespace MyVoxel
{

// 保存最高层体素中心处采样的有限标量距离场。
//
// 当前标准实现使用连续数组保存指定有限范围内的全部采样值。
// 正值表示外部，负值表示内部，零值表示等值面。
class LevelSetVolume
{
public:
    // 使用指定网格、最高层采样范围和范围外背景值创建距离场。
    LevelSetVolume(const VoxelGrid& grid, const VoxelCellRange& sampleRange, float backgroundValue);

    /// 状态判断

    // 判断网格、采样范围和内部存储是否一致。
    bool isValid() const;

    // 判断当前距离场是否没有任何采样值。
    bool isEmpty() const;

    /// 距离场属性

    // 返回当前距离场使用的体素网格。
    const VoxelGrid& grid() const;

    // 返回当前距离场在最高层网格中的有限采样范围。
    const VoxelCellRange& sampleRange() const;

    // 返回采样范围以外使用的固定背景值。
    float backgroundValue() const;

    // 返回有限采样范围中的采样数量。
    std::size_t sampleCount() const;

    // 判断指定最高层索引是否位于有限采样范围中。
    bool contains(const VoxelCellIndex& index) const;

    /// 采样访问

    // 返回指定最高层体素中心处的距离值，范围外返回背景值。
    float value(const VoxelCellIndex& index) const;

    // 设置指定最高层体素中心处的距离值，索引必须位于有限采样范围中。
    void setValue(const VoxelCellIndex& index, float value);

    // 返回指定最高层索引对应的采样位置。
    MyMath::Vector3 samplePosition(const VoxelCellIndex& index) const;

private:
    // 返回指定范围内索引对应的连续数组位置。
    std::size_t linearIndex(const VoxelCellIndex& index) const;

private:
    VoxelGrid m_grid; // 当前距离场使用的固定体素网格。
    VoxelCellRange m_sampleRange; // 当前距离场保存的最高层有限采样范围。
    float m_backgroundValue; // 有限采样范围外使用的距离值。
    std::size_t m_countX; // X方向采样数量。
    std::size_t m_countY; // Y方向采样数量。
    std::size_t m_countZ; // Z方向采样数量。
    std::vector<float> m_values; // 按Z、Y、X顺序保存的连续采样值。
};

}

#endif // MYVOXEL_VOLUME_LEVELSETVOLUME_H