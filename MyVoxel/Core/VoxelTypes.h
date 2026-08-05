#ifndef MYVOXEL_CORE_VOXELTYPES_H
#define MYVOXEL_CORE_VOXELTYPES_H

#include <cstddef>
#include <cstdint>
#include <limits>

namespace MyVoxel
{

using VoxelLevel = std::uint8_t; // 体素层级类型。
using VoxelIndex = std::int32_t; // 体素索引类型。

const VoxelIndex InvalidVoxelIndex = (std::numeric_limits<VoxelIndex>::max)(); // 无效体素索引。

class VoxelCellIndex; // 三维整数体素索引，用于表示离散空间中的坐标。
class VoxelCellAddress; // 体素地址，由体素索引和体素层级组成。
class VoxelCellRange; // 同一层级中的体素索引范围。

class VoxelGrid; // 连接连续几何空间与离散体素索引空间。
class VoxelNode; // 八叉树节点。

const VoxelLevel BaseVoxelLevel = 0; // 第0层为未细分的基础体素层级。
const std::size_t VoxelCornerCount = 8; // 一个立方体体素固定包含八个角点。

// 表示体素角点，数值的第0、1、2位分别控制X、Y、Z坐标，0取最小值，1取最大值。
enum class VoxelCorner : std::uint8_t
{
    Minimum = 0, // 使用X、Y、Z三个方向的最小坐标。
    MaximumX = 1, // 使用X最大坐标，Y和Z最小坐标。
    MaximumY = 2, // 使用Y最大坐标，X和Z最小坐标。
    MaximumXY = 3, // 使用X和Y最大坐标，Z最小坐标。
    MaximumZ = 4, // 使用Z最大坐标，X和Y最小坐标。
    MaximumXZ = 5, // 使用X和Z最大坐标，Y最小坐标。
    MaximumYZ = 6, // 使用Y和Z最大坐标，X最小坐标。
    MaximumXYZ = 7 // 使用X、Y、Z三个方向的最大坐标。
};

// 逻辑状态：表示逻辑体素对外呈现的状态。
enum class VoxelState : std::uint8_t
{
    Empty = 0,              // 当前逻辑体素为空。
    Material = 1,           // 当前逻辑体素完全包含材料。
    Subdivided = 2          // 当前逻辑体素已经细分。
};

// 物理状态：表示VoxelNodeBlock中记录的直接子节点存储状态。
enum class VoxelNodeState : std::uint8_t
{
    Empty = 0,              // 子节点为空，不占用物理槽。
    Material = 1,           // 子节点完全包含材料，不占用物理槽。
    Branch = 2,             // 子节点使用VoxelNodeBlock物理槽。
    MaskLeaf = 3            // 子节点使用VoxelLeafBlock物理槽。
};
// 将节点存储状态转换为逻辑体素状态。
inline VoxelState voxelState(VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        return VoxelState::Empty;

    case VoxelNodeState::Material:
        return VoxelState::Material;

    case VoxelNodeState::Branch:
    case VoxelNodeState::MaskLeaf:
        return VoxelState::Subdivided;
    }
    return VoxelState::Empty;
}
}

#endif // MYVOXEL_CORE_VOXELTYPES_H