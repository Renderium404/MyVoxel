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

const VoxelLevel BaseVoxelLevel = 0; // 第0层为未细分的基础体素层级。
const std::size_t VoxelCornerCount = 8; // 一个立方体体素固定包含八个角点。

// 表示体素角点，数值0~7同时作为八叉子项固定索引，第0、1、2位分别控制X、Y、Z坐标。
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

// 逻辑状态：表示逻辑体素对外呈现的结构状态。
enum class VoxelState : std::uint8_t
{
    Empty = 0, // 当前逻辑区域统一位于截断距离场外侧。
    Material = 1, // 当前逻辑区域统一位于截断距离场内侧。
    Subdivided = 2 // 当前逻辑区域存在更细结构或显式距离样本。
};

// 物理状态：表示分化NodeBlock中一个直接子项的物理解释方式。
enum class VoxelNodeState : std::uint8_t
{
    Empty = 0, // 子项为非分化空侧区域，对应VoxelBlock按NodeBlock::nodeData.value解释。
    Material = 1, // 子项为非分化材料侧区域，对应VoxelBlock按NodeBlock::nodeData.value解释。
    Node = 2, // 子项继续分化，对应VoxelBlock按NodeBlock解释。
    MaskLeaf = 3 // 子项为叶节点，对应VoxelBlock按IndexBlock解释。
};

// 将节点物理状态转换为逻辑结构状态。
inline VoxelState voxelState(VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        return VoxelState::Empty;
    case VoxelNodeState::Material:
        return VoxelState::Material;
    case VoxelNodeState::Node:
    case VoxelNodeState::MaskLeaf:
        return VoxelState::Subdivided;
    }

    return VoxelState::Empty;
}

}

#endif // MYVOXEL_CORE_VOXELTYPES_H
