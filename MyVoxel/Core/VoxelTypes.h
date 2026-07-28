#ifndef MYVOXEL_VOXEL_H
#define MYVOXEL_VOXEL_H

#include <cstdint>

namespace MyVoxel
{


using VoxelLevel = std::uint8_t;    //体素层级
using VoxelIndex = std::int32_t;     //方向索引
class VoxelCellIndex;               //体素索引，同层级网格下，体素索引唯一
class VoxelCellAddress;             //体素地址，由体素层级+体素索引决定，同精度下，地址唯一

const VoxelLevel BaseVoxelLevel = 0; // 默认体素层级为0。
const int VoxelCornerCount = 8; // 一个立方体单元固定包含八个角点。

// 表示体素角点，数值的XYZ位分别表示是否使用对应轴的最大坐标。
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

// 表示体素的材料状态。
enum class VoxelState : std::uint8_t
{
    Empty = 0, // 当前节点为空。
    Material = 1, // 当前节点不为空且未分割。
    Subdivided = 2 // 当前节点不为空且已经分割为八个子体素。
};

}

#endif // MYVOXEL_VOXEL_H