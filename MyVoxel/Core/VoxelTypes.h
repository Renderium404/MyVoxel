#ifndef MYVOXEL_CORE_VOXELTYPES_H
#define MYVOXEL_CORE_VOXELTYPES_H

#include <cstddef>
#include <cstdint>
#include <limits>

namespace MyVoxel
{

using VoxelLevel = std::uint8_t;            // 体素层级类型。
using VoxelIndex = std::int32_t;            // 单个坐标方向的体素索引类型。
using VoxelNodeGroupIndex = std::uint32_t;  // 节点池中的节点组索引类型。


class VoxelCellIndex;       //体素索引 三维整数，用于表示离散三维空间中的坐标
class VoxelCellAddress;     //体素地址 = 体素索引 + 体素层级 用于表示指定层级的离散三维空间中的坐标
class VoxelCellRange;       //体素范围  本质为一个包围盒 记录最小地址和最大地址

class VoxelGrid;            //体素网格 用于几何坐标和体素索引间相互转化，用于连接 离散与连续 的空间变化
class VoxelNode;            //八叉树节点，持有子节点组的索引，需配置节点池使用
class VoxelNodeGroup;       //一组八个子节点
class VoxelNodePool;        //节点池，实际存储节点组

const VoxelLevel BaseVoxelLevel = 0; // 第0层为未细分的基础体素层级。
const std::size_t VoxelCornerCount = 8; // 一个立方体体素固定包含八个角点。
const VoxelNodeGroupIndex InvalidVoxelNodeGroupIndex = (std::numeric_limits<VoxelNodeGroupIndex>::max)(); // 表示节点未连接有效节点组。

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

// 表示体素节点当前保存的材料和细分状态。
enum class VoxelState : std::uint8_t
{
    Empty = 0, // 当前节点为空。
    Material = 1, // 当前节点包含材料且未继续细分。
    Subdivided = 2 // 当前节点已经细分为八个子节点。
};

}

#endif // MYVOXEL_CORE_VOXELTYPES_H