#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_ORIENTATION_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_ORIENTATION_H

namespace MyVoxel
{

// 表示拓扑句柄相对于底层共享拓扑实体的使用方向。
enum class Topology_Orientation
{
    Forward = 0,
    Reversed = 1
};

// 返回与指定拓扑使用方向相反的方向。
Topology_Orientation oppositeOrientation(Topology_Orientation orientation);

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_ORIENTATION_H