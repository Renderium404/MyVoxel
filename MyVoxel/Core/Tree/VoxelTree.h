#ifndef MYVOXEL_VOXELTREE_H
#define MYVOXEL_VOXELTREE_H

#include "VoxelPackedRootTree.h"

namespace MyVoxel
{

// 生产根树使用独立Packed节点池并支持根级写时复制。
using VoxelTree = VoxelPackedRootTree;

}

#endif // MYVOXEL_VOXELTREE_H