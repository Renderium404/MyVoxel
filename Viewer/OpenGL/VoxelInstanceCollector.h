#ifndef MYVOXELVIEWER_VOXELINSTANCECOLLECTOR_H
#define MYVOXELVIEWER_VOXELINSTANCECOLLECTOR_H

#include <vector>

#include "MyVoxel/Core/VoxelShape.h"
#include "Viewer/OpenGL/DisplayColor.h"
#include "Viewer/OpenGL/VoxelDisplayInstance.h"

namespace MyVoxelViewer
{

// 将体素形体中的材料节点转换为GPU实例显示数据。
class VoxelInstanceCollector
{
public:
    // 将指定形体中的材料节点追加到显示实例集合。
    static void append(std::vector<VoxelDisplayInstance>& instances, const MyVoxel::VoxelShape& shape, const DisplayColor& color);

    // 收集指定形体中的全部材料节点显示实例。
    static std::vector<VoxelDisplayInstance> collect(const MyVoxel::VoxelShape& shape, const DisplayColor& color);
};

}

#endif // MYVOXELVIEWER_VOXELINSTANCECOLLECTOR_H