#ifndef MYVOXELVIEWER_VOXELSURFACEDISPLAYBUILDER_H
#define MYVOXELVIEWER_VOXELSURFACEDISPLAYBUILDER_H

#include "MyVoxel/Surface/VoxelSurfaceMesh.h"
#include "Viewer/OpenGL/DisplayMesh.h"

namespace MyVoxelViewer
{

// 将体素表面网格转换为统一显示网格。
class VoxelSurfaceDisplayBuilder
{
public:
    // 创建指定体素表面网格对应的统一显示网格。
    static DisplayMesh build(const MyVoxel::VoxelSurfaceMesh& mesh);

private:
    VoxelSurfaceDisplayBuilder() = delete;
};

}

#endif // MYVOXELVIEWER_VOXELSURFACEDISPLAYBUILDER_H