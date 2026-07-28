#ifndef MYVOXELVIEWER_VOXELDISPLAYINSTANCE_H
#define MYVOXELVIEWER_VOXELDISPLAYINSTANCE_H

#include "MyMath/Matrix4.h"
#include "Viewer/OpenGL/DisplayColor.h"

namespace MyVoxelViewer
{

// 表示一个GPU实例化显示用体素块。
struct VoxelDisplayInstance
{
    VoxelDisplayInstance();
    VoxelDisplayInstance(const MyMath::Matrix4& transformValue, const DisplayColor& colorValue);

    float transform[16]; // OpenGL列优先实例变换矩阵。
    DisplayColor color; // 体素块显示颜色。
};

}

#endif // MYVOXELVIEWER_VOXELDISPLAYINSTANCE_H