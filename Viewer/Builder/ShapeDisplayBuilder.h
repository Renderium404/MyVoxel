#ifndef MYVOXELVIEWER_SHAPEDISPLAYBUILDER_H
#define MYVOXELVIEWER_SHAPEDISPLAYBUILDER_H

#include "MyVoxel/Shape/Shape.h"
#include "Viewer/OpenGL/DisplayColor.h"
#include "Viewer/OpenGL/DisplayMesh.h"

namespace MyVoxelViewer
{

// 将标准连续Shape转换为可视化三角网格。
class ShapeDisplayBuilder
{
public:
    // 创建指定标准Shape的显示网格，圆形边界使用radialSegmentCount段离散。
    static DisplayMesh build(const MyVoxel::Shape& shape, const DisplayColor& color, int radialSegmentCount = 64);

private:
    ShapeDisplayBuilder() = delete;
};

}

#endif // MYVOXELVIEWER_SHAPEDISPLAYBUILDER_H