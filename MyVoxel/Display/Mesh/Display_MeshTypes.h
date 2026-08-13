#ifndef MYVOXEL_DISPLAY_MESH_DISPLAY_MESHTYPES_H
#define MYVOXEL_DISPLAY_MESH_DISPLAY_MESHTYPES_H

#include <cstddef>
#include <cstdint>

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Base/Display_Normal.h"
namespace MyVoxel
{

// 保存一个可直接上传到显示后端的逐角点网格顶点。
//
// 数据布局固定为位置3、法线3和颜色4，共十个连续float。
// 每个三角形使用三个独立显示顶点，以支持逐三角形颜色和硬边法线。
struct Display_MeshVertex
{
    Display_MeshVertex();
    Display_MeshVertex(double xValue, double yValue, double zValue, const Display_Normal& normalValue, const Display_Color& colorValue);

    // 判断位置、法线和颜色是否构成有效显示顶点。
    bool isValid() const;

    float x; // 局部空间位置X分量。
    float y; // 局部空间位置Y分量。
    float z; // 局部空间位置Z分量。
    float normalX; // 局部空间单位法线X分量。
    float normalY; // 局部空间单位法线Y分量。
    float normalZ; // 局部空间单位法线Z分量。
    float red; // 线性红色分量。
    float green; // 线性绿色分量。
    float blue; // 线性蓝色分量。
    float alpha; // 透明度分量。
};

}

#endif // MYVOXEL_DISPLAY_MESH_DISPLAY_MESHTYPES_H
