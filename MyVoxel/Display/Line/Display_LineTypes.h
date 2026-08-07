#ifndef MYVOXEL_DISPLAY_LINE_DISPLAY_LINETYPES_H
#define MYVOXEL_DISPLAY_LINE_DISPLAY_LINETYPES_H

#include <cstddef>
#include <cstdint>

#include "MyVoxel/Display/Base/Display_Color.h"

namespace MyVoxel
{

typedef std::uint64_t Display_LineObjectId;
typedef std::uint64_t Display_LinePartId;

// 标识线对象GPU缓冲的预期更新频率。
enum class Display_LineUsage
{
    Static,
    Dynamic
};

// 保存一个可直接上传到显示后端的线顶点。
//
// 数据布局固定为位置3和颜色4，共七个连续float。
// Display_LineResource使用GL_LINES语义，每两个连续顶点形成一个独立线段。
struct Display_LineVertex
{
    Display_LineVertex();
    Display_LineVertex(double xValue, double yValue, double zValue, const Display_Color& colorValue);

    // 判断位置和颜色是否全部有效。
    bool isValid() const;

    float x; // 局部空间位置X分量。
    float y; // 局部空间位置Y分量。
    float z; // 局部空间位置Z分量。
    float red; // 线性红色分量。
    float green; // 线性绿色分量。
    float blue; // 线性蓝色分量。
    float alpha; // 透明度分量。
};

}

#endif // MYVOXEL_DISPLAY_LINE_DISPLAY_LINETYPES_H
