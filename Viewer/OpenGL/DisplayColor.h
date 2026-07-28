#ifndef MYVOXELVIEWER_DISPLAYCOLOR_H
#define MYVOXELVIEWER_DISPLAYCOLOR_H

namespace MyVoxelViewer
{

// 表示可视化对象的显示颜色。
struct DisplayColor
{
    DisplayColor() = default;
    DisplayColor(double redValue, double greenValue, double blueValue, double alphaValue = 1.0);

    float red = 1.0f; // 红色分量。
    float green = 1.0f; // 绿色分量。
    float blue = 1.0f; // 蓝色分量。
    float alpha = 1.0f; // 透明度分量。
};

}

#endif // MYVOXELVIEWER_DISPLAYCOLOR_H