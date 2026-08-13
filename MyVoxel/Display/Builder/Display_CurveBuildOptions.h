#ifndef MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDOPTIONS_H
#define MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDOPTIONS_H

namespace MyVoxel
{

// 控制Curve和Wire转换为线显示资源时的离散精度。
struct Display_CurveBuildOptions
{
    Display_CurveBuildOptions()
        : fullCircleSegmentCount(64)
    {
    }

    // 判断当前曲线显示离散参数是否有效。
    bool isValid() const
    {
        return fullCircleSegmentCount >= 3;
    }

    unsigned int fullCircleSegmentCount; // 一个完整圆固定使用的线段数量，局部圆弧按角度比例分配且至少使用一个线段。
};

}

#endif // MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDOPTIONS_H