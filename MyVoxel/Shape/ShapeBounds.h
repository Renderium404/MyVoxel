#ifndef MYVOXEL_SHAPEBOUNDS_H
#define MYVOXEL_SHAPEBOUNDS_H

namespace MyVoxel
{

// 表示Shape局部坐标系中的轴对齐包围盒。
struct ShapeBounds
{
    ShapeBounds() = default;
    ShapeBounds(double minimumXValue, double minimumYValue, double minimumZValue, double maximumXValue, double maximumYValue, double maximumZValue);

    // 检查包围盒是否由有限且有序的坐标范围组成。
    bool isValid() const;

    // 检查包围盒三个方向是否均具有正长度。
    bool hasVolume() const;

    double minimumX = 0.0; // X方向最小坐标。
    double minimumY = 0.0; // Y方向最小坐标。
    double minimumZ = 0.0; // Z方向最小坐标。
    double maximumX = 0.0; // X方向最大坐标。
    double maximumY = 0.0; // Y方向最大坐标。
    double maximumZ = 0.0; // Z方向最大坐标。
};

}

#endif // MYVOXEL_SHAPEBOUNDS_H