#ifndef MYVOXEL_SHAPEBUILDER_H
#define MYVOXEL_SHAPEBUILDER_H

#include "../Shape/Shape.h"
#include "../Shape/ShapeBounds.h"

namespace MyVoxel
{

// 提供标准连续Shape的创建入口。
class ShapeBuilder
{
public:
    /// 盒体

    // 根据局部坐标范围创建轴对齐盒体。
    static Shape makeBox(double minimumX, double minimumY, double minimumZ, double maximumX, double maximumY, double maximumZ);

    // 根据局部轴对齐包围盒创建盒体。
    static Shape makeBox(const ShapeBounds& bounds);

    /// 圆柱体

    // 创建局部轴线平行于Z轴的圆柱体。
    static Shape makeCylinder(double centerX, double centerY, double minimumZ, double maximumZ, double radius);
};

}

#endif // MYVOXEL_SHAPEBUILDER_H