#include "Display_MeshTypes.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double UnitNormalTolerance = 1.0e-5; // 可直接上传显示顶点时允许的单位法线误差。

// 判断颜色分量是否为有效[0,1]数据。
bool isValidColorComponent(float value)
{
    return std::isfinite(static_cast<double>(value)) && value >= 0.0f && value <= 1.0f;
}

}

namespace MyVoxel
{

Display_MeshVertex::Display_MeshVertex()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , normalX(0.0f)
    , normalY(0.0f)
    , normalZ(0.0f)
    , red(1.0f)
    , green(1.0f)
    , blue(1.0f)
    , alpha(1.0f)
{
}

Display_MeshVertex::Display_MeshVertex(double xValue, double yValue, double zValue,
                                       const Display_Normal& normalValue, const Display_Color& colorValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , normalX(normalValue.x())
    , normalY(normalValue.y())
    , normalZ(normalValue.z())
    , red(colorValue.red())
    , green(colorValue.green())
    , blue(colorValue.blue())
    , alpha(colorValue.alpha())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Display mesh vertex data must be finite and contain a valid unit normal and color.");
}

bool Display_MeshVertex::isValid() const
{
    if (!std::isfinite(static_cast<double>(x)) || !std::isfinite(static_cast<double>(y)) ||
        !std::isfinite(static_cast<double>(z)) || !std::isfinite(static_cast<double>(normalX)) ||
        !std::isfinite(static_cast<double>(normalY)) || !std::isfinite(static_cast<double>(normalZ)))
    {
        return false;
    }

    const double normalLengthSquared = static_cast<double>(normalX) * normalX +
                                       static_cast<double>(normalY) * normalY +
                                       static_cast<double>(normalZ) * normalZ;

    if (!std::isfinite(normalLengthSquared) ||
        std::fabs(std::sqrt(normalLengthSquared) - 1.0) > UnitNormalTolerance)
    {
        return false;
    }

    return isValidColorComponent(red) && isValidColorComponent(green) &&
           isValidColorComponent(blue) && isValidColorComponent(alpha);
}

}
