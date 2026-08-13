#include "Display_LineTypes.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Display_LineVertex::Display_LineVertex()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , red(1.0f)
    , green(1.0f)
    , blue(1.0f)
    , alpha(1.0f)
{
}

Display_LineVertex::Display_LineVertex(double xValue, double yValue, double zValue, const Display_Color& colorValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , red(colorValue.red())
    , green(colorValue.green())
    , blue(colorValue.blue())
    , alpha(colorValue.alpha())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Display line vertex requires finite position and valid color.");
}

bool Display_LineVertex::isValid() const
{
    return std::isfinite(static_cast<double>(x)) && std::isfinite(static_cast<double>(y)) &&
           std::isfinite(static_cast<double>(z)) && Display_Color(red, green, blue, alpha).isValid();
}

}
