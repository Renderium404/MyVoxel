#include "DisplayColor.h"

#include <cassert>
#include <cmath>

namespace MyVoxelViewer
{

DisplayColor::DisplayColor(double redValue, double greenValue, double blueValue, double alphaValue)
    : red(static_cast<float>(redValue))
    , green(static_cast<float>(greenValue))
    , blue(static_cast<float>(blueValue))
    , alpha(static_cast<float>(alphaValue))
{
    assert(std::isfinite(redValue));
    assert(std::isfinite(greenValue));
    assert(std::isfinite(blueValue));
    assert(std::isfinite(alphaValue));
}

}