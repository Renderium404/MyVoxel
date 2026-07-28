#include "ShapeBounds.h"

#include <cassert>
#include <cmath>

namespace MyVoxel
{

ShapeBounds::ShapeBounds(double minimumXValue, double minimumYValue, double minimumZValue, double maximumXValue, double maximumYValue, double maximumZValue)
    : minimumX(minimumXValue)
    , minimumY(minimumYValue)
    , minimumZ(minimumZValue)
    , maximumX(maximumXValue)
    , maximumY(maximumYValue)
    , maximumZ(maximumZValue)
{
    assert(isValid());
}

bool ShapeBounds::isValid() const
{
    return std::isfinite(minimumX) && std::isfinite(minimumY) && std::isfinite(minimumZ) &&
           std::isfinite(maximumX) && std::isfinite(maximumY) && std::isfinite(maximumZ) &&
           minimumX <= maximumX && minimumY <= maximumY && minimumZ <= maximumZ;
}

bool ShapeBounds::hasVolume() const
{
    return isValid() && minimumX < maximumX && minimumY < maximumY && minimumZ < maximumZ;
}

}