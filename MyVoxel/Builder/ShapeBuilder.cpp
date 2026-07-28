#include "ShapeBuilder.h"

#include <cassert>
#include <cmath>
#include <memory>

#include "../Shape/BoxGeometry.h"
#include "../Shape/CylinderGeometry.h"
#include "../Shape/ShapeGeometry.h"

namespace MyVoxel
{

Shape ShapeBuilder::makeBox(double minimumX, double minimumY, double minimumZ, double maximumX, double maximumY, double maximumZ)
{
    assert(std::isfinite(minimumX));
    assert(std::isfinite(minimumY));
    assert(std::isfinite(minimumZ));
    assert(std::isfinite(maximumX));
    assert(std::isfinite(maximumY));
    assert(std::isfinite(maximumZ));
    assert(minimumX < maximumX);
    assert(minimumY < maximumY);
    assert(minimumZ < maximumZ);

    const std::shared_ptr<const ShapeGeometry> geometry = std::make_shared<BoxGeometry>(minimumX, minimumY, minimumZ, maximumX, maximumY, maximumZ);
    return Shape(geometry);
}

Shape ShapeBuilder::makeBox(const ShapeBounds& bounds)
{
    assert(bounds.isValid());
    assert(bounds.hasVolume());

    return makeBox(bounds.minimumX, bounds.minimumY, bounds.minimumZ, bounds.maximumX, bounds.maximumY, bounds.maximumZ);
}

Shape ShapeBuilder::makeCylinder(double centerX, double centerY, double minimumZ, double maximumZ, double radius)
{
    assert(std::isfinite(centerX));
    assert(std::isfinite(centerY));
    assert(std::isfinite(minimumZ));
    assert(std::isfinite(maximumZ));
    assert(std::isfinite(radius));
    assert(minimumZ < maximumZ);
    assert(radius > 0.0);

    const std::shared_ptr<const ShapeGeometry> geometry = std::make_shared<CylinderGeometry>(centerX, centerY, minimumZ, maximumZ, radius);
    return Shape(geometry);
}

}