#include "PrimitiveModeling.h"

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/BaseShape/BoxGeometry.h"
#include "MyVoxel/Geometry/BaseShape/ConeFrustumGeometry.h"
#include "MyVoxel/Geometry/BaseShape/CylinderGeometry.h"
#include "MyVoxel/Geometry/BaseShape/SphereGeometry.h"
#include "MyVoxel/Geometry/ShapeGeometry.h"

namespace MyVoxel
{
namespace Modeling
{

/// 标准基础体创建

Geometry::Shape makeBox(double sizeX, double sizeY, double sizeZ)
{
    const Foundation::RefPtr<Geometry::BoxGeometry> geometry = Foundation::makeRef<Geometry::BoxGeometry>(sizeX, sizeY, sizeZ);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

Geometry::Shape makeSphere(double radius)
{
    const Foundation::RefPtr<Geometry::SphereGeometry> geometry = Foundation::makeRef<Geometry::SphereGeometry>(radius);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

Geometry::Shape makeCylinder(double radius, double height)
{
    const Foundation::RefPtr<Geometry::CylinderGeometry> geometry = Foundation::makeRef<Geometry::CylinderGeometry>(radius, height);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

Geometry::Shape makeCone(double bottomRadius, double height)
{
    const Foundation::RefPtr<Geometry::ConeFrustumGeometry> geometry = Foundation::makeRef<Geometry::ConeFrustumGeometry>(bottomRadius, 0.0, height);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

Geometry::Shape makeConeFrustum(double bottomRadius, double topRadius, double height)
{
    const Foundation::RefPtr<Geometry::ConeFrustumGeometry> geometry = Foundation::makeRef<Geometry::ConeFrustumGeometry>(bottomRadius, topRadius, height);
    const Foundation::RefPtr<const Geometry::ShapeGeometry> shapeGeometry = geometry;
    return Geometry::Shape(shapeGeometry);
}

}
}