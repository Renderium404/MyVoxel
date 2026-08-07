#include "PrimitiveModeling.h"

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

Topology_Shape createBox(double sizeX, double sizeY, double sizeZ)
{
    const Foundation::RefPtr<Geometry_Box> geometry = Foundation::makeRef<Geometry_Box>(sizeX, sizeY, sizeZ);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

Topology_Shape createSphere(double radius)
{
    const Foundation::RefPtr<Geometry_Sphere> geometry = Foundation::makeRef<Geometry_Sphere>(radius);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

Topology_Shape createCylinder(double radius, double height)
{
    const Foundation::RefPtr<Geometry_Cylinder> geometry = Foundation::makeRef<Geometry_Cylinder>(radius, height);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

Topology_Shape createCone(double bottomRadius, double height)
{
    const Foundation::RefPtr<Geometry_ConeFrustum> geometry = Foundation::makeRef<Geometry_ConeFrustum>(bottomRadius, 0.0, height);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

Topology_Shape createConeFrustum(double bottomRadius, double topRadius, double height)
{
    const Foundation::RefPtr<Geometry_ConeFrustum> geometry = Foundation::makeRef<Geometry_ConeFrustum>(bottomRadius, topRadius, height);
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

/// 空间Shape实例创建

Shape makeBox(double sizeX, double sizeY, double sizeZ)
{
    return Shape(createBox(sizeX, sizeY, sizeZ));
}

Shape makeBox(double sizeX, double sizeY, double sizeZ, const MyMath::Matrix4& localToWorld)
{
    return Shape(createBox(sizeX, sizeY, sizeZ), localToWorld);
}

Shape makeSphere(double radius)
{
    return Shape(createSphere(radius));
}

Shape makeSphere(double radius, const MyMath::Matrix4& localToWorld)
{
    return Shape(createSphere(radius), localToWorld);
}

Shape makeCylinder(double radius, double height)
{
    return Shape(createCylinder(radius, height));
}

Shape makeCylinder(double radius, double height, const MyMath::Matrix4& localToWorld)
{
    return Shape(createCylinder(radius, height), localToWorld);
}

Shape makeCone(double bottomRadius, double height)
{
    return Shape(createCone(bottomRadius, height));
}

Shape makeCone(double bottomRadius, double height, const MyMath::Matrix4& localToWorld)
{
    return Shape(createCone(bottomRadius, height), localToWorld);
}

Shape makeConeFrustum(double bottomRadius, double topRadius, double height)
{
    return Shape(createConeFrustum(bottomRadius, topRadius, height));
}

Shape makeConeFrustum(double bottomRadius, double topRadius, double height, const MyMath::Matrix4& localToWorld)
{
    return Shape(createConeFrustum(bottomRadius, topRadius, height), localToWorld);
}

}
}
