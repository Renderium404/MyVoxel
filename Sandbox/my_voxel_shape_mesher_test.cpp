#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"
#include "MyVoxel/Mesh/Geometry_Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 回转圆弧测试使用的圆周率。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

MyVoxel::Shape makeShape(MyVoxel::Geometry_Shape* geometry)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape> resource(geometry);
    return MyVoxel::Shape(MyVoxel::Topology_Shape(resource));
}

void checkRenderable(const MyVoxel::Mesh& mesh, const char* name)
{
    check(mesh.isValid() && mesh.isRenderable() && !mesh.hasDegenerateTriangles(), name);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> resource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    check(resource && resource->isValid(), "Display resource accepts shape mesh");
}

std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > rectangleProfile(double radius)
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > curves;
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(0.0, -1.0, 0.0), MyMath::Vector3(radius, -1.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(radius, -1.0, 0.0), MyMath::Vector3(radius, 1.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(radius, 1.0, 0.0), MyMath::Vector3(0.0, 1.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(0.0, 1.0, 0.0), MyMath::Vector3(0.0, -1.0, 0.0))));
    return curves;
}

std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > arcProfile()
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > curves;
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Arc(MyMath::Vector3(0.0, 0.0, 0.0), 2.0, -Pi * 0.5, Pi)));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(0.0, 2.0, 0.0), MyMath::Vector3(0.0, -2.0, 0.0))));
    return curves;
}

}

int main()
{
    const MyVoxel::Display_Color color(0.35, 0.65, 0.90, 1.0);
    MyVoxel::ShapeMeshingOptions options;
    options.circularSegmentCount = 16;
    options.sphereStackCount = 8;
    options.profileArcSegmentCount = 16;

    const MyVoxel::Shape box = makeShape(new MyVoxel::Geometry_Box(2.0, 4.0, 6.0));
    const MyVoxel::Mesh boxMesh = MyVoxel::ShapeMesher::build(box, color, options);
    check(MyVoxel::ShapeMesher::supports(box), "Box supported");
    check(boxMesh.triangleCount() == 12, "Box triangle count");
    checkRenderable(boxMesh, "Box renderable");

    const MyVoxel::Shape sphere = makeShape(new MyVoxel::Geometry_Sphere(3.0));
    const MyVoxel::Mesh sphereMesh = MyVoxel::ShapeMesher::build(sphere, color, options);
    check(sphereMesh.triangleCount() == 224, "Sphere triangle count");
    checkRenderable(sphereMesh, "Sphere renderable");

    const MyVoxel::Shape cylinder = makeShape(new MyVoxel::Geometry_Cylinder(2.0, 5.0));
    const MyVoxel::Mesh cylinderMesh = MyVoxel::ShapeMesher::build(cylinder, color, options);
    check(cylinderMesh.triangleCount() == 64, "Cylinder triangle count");
    checkRenderable(cylinderMesh, "Cylinder renderable");

    const MyVoxel::Shape frustum = makeShape(new MyVoxel::Geometry_ConeFrustum(3.0, 1.5, 5.0));
    const MyVoxel::Mesh frustumMesh = MyVoxel::ShapeMesher::build(frustum, color, options);
    check(frustumMesh.triangleCount() == 64, "ConeFrustum triangle count");
    checkRenderable(frustumMesh, "ConeFrustum renderable");

    const MyVoxel::Shape cone = makeShape(new MyVoxel::Geometry_ConeFrustum(3.0, 0.0, 5.0));
    const MyVoxel::Mesh coneMesh = MyVoxel::ShapeMesher::build(cone, color, options);
    check(coneMesh.triangleCount() == 32, "Cone triangle count");
    checkRenderable(coneMesh, "Cone renderable");

    const MyVoxel::Shape invertedCone = makeShape(new MyVoxel::Geometry_ConeFrustum(0.0, 3.0, 5.0));
    const MyVoxel::Mesh invertedConeMesh = MyVoxel::ShapeMesher::build(invertedCone, color, options);
    check(invertedConeMesh.triangleCount() == 32, "Inverted cone triangle count");
    checkRenderable(invertedConeMesh, "Inverted cone renderable");

    const MyVoxel::Shape revolved = makeShape(new MyVoxel::Geometry_Revolved(rectangleProfile(2.0), 1.0e-9));
    const MyVoxel::Mesh revolvedMesh = MyVoxel::ShapeMesher::build(revolved, color, options);
    check(MyVoxel::ShapeMesher::supports(revolved), "Revolved supported");
    check(revolvedMesh.triangleCount() == 64, "Revolved rectangle triangle count");
    checkRenderable(revolvedMesh, "Revolved rectangle renderable");

    const MyVoxel::Shape negativeRevolved = makeShape(new MyVoxel::Geometry_Revolved(rectangleProfile(-2.0), 1.0e-9));
    const MyVoxel::Mesh negativeRevolvedMesh = MyVoxel::ShapeMesher::build(negativeRevolved, color, options);
    check(negativeRevolvedMesh.triangleCount() == 64, "Negative-side revolved triangle count");
    checkRenderable(negativeRevolvedMesh, "Negative-side revolved renderable");

    const MyVoxel::Shape arcRevolved = makeShape(new MyVoxel::Geometry_Revolved(arcProfile(), 1.0e-9));
    const MyVoxel::Mesh arcRevolvedMesh = MyVoxel::ShapeMesher::build(arcRevolved, color, options);
    check(arcRevolvedMesh.triangleCount() == 224, "Arc revolved triangle count");
    checkRenderable(arcRevolvedMesh, "Arc revolved renderable");

    const MyVoxel::Shape meshShape = makeShape(new MyVoxel::Geometry_Mesh(boxMesh));
    const MyVoxel::Mesh rebuiltMesh = MyVoxel::ShapeMesher::build(meshShape, MyVoxel::Display_Color(0.9, 0.3, 0.2, 1.0), options);
    check(MyVoxel::ShapeMesher::supports(meshShape), "Geometry_Mesh supported");
    check(rebuiltMesh.triangleCount() == boxMesh.triangleCount(), "Geometry_Mesh triangle count preserved");
    checkRenderable(rebuiltMesh, "Geometry_Mesh renderable");

    const MyVoxel::Mesh worldMesh = MyVoxel::ShapeMesher::buildWorld(box, color, options);
    check(worldMesh.triangleCount() == boxMesh.triangleCount(), "World build triangle count");
    checkRenderable(worldMesh, "World build renderable");

    std::cout << "ShapeMesher Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}