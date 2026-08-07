#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Surface/SurfaceKind.h"
#include "MyVoxel/Topology/Topology_Shape.h"
#include "MyVoxel/Modeling/Shape/Topology_ShapeBuilder.h"

namespace
{

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

bool isDirection(const MyMath::Vector3& value, const MyMath::Vector3& expected)
{
    return value.isEqualTo(expected, 1.0e-12);
}

}

int main()
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape> geometry(
        new MyVoxel::Geometry_Box(2.0, 4.0, 6.0));

    check(geometry && geometry->kind() == MyVoxel::ShapeKind::Box, "Box geometry valid");
    check(MyVoxel::Modeling::Topology_ShapeBuilder::supports(*geometry), "Box topology supported");

    const MyVoxel::Topology_Shape compatibilityTopology(geometry);
    check(compatibilityTopology.isValid(), "Geometry-only Topology_Shape remains valid");
    check(!compatibilityTopology.hasFaces(), "Geometry-only Topology_Shape has no faces");
    check(compatibilityTopology.faceCount() == 0, "Geometry-only face count");
    check(compatibilityTopology.containsPoint(MyMath::Vector3::zero()), "Geometry-only query compatibility");

    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::Topology_ShapeBuilder::build(geometry);
    check(topology.isValid(), "Box topology valid");
    check(topology.hasFaces(), "Box topology has faces");
    check(topology.faceCount() == 6, "Box topology face count");
    check(topology.geometryPointer() == geometry.get(), "Box topology preserves geometry resource");
    check(topology.kind() == MyVoxel::ShapeKind::Box, "Box topology kind");

    bool negativeX = false;
    bool positiveX = false;
    bool negativeY = false;
    bool positiveY = false;
    bool negativeZ = false;
    bool positiveZ = false;
    bool allPlane = true;
    bool allRectangles = true;
    bool allNoHoles = true;
    MyVoxel::Bounds3 combinedFaceBounds;

    for (std::size_t faceIndex = 0; faceIndex < topology.faceCount(); ++faceIndex)
    {
        const MyVoxel::Topology_Face& face = topology.face(faceIndex);
        allPlane = allPlane && face.isValid() && face.surfaceKind() == MyVoxel::SurfaceKind::Plane;
        allRectangles = allRectangles && face.outerWire().curveCount() == 4 && face.outerWire().isClosed();
        allNoHoles = allNoHoles && face.innerWireCount() == 0;
        combinedFaceBounds.include(face.localBounds());

        const MyMath::Vector3 normal = face.normalAt(MyMath::Vector3::zero());

        negativeX = negativeX || isDirection(normal, MyMath::Vector3(-1.0, 0.0, 0.0));
        positiveX = positiveX || isDirection(normal, MyMath::Vector3(1.0, 0.0, 0.0));
        negativeY = negativeY || isDirection(normal, MyMath::Vector3(0.0, -1.0, 0.0));
        positiveY = positiveY || isDirection(normal, MyMath::Vector3(0.0, 1.0, 0.0));
        negativeZ = negativeZ || isDirection(normal, MyMath::Vector3(0.0, 0.0, -1.0));
        positiveZ = positiveZ || isDirection(normal, MyMath::Vector3(0.0, 0.0, 1.0));
    }

    check(allPlane, "All Box faces use Plane surfaces");
    check(allRectangles, "All Box faces use rectangle outer wires");
    check(allNoHoles, "All Box faces have no holes");
    check(negativeX && positiveX && negativeY && positiveY && negativeZ && positiveZ, "Box face normals cover six outward directions");
    check(combinedFaceBounds.isEqualTo(topology.bounds(), 1.0e-12), "Combined Face bounds match Box geometry bounds");

    std::cout << "Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}