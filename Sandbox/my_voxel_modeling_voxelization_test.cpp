#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Modeling/Shape/MeshModeling.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/RevolvedModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Modeling/Shape/VoxelizationWorkspace.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Curve.h"
#include "MyVoxel/Topology/Topology_Shape.h"
#include "MyVoxel/Topology/Topology_Wire.h"

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

// 创建回转建模测试使用的局部XY平面矩形轮廓。
MyVoxel::Topology_Wire makeRevolvedProfile()
{
    std::vector<MyVoxel::Topology_Curve> curves;
    curves.reserve(4); // 矩形闭合轮廓固定由四条直线组成。
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(1.0, -2.0, 0.0), MyMath::Vector3(3.0, -2.0, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, -2.0, 0.0), MyMath::Vector3(3.0, 2.0, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, 2.0, 0.0), MyMath::Vector3(1.0, 2.0, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(1.0, 2.0, 0.0), MyMath::Vector3(1.0, -2.0, 0.0)))));
    return MyVoxel::Topology_Wire(curves, 1.0e-9);
}

void checkTopology(const MyVoxel::Topology_Shape& topology, MyVoxel::ShapeKind kind, const char* name)
{
    check(topology.isValid() && topology.kind() == kind && topology.bounds().isValid(), name);
}

void checkVoxelized(const MyVoxel::Topology_Shape& topology, const MyVoxel::VoxelGrid& grid, const char* name)
{
    MyVoxel::Modeling::VoxelizationStatistics statistics;
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid, statistics);
    check(voxels.isValid() && !voxels.isEmpty() && voxels.rootCount() > 0, name);
    check(voxels.transform().isIdentity(0.0), "Topology voxelization keeps identity transform");
    check(statistics.rootCandidateCount > 0 && statistics.visitedCellCount > 0, "Topology voxelization statistics");
}

}

int main()
{
    const MyVoxel::Topology_Shape box = MyVoxel::Modeling::createBox(4.0, 5.0, 6.0);
    const MyVoxel::Topology_Shape sphere = MyVoxel::Modeling::createSphere(3.0);
    const MyVoxel::Topology_Shape cylinder = MyVoxel::Modeling::createCylinder(2.5, 6.0);
    const MyVoxel::Topology_Shape cone = MyVoxel::Modeling::createCone(3.0, 6.0);
    const MyVoxel::Topology_Shape frustum = MyVoxel::Modeling::createConeFrustum(3.0, 1.5, 6.0);
    const MyVoxel::Topology_Shape revolved = MyVoxel::Modeling::createRevolved(makeRevolvedProfile());

    checkTopology(box, MyVoxel::ShapeKind::Box, "Box modeling");
    checkTopology(sphere, MyVoxel::ShapeKind::Sphere, "Sphere modeling");
    checkTopology(cylinder, MyVoxel::ShapeKind::Cylinder, "Cylinder modeling");
    checkTopology(cone, MyVoxel::ShapeKind::ConeFrustum, "Cone modeling");
    checkTopology(frustum, MyVoxel::ShapeKind::ConeFrustum, "ConeFrustum modeling");
    checkTopology(revolved, MyVoxel::ShapeKind::Revolved, "Revolved modeling");

    const MyVoxel::Mesh sourceMesh =
        MyVoxel::ShapeMesher::build(box, MyVoxel::Display_Color(0.5, 0.6, 0.7, 1.0));
    const MyVoxel::Topology_Shape meshShape = MyVoxel::Modeling::createMesh(sourceMesh);
    checkTopology(meshShape, MyVoxel::ShapeKind::Mesh, "Mesh modeling");

    const MyVoxel::VoxelGrid grid(4.0, static_cast<MyVoxel::VoxelLevel>(3));
    checkVoxelized(box, grid, "Box voxelization");
    checkVoxelized(sphere, grid, "Sphere voxelization");
    checkVoxelized(cylinder, grid, "Cylinder voxelization");
    checkVoxelized(cone, grid, "Cone voxelization");
    checkVoxelized(frustum, grid, "ConeFrustum voxelization");
    checkVoxelized(revolved, grid, "Revolved voxelization");
    checkVoxelized(meshShape, grid, "Mesh voxelization");

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(12.0, -3.0, 5.0));
    const MyVoxel::Shape placedBox(box, transform);
    const MyVoxel::VoxelShape localPlaced = MyVoxel::Modeling::voxelize(placedBox, grid);
    check(localPlaced.isValid() && !localPlaced.isEmpty(), "Placed Shape local voxelization");
    check(localPlaced.transform().isEqualTo(transform, 0.0), "Placed Shape transform preserved");

    const MyVoxel::VoxelShape reference = MyVoxel::Modeling::voxelize(box, grid);
    const MyVoxel::VoxelShape aligned = MyVoxel::Modeling::voxelizeAligned(placedBox, reference);
    check(aligned.isValid() && !aligned.isEmpty(), "Aligned Shape voxelization");
    check(aligned.grid().isEqualTo(reference.grid(), 0.0), "Aligned voxelization grid preserved");
    check(aligned.transform().isEqualTo(reference.transform(), 0.0), "Aligned voxelization transform preserved");

    MyVoxel::Modeling::VoxelizationWorkspace workspace;
    const MyVoxel::VoxelShape& firstWorkspaceResult = MyVoxel::Modeling::voxelizeAligned(placedBox, reference, workspace);
    check(firstWorkspaceResult.isValid() && !firstWorkspaceResult.isEmpty() && workspace.isInitialized(), "Workspace first voxelization");
    workspace.resetStatistics();
    const MyVoxel::VoxelShape& secondWorkspaceResult = MyVoxel::Modeling::voxelizeAligned(placedBox, reference, workspace);
    check(secondWorkspaceResult.isValid() && !secondWorkspaceResult.isEmpty(), "Workspace second voxelization");
    check(workspace.exactRootReuseCount() + workspace.fallbackRootReuseCount() > 0, "Workspace reuses root storage");

    std::cout << "Modeling/Voxelization Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
