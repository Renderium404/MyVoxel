#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Modeling/Shape/VoxelizationWorkspace.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const double Tolerance = 1.0e-10; // 对齐体素化Feature包围盒比较使用的绝对容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

void testLocalPrimitiveVoxelization()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-8.0, -8.0, -8.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));

    const MyVoxel::Topology_Shape box = MyVoxel::Modeling::createBox(3.0, 4.0, 5.0);
    const MyVoxel::VoxelShape boxVoxels = MyVoxel::Modeling::voxelize(box, grid);
    check(boxVoxels.isValid() && !boxVoxels.isEmpty(), "Box voxelization valid");
    check(boxVoxels.hasCompleteFeatures(), "Box voxelization marks FeatureSet complete");
    check(boxVoxels.features().vertexCount() == 8 && boxVoxels.features().edgeCount() == 12, "Box voxelization stores 8 vertices and 12 edges");
    check(boxVoxels.features().bounds().isEqualTo(box.geometry().localBounds(), 0.0), "Box voxel Features remain in local query space");

    const MyVoxel::Topology_Shape sphere = MyVoxel::Modeling::createSphere(2.0);
    const MyVoxel::VoxelShape sphereVoxels = MyVoxel::Modeling::voxelize(sphere, grid);
    check(sphereVoxels.hasCompleteFeatures(), "Sphere voxelization marks empty FeatureSet complete");
    check(sphereVoxels.features().isEmpty(), "Sphere voxelization stores confirmed empty FeatureSet");

    const MyVoxel::Topology_Shape frustum = MyVoxel::Modeling::createConeFrustum(2.5, 1.25, 4.0);
    const MyVoxel::VoxelShape frustumVoxels = MyVoxel::Modeling::voxelize(frustum, grid);
    check(frustumVoxels.isValid() && !frustumVoxels.isEmpty(), "Unsupported-feature ConeFrustum TSDF voxelization remains valid");
    check(!frustumVoxels.hasCompleteFeatures(), "Unsupported-feature ConeFrustum is marked Feature unavailable");
    check(frustumVoxels.features().isEmpty(), "Unavailable Feature state does not retain misleading partial data");
}

void testShapeLocalAndAlignedCoordinates()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-8.0, -8.0, -8.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyMath::Matrix4 placement = MyMath::Matrix4::fromTranslation(MyMath::Vector3(6.0, -2.0, 3.0));
    const MyVoxel::Shape placedBox = MyVoxel::Modeling::makeBox(2.0, 4.0, 6.0, placement);
    const MyVoxel::VoxelShape localVoxels = MyVoxel::Modeling::voxelize(placedBox, grid);

    check(localVoxels.transform().isEqualTo(placement, 0.0), "Shape local voxelization preserves instance placement on VoxelShape");
    check(localVoxels.hasCompleteFeatures() && localVoxels.features().edgeCount() == 12, "Shape local voxelization stores complete local Features");
    check(localVoxels.features().bounds().isEqualTo(placedBox.topology().geometry().localBounds(), 0.0), "Shape local voxelization keeps Features in Shape local space");

    MyVoxel::VoxelShape reference(grid, 0.75f);
    reference.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(4.0, -5.0, 1.0)));
    const MyVoxel::Shape alignedBox = MyVoxel::Modeling::makeBox(2.0, 2.0, 2.0, placement);
    const MyVoxel::ShapeQuery query(alignedBox, reference.transform());
    const MyVoxel::VoxelShape alignedVoxels = MyVoxel::Modeling::voxelizeAligned(alignedBox, reference);

    check(alignedVoxels.transform().isEqualTo(reference.transform(), 0.0), "Aligned voxelization inherits reference transform");
    check(alignedVoxels.hasCompleteFeatures() && alignedVoxels.features().vertexCount() == 8, "Aligned Box stores complete transformed Features");
    check(alignedVoxels.features().bounds().isEqualTo(query.queryBounds(), Tolerance), "Aligned Features are stored in reference local query space");
}

void testDirectFieldMutationInvalidatesFeatures()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(MyVoxel::Modeling::createBox(2.0, 2.0, 2.0), grid);
    check(voxels.hasCompleteFeatures() && voxels.features().edgeCount() == 12, "Precondition: voxelized Box has complete Features");

    {
        MyVoxel::VoxelShapeSession session(voxels, MyVoxel::BaseVoxelLevel);
        const MyVoxel::VoxelCellAddress root(MyVoxel::VoxelCellIndex(8, 8, 8), MyVoxel::BaseVoxelLevel);
        check(session.setState(root, MyVoxel::VoxelState::Material), "Direct TSDF mutation changes field");
        check(!session.hasCompleteFeatures() && session.features().isEmpty(), "Direct TSDF mutation invalidates and clears explicit Features");
    }

    check(!voxels.hasCompleteFeatures() && voxels.features().isEmpty(), "VoxelShape preserves unavailable Feature state after direct field mutation");
}

void testWorkspaceFeatureReuse()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-6.0, -6.0, -6.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyVoxel::VoxelShape reference(grid, 0.75f);
    MyVoxel::Modeling::VoxelizationWorkspace workspace;

    const MyVoxel::VoxelShape& boxResult = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeBox(3.0, 3.0, 3.0), reference, workspace);
    check(boxResult.hasCompleteFeatures() && boxResult.features().edgeCount() == 12, "Workspace first Box result stores complete Features");

    const MyVoxel::VoxelShape& sphereResult = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(1.8), reference, workspace);
    check(sphereResult.hasCompleteFeatures() && sphereResult.features().isEmpty(), "Workspace Box-to-Sphere reuse removes all stale Box Features");

    const MyVoxel::VoxelShape& secondBox = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeBox(2.0, 2.0, 2.0), reference, workspace);
    check(secondBox.hasCompleteFeatures() && secondBox.features().vertexCount() == 8 && secondBox.features().edgeCount() == 12, "Workspace Sphere-to-Box reuse rebuilds Box Features");

    const MyVoxel::VoxelShape& frustumResult = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeConeFrustum(2.0, 1.0, 3.0), reference, workspace);
    check(!frustumResult.hasCompleteFeatures() && frustumResult.features().isEmpty(), "Workspace unsupported Shape does not inherit previous Box Features");

    workspace.reset();
    check(workspace.result().isEmpty(), "Workspace reset clears TSDF result");
    check(workspace.result().hasCompleteFeatures() && workspace.result().features().isEmpty(), "Workspace reset restores complete empty Feature state");
}

}

int main()
{
    std::cout << "MyVoxel voxelization Feature integration F3B2 test" << std::endl << std::endl;
    testLocalPrimitiveVoxelization();
    testShapeLocalAndAlignedCoordinates();
    testDirectFieldMutationInvalidatesFeatures();
    testWorkspaceFeatureReuse();
    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
