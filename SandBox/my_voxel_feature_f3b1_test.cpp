#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 使用共享拓扑端点建立一条直线Feature Edge。
MyVoxel::Topology_Edge makeLineEdge(const MyVoxel::Topology_Vertex& startVertex, const MyVoxel::Topology_Vertex& endVertex)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> geometry(
        new MyVoxel::Geometry_Line(startVertex.point(), endVertex.point()));
    return MyVoxel::Topology_Edge(startVertex, endVertex, geometry, 0.0);
}

// 建立一条最小可验证Feature链。
MyVoxel::VoxelFeatureSet makeFeatureSet(double offset)
{
    const MyVoxel::Topology_Vertex a(MyMath::Vector3(offset, 0.0, 0.0));
    const MyVoxel::Topology_Vertex b(MyMath::Vector3(offset + 1.0, 0.0, 0.0));
    MyVoxel::VoxelFeatureSet features;
    features.addEdge(makeLineEdge(a, b));
    return features;
}

}

int main()
{
    std::cout << "MyVoxel VoxelShape Feature COW F3B1 test" << std::endl << std::endl;

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    MyVoxel::VoxelShape original(grid, 0.75f);

    check(original.isValid(), "Fresh VoxelShape valid with FeatureSet state");
    check(original.features().isValid() && original.features().isEmpty(), "Fresh VoxelShape owns valid empty FeatureSet");

    const MyVoxel::VoxelFeatureSet firstFeatures = makeFeatureSet(0.0);

    {
        MyVoxel::VoxelShapeSession session(original, MyVoxel::BaseVoxelLevel);
        session.setFeatures(firstFeatures);
        check(session.features().edgeCount() == 1 && session.features().vertexCount() == 2, "Session installs FeatureSet into VoxelShape shared state");
        check(!session.hasChanges(), "Feature-only mutation does not pollute current field ChangeSet");
    }

    check(original.features().edgeCount() == 1 && original.features().vertexCount() == 2, "VoxelShape exposes installed FeatureSet");
    check(original.isValid(), "VoxelShape remains valid after FeatureSet installation");

    MyVoxel::VoxelShape copy = original;
    check(original.sharesDataWith(copy), "VoxelShape copy shares TSDF and FeatureSet SharedData");
    check(copy.features().edges()[0].isSame(original.features().edges()[0]), "Shared copy preserves exact Topology_Edge feature identity");

    {
        MyVoxel::VoxelShapeSession session(copy, MyVoxel::BaseVoxelLevel);
        check(!original.sharesDataWith(copy), "Opening write Session detaches complete SharedData including FeatureSet");
        check(session.features().edges()[0].isSame(original.features().edges()[0]), "Detached FeatureSet copy preserves immutable Topology feature identity");
        const MyVoxel::VoxelCellAddress root(MyVoxel::VoxelCellIndex(0, 0, 0), MyVoxel::BaseVoxelLevel);
        check(session.setState(root, MyVoxel::VoxelState::Material), "Detached copy TSDF can change independently");
    }

    check(original.isEmpty(), "Original TSDF remains unchanged after copied Session mutation");
    check(!copy.isEmpty(), "Copied TSDF receives independent mutation");
    check(original.hasCompleteFeatures() && original.features().edgeCount() == 1, "Original FeatureSet remains complete after copied TSDF mutation");
    check(!copy.hasCompleteFeatures() && copy.features().isEmpty(), "Direct copied TSDF mutation invalidates stale FeatureSet");

    {
        MyVoxel::VoxelShapeSession session(copy, MyVoxel::BaseVoxelLevel);
        const MyVoxel::VoxelFeatureSet secondFeatures = makeFeatureSet(5.0);
        session.setFeatures(secondFeatures);
        check(session.hasCompleteFeatures(), "Installing replacement FeatureSet restores Complete state");
        check(session.features().edges()[0].startVertex().point().x() == 5.0, "Session can replace detached FeatureSet");
    }

    check(original.features().edges()[0].startVertex().point().x() == 0.0, "Replacing copied FeatureSet does not modify original FeatureSet");
    check(copy.features().edges()[0].startVertex().point().x() == 5.0, "Copied VoxelShape receives replacement FeatureSet");

    {
        MyVoxel::VoxelShapeSession session(copy, MyVoxel::BaseVoxelLevel);
        check(session.clear(), "Session clear removes TSDF and FeatureSet together");
        check(session.isEmpty(), "Session clear removes all TSDF roots");
        check(session.features().isEmpty(), "Session clear removes all explicit Features");
    }

    check(copy.isEmpty() && copy.features().isEmpty(), "Cleared VoxelShape has empty TSDF and FeatureSet");
    check(original.features().edgeCount() == 1, "Clearing copy preserves original FeatureSet");
    check(original.isValid() && copy.isValid(), "Both original and detached copy remain valid");

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
