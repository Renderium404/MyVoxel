#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 返回Mesh顶点位置。
MyMath::Vector3 position(const MyVoxel::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 返回球面顶点最大径向误差。
double maximumSphereError(const MyVoxel::Mesh& mesh, double radius)
{
    double result = 0.0;
    const std::vector<MyVoxel::MeshVertex>& vertices = mesh.vertices();

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        result = (std::max)(result, std::fabs(position(vertices[index]).length() - radius));
    }

    return result;
}

// 返回Box棱附近顶点到理想XY棱线的平均横向误差。
double averageBoxXYEdgeError(const MyVoxel::Mesh& mesh, double halfSize, double band)
{
    double error = 0.0;
    std::size_t count = 0;
    const std::vector<MyVoxel::MeshVertex>& vertices = mesh.vertices();

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        const MyMath::Vector3 p = position(vertices[index]);
        const double dx = std::fabs(std::fabs(p.x()) - halfSize);
        const double dy = std::fabs(std::fabs(p.y()) - halfSize);

        if (dx <= band && dy <= band && std::fabs(p.z()) < halfSize - band)
        {
            error += std::sqrt(dx * dx + dy * dy);
            ++count;
        }
    }

    return count > 0 ? error / static_cast<double>(count) : band * 10.0;
}

}

int main()
{
    std::cout << "MyVoxel feature-sensitive TSDF Surface Nets test" << std::endl << std::endl;

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));
    const float backgroundDistance = 0.75f;
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);

    const MyVoxel::VoxelShape box = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeBox(3.0, 3.0, 3.0), reference);
    MyVoxel::Meshing::VolumeMeshingOptions standardOptions;
    MyVoxel::Meshing::VolumeMeshingOptions featureOptions;
    featureOptions.mode = MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets;
    featureOptions.featureAngleDegrees = 30.0;

    const MyVoxel::Mesh standardBox = MyVoxel::Meshing::VolumeMesher::build(box, standardOptions);
    const MyVoxel::Mesh featureBox = MyVoxel::Meshing::VolumeMesher::build(box, featureOptions);

    check(standardBox.isRenderable() && featureBox.isRenderable(), "Standard and feature Box meshes are renderable");
    check(standardBox.triangleCount() == featureBox.triangleCount(), "Feature mode preserves Box triangle topology count");
    check(!featureBox.hasDegenerateTriangles(1.0e-10), "Feature Box contains no degenerate triangles");

    const double h = grid.minimumCellEdgeLength();
    const double standardEdgeError = averageBoxXYEdgeError(standardBox, 1.5, h * 1.25);
    const double featureEdgeError = averageBoxXYEdgeError(featureBox, 1.5, h * 1.25);
    std::cout << "Box XY edge average error: standard=" << standardEdgeError << " feature=" << featureEdgeError << std::endl;
    check(featureEdgeError < standardEdgeError, "Feature QEF moves Box edge vertices closer to the analytic sharp edge");

    const MyVoxel::VoxelShape sphere = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(1.5), reference);
    const MyVoxel::Mesh standardSphere = MyVoxel::Meshing::VolumeMesher::build(sphere, standardOptions);
    const MyVoxel::Mesh featureSphere = MyVoxel::Meshing::VolumeMesher::build(sphere, featureOptions);

    check(featureSphere.isRenderable(), "Feature Sphere mesh is renderable");
    check(featureSphere.triangleCount() == standardSphere.triangleCount(), "Feature mode preserves Sphere triangle topology count");
    check(!featureSphere.hasDegenerateTriangles(1.0e-10), "Feature Sphere contains no degenerate triangles");

    const double standardSphereError = maximumSphereError(standardSphere, 1.5);
    const double featureSphereError = maximumSphereError(featureSphere, 1.5);
    std::cout << "Sphere maximum radial error: standard=" << standardSphereError << " feature=" << featureSphereError << std::endl;
    check(featureSphereError <= standardSphereError + h * 0.05,
          "Feature threshold keeps smooth Sphere geometry no worse than Standard beyond a small tolerance");

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}