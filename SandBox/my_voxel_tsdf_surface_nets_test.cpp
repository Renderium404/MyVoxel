#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#include "MyMath/Vector3.h"

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Meshing/VolumeMeshingWorkspace.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果并累计统计。
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

// 返回MeshVertex位置。
MyMath::Vector3 vertexPosition(const MyVoxel::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 返回MeshVertex显示法线。
MyMath::Vector3 vertexNormal(const MyVoxel::MeshVertex& vertex)
{
    return vertex.normal.vector();
}

// 比较两个Mesh的几何数据是否在指定误差内一致。
bool sameMeshGeometry(const MyVoxel::Mesh& first, const MyVoxel::Mesh& second, double tolerance)
{
    if (first.vertexCount() != second.vertexCount() || first.indexCount() != second.indexCount())
    {
        return false;
    }

    const std::vector<MyVoxel::MeshVertex>& firstVertices = first.vertices();
    const std::vector<MyVoxel::MeshVertex>& secondVertices = second.vertices();

    for (std::size_t index = 0; index < firstVertices.size(); ++index)
    {
        if (!vertexPosition(firstVertices[index]).isEqualTo(vertexPosition(secondVertices[index]), tolerance) ||
            !vertexNormal(firstVertices[index]).isEqualTo(vertexNormal(secondVertices[index]), tolerance))
        {
            return false;
        }
    }

    return first.indices() == second.indices();
}

// 创建统一球体TSDF测试对象。
MyVoxel::VoxelShape makeSphereVoxels(double radius, const MyVoxel::VoxelGrid& grid, float backgroundDistance)
{
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    return MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(radius), reference);
}

/// 空场

void testEmptyShape()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelShape shape(grid, 0.75f);
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(shape);

    check(mesh.isValid() && mesh.isEmpty(), "Empty TSDF shape produces an empty valid Surface Nets mesh");
    check(!MyVoxel::Meshing::VolumeMesher::defaultSampleRange(shape).isValid(), "Empty TSDF shape has no default meshing range");
}

/// Sphere几何与光滑法线

void testSphereSurface()
{
    const double radius = 1.5;
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5)); // h=0.125。
    const MyVoxel::VoxelShape voxels = makeSphereVoxels(radius, grid, 0.75f);
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = MyVoxel::Display_Color(0.8, 0.8, 0.85, 1.0);

    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);

    check(mesh.isValid() && !mesh.isEmpty(), "Sphere TSDF produces non-empty valid Surface Nets mesh");
    check(mesh.isRenderable(), "Sphere Surface Nets mesh contains complete normals and colors");
    check(mesh.vertexCount() > 100 && mesh.triangleCount() > 100, "Sphere Surface Nets mesh contains substantial shared geometry");
    check(!mesh.hasDegenerateTriangles(1.0e-10), "Sphere Surface Nets mesh contains no degenerate triangles");

    double maximumSurfaceError = 0.0;
    double minimumNormalAlignment = 1.0;
    bool allNormalsOutward = true;

    for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertices().size(); ++vertexIndex)
    {
        const MyMath::Vector3 position = vertexPosition(mesh.vertices()[vertexIndex]);
        const double length = position.length();
        maximumSurfaceError = (std::max)(maximumSurfaceError, std::fabs(length - radius));

        MyMath::Vector3 expectedNormal = position;
        MyMath::Vector3 actualNormal = vertexNormal(mesh.vertices()[vertexIndex]);
        const bool expectedOk = expectedNormal.normalize();
        const bool actualOk = actualNormal.normalize();

        if (!expectedOk || !actualOk)
        {
            allNormalsOutward = false;
            minimumNormalAlignment = -1.0;
            break;
        }

        const double alignment = MyMath::Vector3::dot(expectedNormal, actualNormal);
        minimumNormalAlignment = (std::min)(minimumNormalAlignment, alignment);
        allNormalsOutward = allNormalsOutward && alignment > 0.0;
    }

    check(maximumSurfaceError < grid.minimumCellEdgeLength() * 0.8, "Sphere Surface Nets vertices remain close to analytic zero surface");
    check(allNormalsOutward, "Sphere Surface Nets vertex normals point outward");
    check(minimumNormalAlignment > 0.97, "Sphere trilinear TSDF normals closely match analytic sphere normals");

    bool allTriangleWindingOutward = true;
    const std::vector<std::uint32_t>& indices = mesh.indices();

    for (std::size_t indexPosition = 0; indexPosition < indices.size(); indexPosition += 3)
    {
        const MyMath::Vector3 point0 = vertexPosition(mesh.vertices()[indices[indexPosition]]);
        const MyMath::Vector3 point1 = vertexPosition(mesh.vertices()[indices[indexPosition + 1]]);
        const MyMath::Vector3 point2 = vertexPosition(mesh.vertices()[indices[indexPosition + 2]]);
        const MyMath::Vector3 geometricNormal = MyMath::Vector3::cross(point1 - point0, point2 - point0);
        const MyMath::Vector3 center = (point0 + point1 + point2) / 3.0;

        if (MyMath::Vector3::dot(geometricNormal, center) <= 0.0)
        {
            allTriangleWindingOutward = false;
            break;
        }
    }

    check(allTriangleWindingOutward, "Sphere triangle winding consistently points outward");
}

/// 标准与快速拓扑/符号/采样路径一致

void testEquivalentPaths()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelShape voxels = makeSphereVoxels(1.4, grid, 0.75f);

    MyVoxel::Meshing::VolumeMeshingOptions fastOptions;
    fastOptions.color = MyVoxel::Display_Color(0.6, 0.7, 0.9, 1.0);

    MyVoxel::Meshing::VolumeMeshingOptions standardOptions = fastOptions;
    standardOptions.topologyMode = MyVoxel::Meshing::VolumeTopologyMode::StandardDynamic;
    standardOptions.samplingMode = MyVoxel::Meshing::VolumeSamplingMode::StandardOnDemand;
    standardOptions.signMode = MyVoxel::Meshing::VolumeSignMode::StandardDirect;

    const MyVoxel::Mesh fastMesh = MyVoxel::Meshing::VolumeMesher::build(voxels, fastOptions);
    const MyVoxel::Mesh standardMesh = MyVoxel::Meshing::VolumeMesher::build(voxels, standardOptions);

    check(fastMesh.isRenderable() && standardMesh.isRenderable(), "Fast and standard Surface Nets paths both produce renderable meshes");
    check(sameMeshGeometry(fastMesh, standardMesh, 1.0e-6), "Fast lookup/precomputed path matches independent standard Surface Nets path");
}

/// 显式范围和工作区复用

void testWorkspaceReuse()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelShape voxels = makeSphereVoxels(1.35, grid, 0.75f);
    const MyVoxel::VoxelCellRange range = MyVoxel::Meshing::VolumeMesher::defaultSampleRange(voxels);
    MyVoxel::Meshing::VolumeMeshingWorkspace workspace(range);
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = MyVoxel::Display_Color(0.75, 0.65, 0.4, 1.0);

    const MyVoxel::Mesh first = MyVoxel::Meshing::VolumeMesher::buildWithWorkspace(voxels, workspace, options);

    check(workspace.isValid(), "Surface Nets workspace remains valid after first build");
    check(workspace.hasCompleteSampleSigns(), "Surface Nets workspace precomputes every sample sign");
    check(workspace.hasCompleteCellSignMasks(), "Surface Nets workspace precomputes every cell sign mask");
    check(workspace.activeCellCount() > 0, "Surface Nets workspace stores active mixed-sign cells");
    check(workspace.computedGradientCount() == 0, "Uniform Surface Nets no longer maintains obsolete gradient cache");

    const std::size_t firstActiveCellCount = workspace.activeCellCount();
    const MyVoxel::Mesh second = MyVoxel::Meshing::VolumeMesher::buildWithWorkspace(voxels, workspace, options);

    check(second.isRenderable(), "Reused Surface Nets workspace produces renderable mesh");
    check(workspace.activeCellCount() == firstActiveCellCount, "Workspace clear and rebuild preserves active cell count");
    check(sameMeshGeometry(first, second, 1.0e-6), "Reused Surface Nets workspace reproduces identical mesh geometry");

    const MyVoxel::Mesh explicitRangeMesh = MyVoxel::Meshing::VolumeMesher::build(voxels, range, options);
    check(sameMeshGeometry(first, explicitRangeMesh, 1.0e-6), "Explicit default sample range matches workspace build");
}

/// Box平面位置和法线

void testBoxSurface()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyVoxel::VoxelShape reference(grid, 0.75f);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeBox(2.0, 2.0, 2.0), reference);
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = MyVoxel::Display_Color(0.7, 0.8, 0.7, 1.0);

    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);

    check(mesh.isRenderable(), "Box TSDF produces renderable Surface Nets mesh");
    check(!mesh.hasDegenerateTriangles(1.0e-10), "Box Surface Nets mesh contains no degenerate triangles");

    double maximumPlaneError = 0.0;
    double minimumNormalLength = 1.0;
    double maximumNormalLength = 0.0;
    double minimumOutwardDot = (std::numeric_limits<double>::max)();
    double minimumAnalyticGradientAlignment = 1.0;
    const double gradientStep = grid.minimumCellEdgeLength() * 1.0e-3; // 解析Box SDF数值梯度使用最高层采样间距的千分之一，兼顾棱边/角点对称导数和双精度稳定性。

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyMath::Vector3 position = vertexPosition(mesh.vertices()[index]);
        const double distances[3] =
        {
            std::fabs(std::fabs(position.x()) - 1.0),
            std::fabs(std::fabs(position.y()) - 1.0),
            std::fabs(std::fabs(position.z()) - 1.0)
        };
        maximumPlaneError = (std::max)(maximumPlaneError, (std::min)(distances[0], (std::min)(distances[1], distances[2])));

        MyMath::Vector3 normal = vertexNormal(mesh.vertices()[index]);
        const double normalLength = normal.length();
        minimumNormalLength = (std::min)(minimumNormalLength, normalLength);
        maximumNormalLength = (std::max)(maximumNormalLength, normalLength);
        minimumOutwardDot = (std::min)(minimumOutwardDot, MyMath::Vector3::dot(normal, position));

        const auto boxDistance = [](const MyMath::Vector3& point) -> double
        {
            const double qx = std::fabs(point.x()) - 1.0;
            const double qy = std::fabs(point.y()) - 1.0;
            const double qz = std::fabs(point.z()) - 1.0;
            const double ox = (std::max)(qx, 0.0);
            const double oy = (std::max)(qy, 0.0);
            const double oz = (std::max)(qz, 0.0);
            const double outside = std::sqrt(ox * ox + oy * oy + oz * oz);
            const double inside = (std::min)((std::max)(qx, (std::max)(qy, qz)), 0.0);
            return outside + inside;
        };

        const MyMath::Vector3 offsetX(gradientStep, 0.0, 0.0);
        const MyMath::Vector3 offsetY(0.0, gradientStep, 0.0);
        const MyMath::Vector3 offsetZ(0.0, 0.0, gradientStep);
        MyMath::Vector3 expectedNormal(
            boxDistance(position + offsetX) - boxDistance(position - offsetX),
            boxDistance(position + offsetY) - boxDistance(position - offsetY),
            boxDistance(position + offsetZ) - boxDistance(position - offsetZ));

        if (normal.normalize() && expectedNormal.normalize())
        {
            minimumAnalyticGradientAlignment = (std::min)(minimumAnalyticGradientAlignment, MyMath::Vector3::dot(normal, expectedNormal));
        }
        else
        {
            minimumAnalyticGradientAlignment = -1.0;
        }
    }

    check(maximumPlaneError < grid.minimumCellEdgeLength() * 0.8, "Box Surface Nets vertices stay close to analytic planes");
    check(minimumNormalLength > 0.999 && maximumNormalLength < 1.001, "Box TSDF vertex normals remain normalized");
    check(minimumOutwardDot > 0.0, "Box TSDF vertex normals consistently point outward");
    check(minimumAnalyticGradientAlignment > 0.90, "Box smooth TSDF normals follow analytic Box SDF gradient including edges and corners");
}

}

int main()
{
    std::cout << "MyVoxel direct TSDF Uniform Surface Nets regression test" << std::endl << std::endl;

    testEmptyShape();
    testSphereSurface();
    testEquivalentPaths();
    testWorkspaceReuse();
    testBoxSurface();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}