#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"

namespace
{

const double PositionTolerance = 1.0e-6; // MeshVertex存储后的解析Feature位置比较容差。
const double DegenerateTolerance = 1.0e-10; // 约束后检查三角形几何退化使用的双倍面积容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 判断double是否位于测试容差内。
bool nearValue(double first, double second)
{
    return std::fabs(first - second) <= PositionTolerance;
}

// 返回MeshVertex对应的三维位置。
MyMath::Vector3 vertexPosition(const MyVoxel::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 返回顶点落在轴对齐Box解析边界平面上的坐标数量。
unsigned int boxBoundaryAxisCount(const MyVoxel::MeshVertex& vertex, double halfSize)
{
    unsigned int count = 0;
    if (nearValue(std::fabs(static_cast<double>(vertex.x)), halfSize)){++count;}
    if (nearValue(std::fabs(static_cast<double>(vertex.y)), halfSize)){++count;}
    if (nearValue(std::fabs(static_cast<double>(vertex.z)), halfSize)){++count;}
    return count;
}

// 返回当前Mesh精确命中的Box八个角点位掩码。
unsigned int boxCornerMask(const MyVoxel::Mesh& mesh, double halfSize)
{
    unsigned int mask = 0;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyVoxel::MeshVertex& vertex = mesh.vertices()[index];

        if (boxBoundaryAxisCount(vertex, halfSize) != 3)
        {
            continue;
        }

        const unsigned int x = static_cast<double>(vertex.x) > 0.0 ? 1U : 0U;
        const unsigned int y = static_cast<double>(vertex.y) > 0.0 ? 1U : 0U;
        const unsigned int z = static_cast<double>(vertex.z) > 0.0 ? 1U : 0U;
        mask |= 1U << (x | (y << 1) | (z << 2));
    }

    return mask;
}

// 返回至少落在Box两张解析边界平面上的Feature棱顶点数量。
std::size_t boxFeatureEdgeVertexCount(const MyVoxel::Mesh& mesh, double halfSize)
{
    std::size_t count = 0;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        if (boxBoundaryAxisCount(mesh.vertices()[index], halfSize) >= 2)
        {
            ++count;
        }
    }

    return count;
}

// 判断两个Mesh是否具有完全相同的顶点位置和拓扑索引。
bool sameMeshGeometry(const MyVoxel::Mesh& first, const MyVoxel::Mesh& second)
{
    if (first.vertexCount() != second.vertexCount() || first.indices() != second.indices())
    {
        return false;
    }

    for (std::size_t index = 0; index < first.vertices().size(); ++index)
    {
        const MyVoxel::MeshVertex& a = first.vertices()[index];
        const MyVoxel::MeshVertex& b = second.vertices()[index];

        if (a.x != b.x || a.y != b.y || a.z != b.z)
        {
            return false;
        }
    }

    return true;
}

// 建立当前F4统一测试参考体素空间，最高层采样间距h=0.125。
MyVoxel::VoxelShape makeReference()
{
    return MyVoxel::VoxelShape(MyVoxel::VoxelGrid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5)), 0.75f);
}

// 返回现有TSDF QEF模式。
MyVoxel::Meshing::VolumeMeshingOptions featureSensitiveOptions()
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.mode = MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets;
    options.featureAngleDegrees = 30.0;
    return options;
}

// 返回显式Feature优先、QEF后备的F4模式。
MyVoxel::Meshing::VolumeMeshingOptions featureConstrainedOptions()
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.mode = MyVoxel::Meshing::VolumeMeshingMode::FeatureConstrainedSurfaceNets;
    options.featureAngleDegrees = 30.0;
    options.featureSnapDistanceScale = 0.75;
    return options;
}

void testBox()
{
    const double size = 3.0;
    const double halfSize = size * 0.5;
    const MyVoxel::VoxelShape reference = makeReference();
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeBox(size, size, size);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(shape, reference);
    const MyVoxel::Mesh sensitive = MyVoxel::Meshing::VolumeMesher::build(voxels, featureSensitiveOptions());
    const MyVoxel::Mesh constrained = MyVoxel::Meshing::VolumeMesher::build(voxels, featureConstrainedOptions());

    check(voxels.hasCompleteFeatures() && voxels.features().vertexCount() == 8 && voxels.features().edgeCount() == 12,
          "Box precondition contains complete 8-vertex 12-edge FeatureSet");
    check(constrained.isRenderable(), "Feature-constrained Box mesh renderable");
    check(constrained.vertexCount() == sensitive.vertexCount() && constrained.triangleCount() == sensitive.triangleCount(),
          "Explicit Box Feature constraints preserve Surface Nets topology counts");
    check(!constrained.hasDegenerateTriangles(DegenerateTolerance), "Explicit Box Feature constraints do not create degenerate triangles");
    check(boxCornerMask(constrained, halfSize) == 0xFFU, "Feature-constrained Box mesh contains all eight exact analytic corners");

    const std::size_t sensitiveEdgeVertices = boxFeatureEdgeVertexCount(sensitive, halfSize);
    const std::size_t constrainedEdgeVertices = boxFeatureEdgeVertexCount(constrained, halfSize);
    check(constrainedEdgeVertices >= 12, "Feature-constrained Box creates explicit vertices on analytic sharp edges");
    check(constrainedEdgeVertices > sensitiveEdgeVertices, "Explicit Box Feature constraints increase exact sharp-edge adherence");
    check(constrained.localBounds().isEqualTo(shape.topology().geometry().localBounds(), PositionTolerance),
          "Feature-constrained Box mesh bounds match exact analytic Box bounds");
}

void testCylinder()
{
    const double radius = 1.25;
    const double height = 3.0;
    const double halfHeight = height * 0.5;
    const MyVoxel::VoxelShape reference = makeReference();
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeCylinder(radius, height);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(shape, reference);
    const MyVoxel::Mesh sensitive = MyVoxel::Meshing::VolumeMesher::build(voxels, featureSensitiveOptions());
    const MyVoxel::Mesh constrained = MyVoxel::Meshing::VolumeMesher::build(voxels, featureConstrainedOptions());

    check(voxels.hasCompleteFeatures() && voxels.features().vertexCount() == 2 && voxels.features().edgeCount() == 2,
          "Cylinder precondition contains two circular Features and two seam vertices");
    check(constrained.isRenderable(), "Feature-constrained Cylinder mesh renderable");
    check(constrained.vertexCount() == sensitive.vertexCount() && constrained.triangleCount() == sensitive.triangleCount(),
          "Cylinder explicit Feature constraints preserve Surface Nets topology counts");
    check(!constrained.hasDegenerateTriangles(DegenerateTolerance), "Cylinder seam vertices do not collapse neighboring cells or create degenerates");

    std::size_t bottomRimCount = 0;
    std::size_t topRimCount = 0;

    for (std::size_t index = 0; index < constrained.vertices().size(); ++index)
    {
        const MyVoxel::MeshVertex& vertex = constrained.vertices()[index];
        const double x = static_cast<double>(vertex.x);
        const double y = static_cast<double>(vertex.y);
        const double z = static_cast<double>(vertex.z);
        const double radial = std::sqrt(x * x + y * y);

        if (!nearValue(radial, radius))
        {
            continue;
        }

        if (nearValue(z, -halfHeight)){++bottomRimCount;}
        if (nearValue(z, halfHeight)){++topRimCount;}
    }

    check(bottomRimCount > 4 && topRimCount > 4, "Feature-constrained Cylinder vertices lie on both exact analytic rim circles");
}

void testSphereFallback()
{
    const MyVoxel::VoxelShape reference = makeReference();
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(1.6), reference);
    const MyVoxel::Mesh sensitive = MyVoxel::Meshing::VolumeMesher::build(voxels, featureSensitiveOptions());
    const MyVoxel::Mesh constrained = MyVoxel::Meshing::VolumeMesher::build(voxels, featureConstrainedOptions());

    check(voxels.hasCompleteFeatures() && voxels.features().isEmpty(), "Sphere has complete but intentionally empty FeatureSet");
    check(sameMeshGeometry(sensitive, constrained), "Complete empty Sphere FeatureSet preserves existing FeatureSensitive geometry exactly");
}

void testUnavailableFallback()
{
    const MyVoxel::VoxelShape reference = makeReference();
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeConeFrustum(1.8, 0.9, 3.0), reference);
    const MyVoxel::Mesh sensitive = MyVoxel::Meshing::VolumeMesher::build(voxels, featureSensitiveOptions());
    const MyVoxel::Mesh constrained = MyVoxel::Meshing::VolumeMesher::build(voxels, featureConstrainedOptions());

    check(!voxels.hasCompleteFeatures() && voxels.features().isEmpty(), "Unsupported ConeFrustum carries unavailable Feature state");
    check(sameMeshGeometry(sensitive, constrained), "Unavailable Feature state falls back to existing FeatureSensitive geometry exactly");
}

}

int main()
{
    std::cout << "MyVoxel explicit Feature-constrained Surface Nets F4 test" << std::endl << std::endl;
    testBox();
    testCylinder();
    testSphereFallback();
    testUnavailableFallback();
    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}