#include <cstdlib>
#include <iostream>
#include <string>

#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Surface/VoxelFaceExtractor.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"
#include "MyVoxel/Surface/VoxelSurfaceMesher.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
        return;
    }

    ++g_failedCount;
    std::cout << "[FAIL] " << name << std::endl;
}

MyVoxel::VoxelLevel surfaceTrackingLevel(const MyVoxel::VoxelGrid& grid)
{
    const unsigned int maximumLevel =
        static_cast<unsigned int>(grid.maximumLevel());

    // 表面增量缓存以最高层下面两级的4x4x4块作为局部更新单元。
    return maximumLevel >= 2U
        ? static_cast<MyVoxel::VoxelLevel>(maximumLevel - 2U)
        : MyVoxel::BaseVoxelLevel;
}

MyVoxel::VoxelChangeSet setMaterialRoot(
    MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    MyVoxel::VoxelShapeSession session(
        shape,
        surfaceTrackingLevel(shape.grid()));

    session.setTree(
        rootIndex,
        MyVoxel::VoxelTree(MyVoxel::VoxelState::Material));

    return session.takeChanges();
}

std::size_t rootAxisCellCount(const MyVoxel::VoxelGrid& grid)
{
    const unsigned int levelDifference =
        static_cast<unsigned int>(
            grid.maximumLevel() - MyVoxel::BaseVoxelLevel);

    return static_cast<std::size_t>(1) << levelDifference;
}

void testEmptySurface()
{
    const MyVoxel::VoxelGrid grid(
        1.0,
        static_cast<MyVoxel::VoxelLevel>(3));
    const MyVoxel::VoxelShape shape(grid);
    const MyVoxel::Geometry::MeshColor color(0.7, 0.7, 0.7, 1.0);

    const MyVoxel::VoxelFaceSet faces =
        MyVoxel::VoxelFaceExtractor::extract(shape);
    const MyVoxel::Geometry::Mesh mesh =
        MyVoxel::VoxelSurfaceMesher::build(shape, color);

    check(faces.isEmpty(), "empty shape has no surface faces");
    check(mesh.isEmpty(), "empty shape has no surface mesh");
}

void testSingleMaterialRoot()
{
    const MyVoxel::VoxelGrid grid(
        1.0,
        static_cast<MyVoxel::VoxelLevel>(3));
    MyVoxel::VoxelShape shape(grid);
    const MyVoxel::VoxelCellIndex rootIndex(0, 0, 0);
    const MyVoxel::Geometry::MeshColor color(0.8, 0.8, 0.8, 1.0);

    setMaterialRoot(shape, rootIndex);

    const std::size_t axisCellCount =
        rootAxisCellCount(grid);
    const std::size_t expectedFaceCount =
        static_cast<std::size_t>(6) *
        axisCellCount *
        axisCellCount;

    const MyVoxel::VoxelFaceSet faces =
        MyVoxel::VoxelFaceExtractor::extract(shape);
    const MyVoxel::VoxelRootFaceMasks masks =
        MyVoxel::VoxelFaceExtractor::extractRootMasks(
            shape,
            rootIndex);
    const MyVoxel::Geometry::Mesh mesh =
        MyVoxel::VoxelSurfaceMesher::build(shape, color);

    check(faces.size() == expectedFaceCount,
          "single material root unit face count");
    check(masks.faceCount() == expectedFaceCount,
          "single material root mask face count");
    check(mesh.isRenderable(),
          "single material root produces renderable mesh");
    check(mesh.vertexCount() == 24,
          "single material root greedy mesh vertex count");
    check(mesh.triangleCount() == 12,
          "single material root greedy mesh triangle count");
}

void testAdjacentMaterialRootsAndCache()
{
    const MyVoxel::VoxelGrid grid(
        1.0,
        static_cast<MyVoxel::VoxelLevel>(3));
    MyVoxel::VoxelShape shape(grid);
    const MyVoxel::VoxelCellIndex firstRoot(0, 0, 0);
    const MyVoxel::VoxelCellIndex secondRoot(1, 0, 0);
    const MyVoxel::Geometry::MeshColor initialColor(0.6, 0.6, 0.6, 1.0);
    const MyVoxel::Geometry::MeshColor cutColor(0.9, 0.3, 0.2, 1.0);

    setMaterialRoot(shape, firstRoot);

    MyVoxel::VoxelSurfaceCache cache;
    cache.rebuild(shape, initialColor);

    const MyVoxel::VoxelChangeSet changes =
        setMaterialRoot(shape, secondRoot);
    const MyVoxel::VoxelSurfaceCacheUpdate update =
        cache.update(shape, changes, cutColor);

    const std::size_t axisCellCount =
        rootAxisCellCount(grid);
    const std::size_t expectedFaceCount =
        static_cast<std::size_t>(10) *
        axisCellCount *
        axisCellCount;
    const MyVoxel::VoxelFaceSet faces =
        MyVoxel::VoxelFaceExtractor::extract(shape);
    const MyVoxel::Geometry::Mesh mesh =
        cache.combinedMesh();

    check(changes.hasChanges(),
          "session reports adjacent root change");
    check(update.hasSurfaceChanges(),
          "surface cache reports adjacent root surface change");
    check(faces.size() == expectedFaceCount,
          "adjacent roots remove internal shared faces");
    check(cache.faceCount() == expectedFaceCount,
          "surface cache face count matches full extraction");
    check(cache.rootCount() == 2,
          "surface cache contains both exposed roots");
    check(mesh.isRenderable(),
          "adjacent roots cache produces renderable mesh");
    check(mesh.vertexCount() == 40,
          "adjacent roots preserve root-local greedy quads");
    check(mesh.triangleCount() == 20,
          "adjacent roots cache triangle count");
}

}

int main()
{
    std::cout << "MyVoxel surface extraction test" << std::endl;

    testEmptySurface();
    testSingleMaterialRoot();
    testAdjacentMaterialRootsAndCache();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}