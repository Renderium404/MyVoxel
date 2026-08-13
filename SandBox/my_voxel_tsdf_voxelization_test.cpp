#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Modeling/Shape/VoxelizationWorkspace.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const double DistanceTolerance = 2.0e-6; // TSDF最终保存为float，测试允许double到float转换产生的小量误差。
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

// 判断两个距离值是否在测试容差内一致。
bool nearDistance(double first, double second)
{
    return std::fabs(first - second) <= DistanceTolerance;
}

// 将精确有符号距离截断到指定TSDF范围。
double clampDistance(double distance, double backgroundDistance)
{
    if (distance > backgroundDistance)
    {
        return backgroundDistance;
    }
    if (distance < -backgroundDistance)
    {
        return -backgroundDistance;
    }
    return distance;
}

// 比较指定最高层范围内VoxelShape距离与ShapeQuery精确signed distance截断结果。
bool compareTsdfRange(const MyVoxel::VoxelShape& voxels, const MyVoxel::ShapeQuery& query, const MyVoxel::VoxelCellRange& range,
                      std::size_t& comparedCellCount, double& maximumError)
{
    comparedCellCount = 0;
    maximumError = 0.0;
    const MyVoxel::VoxelGrid& grid = voxels.grid();
    const MyVoxel::VoxelLevel level = grid.maximumLevel();
    const double backgroundDistance = static_cast<double>(voxels.backgroundDistance());

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellAddress address(
                    MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                            static_cast<MyVoxel::VoxelIndex>(y),
                                            static_cast<MyVoxel::VoxelIndex>(z)),
                    level);
                const MyMath::Vector3 center = grid.cellCenter(address);
                const double expected = clampDistance(query.signedDistanceToPoint(center), backgroundDistance);
                const double actual = static_cast<double>(voxels.distance(address));
                const double error = std::fabs(actual - expected);

                if (error > maximumError)
                {
                    maximumError = error;
                }

                ++comparedCellCount;

                if (!nearDistance(actual, expected))
                {
                    std::cout << "  mismatch: index=(" << x << ", " << y << ", " << z << ")"
                              << " center=(" << center.x() << ", " << center.y() << ", " << center.z() << ")"
                              << " expected=" << expected << " actual=" << actual << " error=" << error << std::endl;
                    return false;
                }
            }
        }
    }

    return true;
}

// 检查VoxelShape中全部显式根树是否满足TSDF有效性和归一化约束。
bool allRootTreesNormalized(const MyVoxel::VoxelShape& voxels)
{
    bool valid = true;
    const float backgroundDistance = voxels.backgroundDistance();

    voxels.forest().forEachRootCell(
        [&](const MyVoxel::VoxelCellAddress& rootAddress)
        {
            const MyVoxel::VoxelTree* tree = voxels.forest().getTree(rootAddress.index);
            valid = valid && tree && tree->isTsdfValid(backgroundDistance) && tree->isNormalized(backgroundDistance);
        });

    return valid;
}

// 返回最高层指定空间点所属体素地址。
MyVoxel::VoxelCellAddress highestCell(const MyVoxel::VoxelGrid& grid, const MyMath::Vector3& point)
{
    return grid.cellAddress(point, grid.maximumLevel());
}

/// Sphere完整数值回归

void testSphereVoxelization()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4)); // 最高层边长0.25，默认B=0.25。
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createSphere(1.5);
    const MyVoxel::ShapeQuery query(topology);
    MyVoxel::Modeling::VoxelizationStatistics statistics;
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid, statistics);

    check(voxels.isValid(), "Sphere TSDF voxelization is valid");
    check(!voxels.isEmpty() && voxels.rootCount() > 0, "Sphere TSDF voxelization stores explicit roots");
    check(nearDistance(voxels.backgroundDistance(), grid.minimumCellEdgeLength()), "Default voxelization B equals highest-level cell edge");
    check(allRootTreesNormalized(voxels), "Sphere root trees are TSDF-valid and normalized");

    const double comparisonPadding = static_cast<double>(voxels.backgroundDistance()) + grid.minimumCellEdgeLength(); // B外再比较一个最高层体素，验证缺失Root仍返回+B。
    const MyVoxel::VoxelCellRange range = grid.cellRange(query.queryBounds().expanded(comparisonPadding), grid.maximumLevel());
    std::size_t comparedCellCount = 0;
    double maximumError = 0.0;
    const bool equalField = compareTsdfRange(voxels, query, range, comparedCellCount, maximumError);

    check(equalField, "Sphere highest-level TSDF matches exact signed distance");
    check(comparedCellCount > 1000, "Sphere TSDF comparison covers a dense highest-level sample region");
    check(maximumError <= DistanceTolerance, "Sphere TSDF maximum numeric error is within tolerance");

    const MyVoxel::VoxelCellAddress deepInside = highestCell(grid, MyMath::Vector3(0.0, 0.0, 0.0));
    check(nearDistance(voxels.distance(deepInside), -voxels.backgroundDistance()), "Sphere deep interior saturates to -B");

    const MyVoxel::VoxelCellAddress positiveBand = highestCell(grid, MyMath::Vector3(1.625, 0.125, 0.125));
    const double positiveExpected = clampDistance(query.signedDistanceToPoint(grid.cellCenter(positiveBand)), voxels.backgroundDistance());
    check(positiveExpected > 0.0 && positiveExpected < voxels.backgroundDistance(), "Selected sphere sample lies in positive narrow band");
    check(nearDistance(voxels.distance(positiveBand), positiveExpected), "Sphere positive narrow band outside geometry stores real distance");

    const MyVoxel::VoxelCellAddress farOutside = highestCell(grid, MyMath::Vector3(2.5, 0.0, 0.0));
    check(nearDistance(voxels.distance(farOutside), voxels.backgroundDistance()), "Sphere far exterior resolves to +B");

    check(statistics.rootCandidateCount >= statistics.createdRootCount, "Sphere statistics root candidate count covers created roots");
    check(statistics.visitedCellCount == statistics.centerSampleCount, "Sphere statistics visited cells equal center SDF samples");
    check(statistics.outsideCellCount > 0, "Sphere statistics record +B saturated cells");
    check(statistics.insideCellCount > 0, "Sphere statistics record -B saturated cells");
    check(statistics.intersectingCellCount > 0, "Sphere statistics record narrow-band cells");
    check(statistics.splitCount > 0, "Sphere voxelization creates Node subdivisions");
    check(statistics.maskLeafBuildCount > 0, "Sphere voxelization builds MaskLeaf TSDF blocks");
    check(statistics.maskLeafStoredCount > 0, "Sphere voxelization stores non-uniform MaskLeaf blocks");
}

/// Box完整数值回归

void testBoxVoxelization()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4)); // 最高层边长0.25。
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createBox(2.5, 2.0, 1.5);
    const MyVoxel::ShapeQuery query(topology);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid);

    check(voxels.isValid(), "Box TSDF voxelization is valid");
    check(allRootTreesNormalized(voxels), "Box root trees are TSDF-valid and normalized");

    const double comparisonPadding = static_cast<double>(voxels.backgroundDistance()) + grid.minimumCellEdgeLength();
    const MyVoxel::VoxelCellRange range = grid.cellRange(query.queryBounds().expanded(comparisonPadding), grid.maximumLevel());
    std::size_t comparedCellCount = 0;
    double maximumError = 0.0;

    check(compareTsdfRange(voxels, query, range, comparedCellCount, maximumError), "Box highest-level TSDF matches exact signed distance");
    check(maximumError <= DistanceTolerance, "Box TSDF maximum numeric error is within tolerance");
}

/// Cylinder完整数值回归

void testCylinderVoxelization()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createCylinder(1.25, 2.5);
    const MyVoxel::ShapeQuery query(topology);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid);

    check(voxels.isValid(), "Cylinder TSDF voxelization is valid");
    check(allRootTreesNormalized(voxels), "Cylinder root trees are TSDF-valid and normalized");

    const MyVoxel::VoxelCellRange range =
        grid.cellRange(query.queryBounds().expanded(static_cast<double>(voxels.backgroundDistance()) + grid.minimumCellEdgeLength()), grid.maximumLevel());
    std::size_t comparedCellCount = 0;
    double maximumError = 0.0;

    check(compareTsdfRange(voxels, query, range, comparedCellCount, maximumError), "Cylinder highest-level TSDF matches exact signed distance");
    check(maximumError <= DistanceTolerance, "Cylinder TSDF maximum numeric error is within tolerance");
}

/// ConeFrustum完整数值回归

void testConeFrustumVoxelization()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::Topology_Shape topology = MyVoxel::Modeling::createConeFrustum(1.5, 0.75, 2.5);
    const MyVoxel::ShapeQuery query(topology);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid);

    check(voxels.isValid(), "ConeFrustum TSDF voxelization is valid");
    check(allRootTreesNormalized(voxels), "ConeFrustum root trees are TSDF-valid and normalized");

    const MyVoxel::VoxelCellRange range =
        grid.cellRange(query.queryBounds().expanded(static_cast<double>(voxels.backgroundDistance()) + grid.minimumCellEdgeLength()), grid.maximumLevel());
    std::size_t comparedCellCount = 0;
    double maximumError = 0.0;

    check(compareTsdfRange(voxels, query, range, comparedCellCount, maximumError), "ConeFrustum highest-level TSDF matches exact signed distance");
    check(maximumError <= DistanceTolerance, "ConeFrustum TSDF maximum numeric error is within tolerance");
}

/// 一次性结果已经完成bottom-up归一化

void testVoxelizationAlreadyPruned()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(MyVoxel::Modeling::createSphere(1.5), grid);
    const std::size_t rootCountBefore = voxels.rootCount();

    MyVoxel::VoxelShapeSession session = voxels.session(MyVoxel::BaseVoxelLevel);
    const std::size_t changedRootCount = session.prune();

    check(changedRootCount == 0, "Voxelization output requires no additional prune");
    check(voxels.rootCount() == rootCountBefore, "No-op prune keeps voxelization root count");
    check(allRootTreesNormalized(voxels), "No-op prune keeps all roots normalized");
}

/// 对齐体素化完整继承自定义B

void testAlignedCustomBackgroundDistance()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const float backgroundDistance = 0.75f; // 显式使用最高层边长0.25的三倍，验证B不再由Grid隐式重建。
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(1.5);
    const MyVoxel::ShapeQuery query(shape, reference.transform());
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(shape, reference);

    check(voxels.isValid(), "Aligned custom-B voxelization is valid");
    check(nearDistance(voxels.backgroundDistance(), backgroundDistance), "Aligned voxelization inherits reference B");
    check(voxels.grid().isEqualTo(reference.grid(), 0.0), "Aligned voxelization inherits reference grid");
    check(voxels.transform().isEqualTo(reference.transform(), 0.0), "Aligned voxelization inherits reference transform");
    check(allRootTreesNormalized(voxels), "Aligned custom-B root trees are normalized");

    const MyVoxel::VoxelCellRange range =
        grid.cellRange(query.queryBounds().expanded(static_cast<double>(backgroundDistance) + grid.minimumCellEdgeLength()), grid.maximumLevel());
    std::size_t comparedCellCount = 0;
    double maximumError = 0.0;

    check(compareTsdfRange(voxels, query, range, comparedCellCount, maximumError), "Aligned custom-B TSDF matches exact signed distance");
    check(maximumError <= DistanceTolerance, "Aligned custom-B TSDF maximum error is within tolerance");

    const MyVoxel::VoxelCellAddress positiveBand = highestCell(grid, MyMath::Vector3(1.875, 0.125, 0.125));
    const double expected = clampDistance(query.signedDistanceToPoint(grid.cellCenter(positiveBand)), backgroundDistance);
    check(expected > 0.25 && expected < backgroundDistance, "Custom-B test reaches positive band beyond default B");
    check(nearDistance(voxels.distance(positiveBand), expected), "Custom-B voxelization preserves expanded positive narrow band");
}

/// Workspace结果、B继承和根树复用

void testVoxelizationWorkspace()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const float backgroundDistance = 0.75f;
    MyVoxel::VoxelShape reference(grid, backgroundDistance);
    reference.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(3.0, -2.0, 1.0)));

    const MyVoxel::Shape shape = MyVoxel::Modeling::makeSphere(1.5, reference.transform());
    const MyVoxel::ShapeQuery query(shape, reference.transform());
    MyVoxel::Modeling::VoxelizationWorkspace workspace;
    MyVoxel::Modeling::VoxelizationStatistics firstStatistics;

    const MyVoxel::VoxelShape& firstResult = MyVoxel::Modeling::voxelizeAligned(shape, reference, workspace, firstStatistics);
    check(workspace.isInitialized(), "VoxelizationWorkspace initializes on first aligned voxelization");
    check(firstResult.isValid() && !firstResult.isEmpty(), "VoxelizationWorkspace first result is valid and non-empty");
    check(nearDistance(firstResult.backgroundDistance(), backgroundDistance), "VoxelizationWorkspace first result inherits B");
    check(firstResult.grid().isEqualTo(reference.grid(), 0.0), "VoxelizationWorkspace first result inherits grid");
    check(firstResult.transform().isEqualTo(reference.transform(), 0.0), "VoxelizationWorkspace first result inherits transform");

    const MyVoxel::VoxelCellRange range =
        grid.cellRange(query.queryBounds().expanded(static_cast<double>(backgroundDistance)), grid.maximumLevel());
    std::size_t firstComparedCount = 0;
    double firstMaximumError = 0.0;
    check(compareTsdfRange(firstResult, query, range, firstComparedCount, firstMaximumError), "VoxelizationWorkspace first result TSDF is correct");

    const std::size_t retainedCapacityAfterFirst = workspace.retainedStorageCapacityBytes();
    check(retainedCapacityAfterFirst > 0, "VoxelizationWorkspace retains reusable physical storage");

    workspace.resetStatistics();
    MyVoxel::Modeling::VoxelizationStatistics secondStatistics;
    const MyVoxel::VoxelShape& secondResult = MyVoxel::Modeling::voxelizeAligned(shape, reference, workspace, secondStatistics);

    check(secondResult.isValid(), "VoxelizationWorkspace second result is valid");
    check(nearDistance(secondResult.backgroundDistance(), backgroundDistance), "VoxelizationWorkspace reuse preserves B");
    check(workspace.exactRootReuseCount() > 0, "VoxelizationWorkspace reuses at least one same-index root BlockPool");

    std::size_t secondComparedCount = 0;
    double secondMaximumError = 0.0;
    check(compareTsdfRange(secondResult, query, range, secondComparedCount, secondMaximumError), "VoxelizationWorkspace reused result TSDF is correct");
    check(firstComparedCount == secondComparedCount, "VoxelizationWorkspace repeated run compares the same sample count");

    workspace.reset();
    check(workspace.result().isValid() && workspace.result().isEmpty(), "VoxelizationWorkspace reset leaves a valid empty result");
    check(nearDistance(workspace.result().backgroundDistance(), backgroundDistance), "VoxelizationWorkspace reset preserves custom B");
    check(workspace.cachedRootSlotCount() > 0, "VoxelizationWorkspace reset moves reusable roots into cache");

    workspace.release();
    check(!workspace.isInitialized(), "VoxelizationWorkspace release clears initialization state");
    check(workspace.cachedRootSlotCount() == 0 && workspace.retainedStorageCapacityBytes() == 0, "VoxelizationWorkspace release frees retained cache");
}

}

int main()
{
    std::cout << "MyVoxel narrow-band TSDF voxelization regression test" << std::endl << std::endl;

    testSphereVoxelization();
    testBoxVoxelization();
    testCylinderVoxelization();
    testConeFrustumVoxelization();
    testVoxelizationAlreadyPruned();
    testAlignedCustomBackgroundDistance();
    testVoxelizationWorkspace();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}