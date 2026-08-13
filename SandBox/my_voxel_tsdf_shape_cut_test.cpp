#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"

namespace
{

const double DistanceTolerance = 2.0e-6; // TSDF内部保存float，解析差集对比允许double到float转换误差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果并累计统计。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 判断两个距离是否在测试容差内一致。
bool nearDistance(double first, double second){return std::fabs(first - second) <= DistanceTolerance;}

// 返回标准截断SDF差集参考值max(dObject,clamp(-dTool,-B,+B))。
double expectedDifference(double objectDistance, double toolDistance, double backgroundDistance)
{
    return (std::max)(objectDistance, (std::max)(-backgroundDistance, (std::min)(backgroundDistance, -toolDistance)));
}

// 比较指定最高层范围内实际结果与逐样本解析差集公式。
bool compareDifferenceRange(const MyVoxel::VoxelShape& before, const MyVoxel::VoxelShape& after,
                            const MyVoxel::ShapeQuery& toolQuery, const MyVoxel::VoxelCellRange& range,
                            double& maximumError, std::size_t& changedSampleCount)
{
    maximumError = 0.0;
    changedSampleCount = 0;
    const MyVoxel::VoxelGrid& grid = after.grid();
    const double backgroundDistance = after.backgroundDistance();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellAddress address(
                    MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y), static_cast<MyVoxel::VoxelIndex>(z)),
                    grid.maximumLevel());
                const MyMath::Vector3 center = grid.cellCenter(address);
                const double oldDistance = before.distance(address);
                const double expected = expectedDifference(oldDistance, toolQuery.signedDistanceToPoint(center), backgroundDistance);
                const double actual = after.distance(address);
                const double error = std::fabs(actual - expected);
                maximumError = (std::max)(maximumError, error);
                if (!nearDistance(oldDistance, actual)) ++changedSampleCount;

                if (!nearDistance(actual, expected))
                {
                    std::cout << "  mismatch index=(" << x << "," << y << "," << z << ") expected=" << expected
                              << " actual=" << actual << " old=" << oldDistance << std::endl;
                    return false;
                }
            }

    return true;
}

// 检查全部显式Root保持TSDF有效并已经Value-aware归一化。
bool allRootsNormalized(const MyVoxel::VoxelShape& shape)
{
    bool result = true;
    shape.forest().forEachRootCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            const MyVoxel::VoxelTree* tree = shape.forest().getTree(address.index);
            result = result && tree && tree->isTsdfValid(shape.backgroundDistance()) && tree->isNormalized(shape.backgroundDistance());
        });
    return result;
}

/// 当前Instance::Shape平移工具切削

void testTranslatedInstanceShapeCut()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5)); // h=0.125。
    const float backgroundDistance = 0.75f;
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape objectShape = MyVoxel::Modeling::makeBox(3.0, 3.0, 3.0);
    const MyMath::Matrix4 toolTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(0.75, 0.0, 0.0));
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeSphere(1.0, toolTransform);
    const MyVoxel::VoxelShape before = MyVoxel::Modeling::voxelizeAligned(objectShape, reference);
    MyVoxel::VoxelShape after = before;
    const MyVoxel::ShapeQuery toolQuery(tool, after.transform());

    check(tool.isValid() && tool.localToWorld().isEqualTo(toolTransform, 0.0), "Cut tool uses current Instance::Shape placement");
    check(toolQuery.isValid() && toolQuery.supportsSignedDistance(), "Current ShapeQuery supports translated Instance::Shape signed distance");
    check(MyVoxel::Operation::ShapeCutOperation::subtractInPlace(after, tool), "VoxelShape minus translated Shape reports actual TSDF change");
    check(after.isValid(), "Continuous Shape cut result VoxelShape is valid");
    check(nearDistance(after.backgroundDistance(), backgroundDistance), "Continuous Shape cut preserves object background distance");
    check(after.grid().isEqualTo(before.grid(), 0.0), "Continuous Shape cut preserves object grid");
    check(after.transform().isEqualTo(before.transform(), 0.0), "Continuous Shape cut preserves object transform");
    check(allRootsNormalized(after), "Continuous Shape cut output roots are TSDF-valid and normalized");

    const MyVoxel::VoxelCellRange range =
        grid.cellRange(toolQuery.queryBounds().expanded(static_cast<double>(backgroundDistance) + grid.minimumCellEdgeLength()),
                       grid.maximumLevel());
    double maximumError = 0.0;
    std::size_t changedSampleCount = 0;

    check(compareDifferenceRange(before, after, toolQuery, range, maximumError, changedSampleCount),
          "Continuous Shape cut matches sample-wise TSDF difference formula");
    check(maximumError <= DistanceTolerance, "Continuous Shape cut maximum numeric error is within tolerance");
    check(changedSampleCount > 100, "Continuous Shape cut changes a substantial highest-level sample region");

    const MyVoxel::VoxelCellAddress deepToolCell = grid.cellAddress(MyMath::Vector3(0.75, 0.0, 0.0), grid.maximumLevel());
    check(nearDistance(after.distance(deepToolCell), backgroundDistance), "Deep tool interior saturates cut result to +B");

    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(after);
    check(mesh.isRenderable() && !mesh.isEmpty(), "Cut TSDF remains directly renderable by Surface Nets");
    check(!mesh.hasDegenerateTriangles(1.0e-10), "Cut Surface Nets mesh contains no degenerate triangles");
}

/// ShapeCutOperation返回式和原地式一致

void testReturnedAndInPlace()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelShape reference(grid, 0.75f);
    const MyVoxel::VoxelShape original = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(1.6), reference);
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeCylinder(0.65, 3.0);
    const MyVoxel::VoxelShape returned = MyVoxel::Operation::ShapeCutOperation::subtract(original, tool);
    MyVoxel::VoxelShape inPlace = original;
    const bool changed = MyVoxel::Operation::ShapeCutOperation::subtractInPlace(inPlace, tool);
    const MyVoxel::ShapeQuery query(tool, original.transform());
    const MyVoxel::VoxelCellRange range =
        grid.cellRange(query.queryBounds().expanded(static_cast<double>(original.backgroundDistance())), grid.maximumLevel());
    bool equal = changed;

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z) && equal; ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y) && equal; ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellAddress address(
                    MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y), static_cast<MyVoxel::VoxelIndex>(z)),
                    grid.maximumLevel());
                if (!nearDistance(returned.distance(address), inPlace.distance(address))){equal = false; break;}
            }

    check(changed, "ShapeCutOperation in-place subtraction reports change");
    check(equal, "ShapeCutOperation returned and in-place paths produce identical TSDF samples");

    const MyVoxel::VoxelCellAddress center = grid.cellAddress(MyMath::Vector3(0.0, 0.0, 0.0), grid.maximumLevel());
    check(original.distance(center) < returned.distance(center), "Returned Shape cut leaves original source TSDF unchanged");
}

/// 远离工件的当前Shape实例必须零修改

void testNoOpCut()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelShape reference(grid, 0.75f);
    MyVoxel::VoxelShape object = MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeSphere(1.5), reference);
    const std::size_t rootCount = object.rootCount();
    const MyVoxel::Shape farTool = MyVoxel::Modeling::makeSphere(
        0.5, MyMath::Matrix4::fromTranslation(MyMath::Vector3(20.0, 0.0, 0.0)));

    check(!MyVoxel::Operation::ShapeCutOperation::subtractInPlace(object, farTool), "Far Instance::Shape tool produces no TSDF change");
    check(object.rootCount() == rootCount, "No-op continuous Shape cut preserves root count");
    check(allRootsNormalized(object), "No-op continuous Shape cut keeps roots normalized");
}

/// VoxelShape自身带空间Transform时仍按当前Instance语义切削

void testObjectTransformSpace()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    MyVoxel::VoxelShape reference(grid, 0.75f);
    const MyMath::Matrix4 objectTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -2.0, 1.0));
    reference.setTransform(objectTransform);

    const MyVoxel::Shape objectShape = MyVoxel::Modeling::makeSphere(1.5, objectTransform);
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeSphere(
        0.6, MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -2.0, 1.0)));
    MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(objectShape, reference);
    const MyVoxel::ShapeQuery query(tool, voxels.transform());

    check(query.isValid() && query.supportsSignedDistance(), "ShapeQuery resolves current Shape placement relative to transformed VoxelShape");
    check(query.queryBounds().contains(MyMath::Vector3(0.0, 0.0, 0.0)), "World-space tool center maps to VoxelShape local origin");
    check(MyVoxel::Operation::ShapeCutOperation::subtractInPlace(voxels, tool), "Transformed VoxelShape minus world-space Shape succeeds");

    const MyVoxel::VoxelCellAddress center = grid.cellAddress(MyMath::Vector3(0.0, 0.0, 0.0), grid.maximumLevel());
    check(voxels.distance(center) > 0.0f, "Transformed object cut removes material at local tool center");
    check(voxels.transform().isEqualTo(objectTransform, 0.0), "Shape cut preserves transformed VoxelShape placement");
}

}

int main()
{
    std::cout << "MyVoxel current Instance::Shape TSDF cut regression test" << std::endl << std::endl;

    testTranslatedInstanceShapeCut();
    testReturnedAndInPlace();
    testNoOpCut();
    testObjectTransformSpace();

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}