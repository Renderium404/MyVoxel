#include <cstddef>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyMath/Matrix4.h"

#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 创建指定索引和层级的体素地址。
MyVoxel::VoxelCellAddress makeAddress(MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z, MyVoxel::VoxelLevel level)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), level);
}

// 测试完整覆盖单个根体素的方盒能够直接压缩为材料根节点。
bool testFullRootBox()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(1.0, 1.0, 1.0);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 3);
    MyVoxel::Modeling::VoxelizationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(box, grid, statistics);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    const bool rootCountPassed = result.rootCount() == 1;
    const bool rootStatePassed = result.forest().state(root) == MyVoxel::VoxelState::Material;
    const bool candidateCountPassed = statistics.rootCandidateCount == 1;
    const bool createdRootCountPassed = statistics.createdRootCount == 1;
    const bool insideCountPassed = statistics.insideCellCount == 1;
    const bool splitCountPassed = statistics.splitCount == 0;
    const bool centerSampleCountPassed = statistics.centerSampleCount == 0;

    if (!rootCountPassed ||
        !rootStatePassed ||
        !candidateCountPassed ||
        !createdRootCountPassed ||
        !insideCountPassed ||
        !splitCountPassed ||
        !centerSampleCountPassed)
    {
        std::cout << "  rootCount: " << result.rootCount() << std::endl;
        std::cout << "  rootState: " << static_cast<int>(result.forest().state(root)) << std::endl;
        std::cout << "  rootCandidateCount: " << statistics.rootCandidateCount << std::endl;
        std::cout << "  createdRootCount: " << statistics.createdRootCount << std::endl;
        std::cout << "  visitedCellCount: " << statistics.visitedCellCount << std::endl;
        std::cout << "  outsideCellCount: " << statistics.outsideCellCount << std::endl;
        std::cout << "  insideCellCount: " << statistics.insideCellCount << std::endl;
        std::cout << "  intersectingCellCount: " << statistics.intersectingCellCount << std::endl;
        std::cout << "  centerSampleCount: " << statistics.centerSampleCount << std::endl;
        std::cout << "  splitCount: " << statistics.splitCount << std::endl;
        std::cout << "  mergeSuccessCount: " << statistics.mergeSuccessCount << std::endl;
    }

    const bool passed =
        rootCountPassed &&
        rootStatePassed &&
        candidateCountPassed &&
        createdRootCountPassed &&
        insideCountPassed &&
        splitCountPassed &&
        centerSampleCountPassed;

    return check(passed, "ShapeVoxelization full root box");
}

// 测试局部方盒按最高层级生成材料和空体素。
bool testPartialRootBox()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(0.5, 0.5, 0.5);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 2);
    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(box, grid);

    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress materialCell = makeAddress(1, 1, 1, 2);
    const MyVoxel::VoxelCellAddress emptyCell = makeAddress(0, 0, 0, 2);

    std::size_t materialCellCount = 0;

    result.forest().forEachMaterialCell(
        [&materialCellCount](const MyVoxel::VoxelCellAddress&)
        {
            ++materialCellCount;
        });

    const bool passed =
        result.rootCount() == 1 &&
        result.forest().state(root) == MyVoxel::VoxelState::Subdivided &&
        result.forest().state(materialCell) == MyVoxel::VoxelState::Material &&
        result.forest().state(emptyCell) == MyVoxel::VoxelState::Empty &&
        materialCellCount == 8;

    return check(passed, "ShapeVoxelization partial root box");
}

// 测试跨越两个根体素的方盒。
bool testMultipleRoots()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(2.0, 1.0, 1.0);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-1.0, -0.5, -0.5), 1.0, 2);
    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(box, grid);

    const MyVoxel::VoxelCellAddress firstRoot = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress secondRoot = makeAddress(1, 0, 0, 0);

    const bool passed =
        result.rootCount() == 2 &&
        result.forest().state(firstRoot) == MyVoxel::VoxelState::Material &&
        result.forest().state(secondRoot) == MyVoxel::VoxelState::Material;

    return check(passed, "ShapeVoxelization multiple roots");
}

// 测试球体体素化会生成材料和空体素的混合层级结构。
bool testSphere()
{
    const MyVoxel::Geometry::Shape sphere = MyVoxel::Modeling::makeSphere(0.5);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 3);
    MyVoxel::Modeling::VoxelizationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(sphere, grid, statistics);
    const MyVoxel::VoxelCellAddress centerCell = makeAddress(3, 3, 3, 3);
    const MyVoxel::VoxelCellAddress cornerCell = makeAddress(0, 0, 0, 3);

    const bool passed =
        result.rootCount() == 1 &&
        result.forest().state(makeAddress(0, 0, 0, 0)) == MyVoxel::VoxelState::Subdivided &&
        result.forest().state(centerCell) == MyVoxel::VoxelState::Material &&
        result.forest().state(cornerCell) == MyVoxel::VoxelState::Empty &&
        statistics.intersectingCellCount > 0 &&
        statistics.centerSampleCount > 0 &&
        statistics.splitCount > 0;

    return check(passed, "ShapeVoxelization sphere");
}

// 测试平移后的VoxelGrid仍使用统一空间映射。
bool testTranslatedGrid()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(1.0, 1.0, 1.0);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-10.5, 19.5, 29.5), 1.0, 2);
    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(box, grid);

    const MyVoxel::VoxelCellAddress expectedRoot = makeAddress(10, -20, -30, 0);

    const bool passed =
        result.rootCount() == 1 &&
        result.forest().state(expectedRoot) == MyVoxel::VoxelState::Material &&
        MyMath::Vector3(-0.5, -0.5, -0.5).isEqualTo(result.grid().cellBounds(expectedRoot).minimum());
    return check(passed, "ShapeVoxelization translated grid");
}

// 测试统计数据重置和累加。
bool testStatistics()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(1.0, 1.0, 1.0);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 1);
    MyVoxel::Modeling::VoxelizationStatistics first;
    MyVoxel::Modeling::VoxelizationStatistics second;

    MyVoxel::Modeling::voxelize(box, grid, first);
    MyVoxel::Modeling::voxelize(box, grid, second);

    const std::uint64_t originalCandidateCount = first.rootCandidateCount;
    const std::uint64_t originalCreatedCount = first.createdRootCount;

    first.accumulate(second);

    const bool accumulatedPassed =
        first.rootCandidateCount == originalCandidateCount * 2 &&
        first.createdRootCount == originalCreatedCount * 2;

    first.reset();

    const bool resetPassed =
        first.rootCandidateCount == 0 &&
        first.createdRootCount == 0 &&
        first.visitedCellCount == 0 &&
        first.outsideCellCount == 0 &&
        first.insideCellCount == 0 &&
        first.intersectingCellCount == 0 &&
        first.centerSampleCount == 0 &&
        first.splitCount == 0 &&
        first.mergeSuccessCount == 0;

    return check(accumulatedPassed && resetPassed, "ShapeVoxelization statistics");
}


// 测试ShapeInstance体素化保持局部体素结构并传递实例变换。
bool testShapeInstanceTransform()
{
    const MyVoxel::Geometry::Shape box = MyVoxel::Modeling::makeBox(1.0, 1.0, 1.0);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 3);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 3) = 10.0;
    transform(1, 3) = -4.0;
    transform(2, 3) = 7.5;

    const MyVoxel::Geometry::ShapeInstance instance(box, transform);
    MyVoxel::Modeling::VoxelizationStatistics statistics;

    const MyVoxel::VoxelShape result = MyVoxel::Modeling::voxelize(instance, grid, statistics);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    const bool passed =
        result.rootCount() == 1 &&
        result.forest().state(root) == MyVoxel::VoxelState::Material &&
        result.transform().isEqualTo(transform) &&
        result.grid().cellBounds(root).minimum().isEqualTo(MyMath::Vector3(-0.5, -0.5, -0.5)) &&
        result.grid().cellBounds(root).maximum().isEqualTo(MyMath::Vector3(0.5, 0.5, 0.5)) &&
        statistics.rootCandidateCount == 1 &&
        statistics.createdRootCount == 1 &&
        statistics.insideCellCount == 1 &&
        statistics.splitCount == 0;

    return check(passed, "ShapeVoxelization instance transform");
}


// 测试不同实例变换不会改变Shape局部体素结构。
bool testInstanceTransformDoesNotResample()
{
    const MyVoxel::Geometry::Shape sphere = MyVoxel::Modeling::makeSphere(0.5);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-0.5, -0.5, -0.5), 1.0, 3);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 0) = 0.0;
    transform(0, 1) = -1.0;
    transform(1, 0) = 1.0;
    transform(1, 1) = 0.0;
    transform(0, 3) = 5.0;
    transform(1, 3) = 6.0;
    transform(2, 3) = 7.0;

    const MyVoxel::Geometry::ShapeInstance instance(sphere, transform);
    const MyVoxel::VoxelShape localResult = MyVoxel::Modeling::voxelize(sphere, grid);
    const MyVoxel::VoxelShape instanceResult = MyVoxel::Modeling::voxelize(instance, grid);

    std::size_t localMaterialCount = 0;
    std::size_t instanceMaterialCount = 0;

    localResult.forest().forEachMaterialCell(
        [&localMaterialCount](const MyVoxel::VoxelCellAddress&)
        {
            ++localMaterialCount;
        });

    instanceResult.forest().forEachMaterialCell(
        [&instanceMaterialCount](const MyVoxel::VoxelCellAddress&)
        {
            ++instanceMaterialCount;
        });

    const bool passed =
        localResult.rootCount() == instanceResult.rootCount() &&
        localMaterialCount == instanceMaterialCount &&
        localResult.transform().isIdentity() &&
        instanceResult.transform().isEqualTo(transform);

    return check(passed, "ShapeVoxelization instance no resampling");
}




}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

const bool results[] =
{
    testFullRootBox(),
    testPartialRootBox(),
    testMultipleRoots(),
    testSphere(),
    testTranslatedGrid(),
    testStatistics(),
    testShapeInstanceTransform(),
    testInstanceTransformDoesNotResample()
};

    const std::size_t testCount = sizeof(results) / sizeof(results[0]);

    for (std::size_t testIndex = 0; testIndex < testCount; ++testIndex)
    {
        if (results[testIndex])
        {
            ++passedCount;
        }
        else
        {
            ++failedCount;
        }
    }

    std::cout << std::endl;
    std::cout << "Shape voxelization tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}