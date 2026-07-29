#include <cmath>
#include <cstddef>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace
{

const double TestEpsilon = 1.0e-12; // 体素网格测试使用的浮点比较误差。

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 判断两个标量是否近似相等。
bool equalValue(double first, double second)
{
    return std::fabs(first - second) <= TestEpsilon;
}

// 判断两个三维数据是否近似相等。
bool equalVector(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.isEqualTo(second, TestEpsilon);
}

// 判断体素索引是否与指定值相等。
bool equalIndex(const MyVoxel::VoxelCellIndex& index, MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z)
{
    return index == MyVoxel::VoxelCellIndex(x, y, z);
}

// 测试体素索引和地址比较。
bool testAddressValues()
{
    const MyVoxel::VoxelCellIndex firstIndex(1, -2, 3);
    const MyVoxel::VoxelCellIndex secondIndex(1, -2, 4);
    const MyVoxel::VoxelCellAddress firstAddress(firstIndex, 2);
    const MyVoxel::VoxelCellAddress secondAddress(firstIndex, 3);

    const bool passed =
        firstIndex == MyVoxel::VoxelCellIndex(1, -2, 3) &&
        firstIndex != secondIndex &&
        firstIndex < secondIndex &&
        firstAddress == MyVoxel::VoxelCellAddress(firstIndex, 2) &&
        firstAddress != secondAddress &&
        firstAddress < secondAddress;

    return check(passed, "Voxel address values");
}

// 测试正索引体素的父子层级关系。
bool testPositiveHierarchy()
{
    const MyVoxel::VoxelCellAddress parent(MyVoxel::VoxelCellIndex(2, 3, 4), 1);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(parent, MyVoxel::VoxelCorner::MaximumXZ);

    const bool passed =
        equalIndex(child.index, 5, 6, 9) &&
        child.level == 2 &&
        MyVoxel::parentCellAddress(child) == parent &&
        MyVoxel::childCornerInParent(child) == MyVoxel::VoxelCorner::MaximumXZ;

    return check(passed, "Voxel positive hierarchy");
}

// 测试负索引体素使用向负无穷取整的父子层级关系。
bool testNegativeHierarchy()
{
    const MyVoxel::VoxelCellAddress child(MyVoxel::VoxelCellIndex(-1, -2, -3), 2);
    const MyVoxel::VoxelCellAddress parent = MyVoxel::parentCellAddress(child);
    const MyVoxel::VoxelCellAddress root = MyVoxel::rootCellAddress(child);

    const bool passed =
        equalIndex(parent.index, -1, -1, -2) &&
        parent.level == 1 &&
        MyVoxel::childCornerInParent(child) == MyVoxel::VoxelCorner::MaximumXZ &&
        MyVoxel::childCellAddress(parent, MyVoxel::VoxelCorner::MaximumXZ) == child &&
        equalIndex(root.index, -1, -1, -1) &&
        root.level == MyVoxel::BaseVoxelLevel;

    return check(passed, "Voxel negative hierarchy");
}

// 测试指定目标层级的祖先体素计算。
bool testAncestorAddress()
{
    const MyVoxel::VoxelCellAddress address(MyVoxel::VoxelCellIndex(11, -6, 5), 3);
    const MyVoxel::VoxelCellAddress levelTwo = MyVoxel::ancestorCellAddress(address, 2);
    const MyVoxel::VoxelCellAddress levelOne = MyVoxel::ancestorCellAddress(address, 1);
    const MyVoxel::VoxelCellAddress levelZero = MyVoxel::ancestorCellAddress(address, 0);

    const bool passed =
        equalIndex(levelTwo.index, 5, -3, 2) &&
        equalIndex(levelOne.index, 2, -2, 1) &&
        equalIndex(levelZero.index, 1, -1, 0);

    return check(passed, "Voxel ancestor address");
}

// 测试体素索引范围状态、包含关系和各方向数量。
bool testCellRangeValues()
{
    const MyVoxel::VoxelCellRange invalidRange;
    const MyVoxel::VoxelCellRange range(MyVoxel::VoxelCellIndex(-2, 3, 5), MyVoxel::VoxelCellIndex(1, 7, 10), 2);

    const bool passed =
        !invalidRange.isValid() &&
        invalidRange.countX() == 0 &&
        range.isValid() &&
        range.countX() == 4 &&
        range.countY() == 5 &&
        range.countZ() == 6 &&
        range.contains(MyVoxel::VoxelCellIndex(-2, 3, 5)) &&
        range.contains(MyVoxel::VoxelCellIndex(1, 7, 10)) &&
        range.contains(MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(0, 5, 8), 2)) &&
        !range.contains(MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(0, 5, 8), 1)) &&
        !range.contains(MyVoxel::VoxelCellIndex(2, 5, 8));

    return check(passed, "Voxel cell range values");
}

// 测试默认体素网格参数。
bool testDefaultGrid()
{
    const MyVoxel::VoxelGrid grid;

    const bool passed =
        grid.isValid() &&
        equalVector(grid.origin(), MyMath::Vector3(0.0, 0.0, 0.0)) &&
        equalValue(grid.baseCellEdgeLength(), 1.0) &&
        grid.maximumLevel() == MyVoxel::BaseVoxelLevel &&
        equalValue(grid.minimumCellEdgeLength(), 1.0);

    return check(passed, "VoxelGrid default parameters");
}

// 测试不同层级的体素边长。
bool testGridLevelEdgeLengths()
{
    const MyVoxel::VoxelGrid grid(1.0, 5);

    const bool passed =
        equalValue(grid.cellEdgeLength(0), 1.0) &&
        equalValue(grid.cellEdgeLength(1), 0.5) &&
        equalValue(grid.cellEdgeLength(2), 0.25) &&
        equalValue(grid.cellEdgeLength(5), 0.03125) &&
        equalValue(grid.minimumCellEdgeLength(), 0.03125);

    return check(passed, "VoxelGrid level edge lengths");
}

// 测试世界坐标点到体素索引的映射。
bool testPointIndexMapping()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(10.0, 20.0, 30.0), 2.0, 2);

    const bool passed =
        equalIndex(grid.cellIndex(MyMath::Vector3(10.0, 20.0, 30.0)), 0, 0, 0) &&
        equalIndex(grid.cellIndex(MyMath::Vector3(11.999, 21.999, 31.999)), 0, 0, 0) &&
        equalIndex(grid.cellIndex(MyMath::Vector3(12.0, 22.0, 32.0)), 1, 1, 1) &&
        equalIndex(grid.cellIndex(MyMath::Vector3(8.0, 18.0, 28.0)), -1, -1, -1) &&
        equalIndex(grid.cellIndex(MyMath::Vector3(7.999, 17.999, 27.999)), -2, -2, -2) &&
        equalIndex(grid.cellIndex(MyMath::Vector3(10.5, 20.5, 30.5), 2), 1, 1, 1);

    return check(passed, "VoxelGrid point index mapping");
}

// 测试体素地址对应的空间包围盒、中心和角点。
bool testCellSpatialProperties()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(10.0, 20.0, 30.0), 4.0, 3);
    const MyVoxel::VoxelCellAddress address(MyVoxel::VoxelCellIndex(-1, 2, 3), 1);
    const MyVoxel::Bounds3 bounds = grid.cellBounds(address);

    const bool passed =
        equalVector(bounds.minimum(), MyMath::Vector3(8.0, 24.0, 36.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(10.0, 26.0, 38.0)) &&
        equalVector(grid.cellCenter(address), MyMath::Vector3(9.0, 25.0, 37.0)) &&
        equalVector(grid.cellCorner(address, MyVoxel::VoxelCorner::Minimum), MyMath::Vector3(8.0, 24.0, 36.0)) &&
        equalVector(grid.cellCorner(address, MyVoxel::VoxelCorner::MaximumXZ), MyMath::Vector3(10.0, 24.0, 38.0)) &&
        equalVector(grid.cellCorner(address, MyVoxel::VoxelCorner::MaximumXYZ), MyMath::Vector3(10.0, 26.0, 38.0));

    return check(passed, "VoxelGrid cell spatial properties");
}

// 测试轴对齐包围盒到体素索引范围的映射。
bool testBoundsRangeMapping()
{
    const MyVoxel::VoxelGrid grid(1.0, 2);

    const MyVoxel::Bounds3 alignedBounds(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(1.0, 2.0, 3.0));
    const MyVoxel::VoxelCellRange alignedRange = grid.cellRange(alignedBounds);

    const MyVoxel::Bounds3 mixedBounds(MyMath::Vector3(-1.5, -0.5, 0.25), MyMath::Vector3(0.5, 1.5, 1.25));
    const MyVoxel::VoxelCellRange mixedRange = grid.cellRange(mixedBounds);

    const MyVoxel::VoxelCellRange refinedRange = grid.cellRange(alignedBounds, 1);

    const bool passed =
        equalIndex(alignedRange.minimum, 0, 0, 0) &&
        equalIndex(alignedRange.maximum, 0, 1, 2) &&
        alignedRange.countX() == 1 &&
        alignedRange.countY() == 2 &&
        alignedRange.countZ() == 3 &&
        equalIndex(mixedRange.minimum, -2, -1, 0) &&
        equalIndex(mixedRange.maximum, 0, 1, 1) &&
        equalIndex(refinedRange.minimum, 0, 0, 0) &&
        equalIndex(refinedRange.maximum, 1, 3, 5) &&
        refinedRange.level == 1;

    return check(passed, "VoxelGrid bounds range mapping");
}

// 测试平移网格原点后的索引范围映射。
bool testTranslatedGridRange()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(10.0, 20.0, 30.0), 2.0, 1);
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(8.0, 18.0, 28.0), MyMath::Vector3(12.0, 22.0, 32.0));
    const MyVoxel::VoxelCellRange range = grid.cellRange(bounds);

    const bool passed =
        equalIndex(range.minimum, -1, -1, -1) &&
        equalIndex(range.maximum, 0, 0, 0) &&
        range.countX() == 2 &&
        range.countY() == 2 &&
        range.countZ() == 2;

    return check(passed, "VoxelGrid translated range");
}

// 测试体素网格参数比较。
bool testGridEquality()
{
    const MyVoxel::VoxelGrid first(MyMath::Vector3(1.0, 2.0, 3.0), 4.0, 5);
    const MyVoxel::VoxelGrid second(MyMath::Vector3(1.0, 2.0, 3.0), 4.0, 5);
    const MyVoxel::VoxelGrid differentOrigin(MyMath::Vector3(1.1, 2.0, 3.0), 4.0, 5);
    const MyVoxel::VoxelGrid differentEdge(MyMath::Vector3(1.0, 2.0, 3.0), 8.0, 5);
    const MyVoxel::VoxelGrid differentLevel(MyMath::Vector3(1.0, 2.0, 3.0), 4.0, 4);

    const bool passed =
        first.isEqualTo(second, TestEpsilon) &&
        !first.isEqualTo(differentOrigin, TestEpsilon) &&
        !first.isEqualTo(differentEdge, TestEpsilon) &&
        !first.isEqualTo(differentLevel, TestEpsilon);

    return check(passed, "VoxelGrid equality");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testAddressValues(),
        testPositiveHierarchy(),
        testNegativeHierarchy(),
        testAncestorAddress(),
        testCellRangeValues(),
        testDefaultGrid(),
        testGridLevelEdgeLengths(),
        testPointIndexMapping(),
        testCellSpatialProperties(),
        testBoundsRangeMapping(),
        testTranslatedGridRange(),
        testGridEquality()
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
    std::cout << "Voxel grid tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}