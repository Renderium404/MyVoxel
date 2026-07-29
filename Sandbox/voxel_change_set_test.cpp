#include <cstddef>
#include <iostream>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelAddress.h"

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 创建指定坐标的第0层根体素索引。
MyVoxel::VoxelCellIndex makeRootIndex(MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z)
{
    return MyVoxel::VoxelCellIndex(x, y, z);
}

// 测试修改集合默认状态。
bool testDefaultState()
{
    const MyVoxel::VoxelChangeSet changes;

    const bool passed =
        !changes.hasChanges() &&
        changes.modifiedRootCount() == 0 &&
        changes.modifiedRootIndices().empty() &&
        !changes.containsModifiedRoot(makeRootIndex(0, 0, 0));

    return check(passed, "VoxelChangeSet default state");
}

// 测试添加单个修改根体素。
bool testAddModifiedRoot()
{
    MyVoxel::VoxelChangeSet changes;
    const MyVoxel::VoxelCellIndex rootIndex = makeRootIndex(2, -3, 5);

    changes.addModifiedRoot(rootIndex);

    const bool passed =
        changes.hasChanges() &&
        changes.modifiedRootCount() == 1 &&
        changes.containsModifiedRoot(rootIndex) &&
        !changes.containsModifiedRoot(makeRootIndex(2, -3, 6));

    return check(passed, "VoxelChangeSet add modified root");
}

// 测试重复添加相同根体素不会产生重复记录。
bool testDuplicateRoot()
{
    MyVoxel::VoxelChangeSet changes;
    const MyVoxel::VoxelCellIndex rootIndex = makeRootIndex(-4, 1, 7);

    changes.addModifiedRoot(rootIndex);
    changes.addModifiedRoot(rootIndex);
    changes.addModifiedRoot(rootIndex);

    const bool passed =
        changes.hasChanges() &&
        changes.modifiedRootCount() == 1 &&
        changes.containsModifiedRoot(rootIndex);

    return check(passed, "VoxelChangeSet duplicate root");
}

// 测试多个正负索引根体素记录。
bool testMultipleRoots()
{
    MyVoxel::VoxelChangeSet changes;
    const MyVoxel::VoxelCellIndex first = makeRootIndex(-10, 0, 3);
    const MyVoxel::VoxelCellIndex second = makeRootIndex(0, 0, 0);
    const MyVoxel::VoxelCellIndex third = makeRootIndex(8, -2, 1);

    changes.addModifiedRoot(third);
    changes.addModifiedRoot(first);
    changes.addModifiedRoot(second);

    const bool passed =
        changes.modifiedRootCount() == 3 &&
        changes.containsModifiedRoot(first) &&
        changes.containsModifiedRoot(second) &&
        changes.containsModifiedRoot(third);

    return check(passed, "VoxelChangeSet multiple roots");
}

// 测试累加修改集合并自动去除重复根体素。
bool testAccumulate()
{
    MyVoxel::VoxelChangeSet first;
    MyVoxel::VoxelChangeSet second;

    const MyVoxel::VoxelCellIndex firstOnly = makeRootIndex(-1, 0, 0);
    const MyVoxel::VoxelCellIndex shared = makeRootIndex(0, 0, 0);
    const MyVoxel::VoxelCellIndex secondOnly = makeRootIndex(1, 0, 0);

    first.addModifiedRoot(firstOnly);
    first.addModifiedRoot(shared);

    second.addModifiedRoot(shared);
    second.addModifiedRoot(secondOnly);

    first.accumulate(second);

    const bool passed =
        first.modifiedRootCount() == 3 &&
        first.containsModifiedRoot(firstOnly) &&
        first.containsModifiedRoot(shared) &&
        first.containsModifiedRoot(secondOnly) &&
        second.modifiedRootCount() == 2;

    return check(passed, "VoxelChangeSet accumulate");
}

// 测试修改集合复制后保持独立。
bool testCopyIndependence()
{
    MyVoxel::VoxelChangeSet original;
    const MyVoxel::VoxelCellIndex originalRoot = makeRootIndex(1, 2, 3);
    const MyVoxel::VoxelCellIndex copyRoot = makeRootIndex(4, 5, 6);

    original.addModifiedRoot(originalRoot);

    MyVoxel::VoxelChangeSet copy = original;
    copy.addModifiedRoot(copyRoot);

    const bool passed =
        original.modifiedRootCount() == 1 &&
        copy.modifiedRootCount() == 2 &&
        original.containsModifiedRoot(originalRoot) &&
        !original.containsModifiedRoot(copyRoot) &&
        copy.containsModifiedRoot(originalRoot) &&
        copy.containsModifiedRoot(copyRoot);

    return check(passed, "VoxelChangeSet copy independence");
}

// 测试根体素索引按照VoxelCellIndex比较规则有序保存。
bool testRootOrdering()
{
    MyVoxel::VoxelChangeSet changes;

    const MyVoxel::VoxelCellIndex first = makeRootIndex(-1, 5, 3);
    const MyVoxel::VoxelCellIndex second = makeRootIndex(0, -2, 7);
    const MyVoxel::VoxelCellIndex third = makeRootIndex(0, 4, -1);

    changes.addModifiedRoot(third);
    changes.addModifiedRoot(first);
    changes.addModifiedRoot(second);

    MyVoxel::VoxelChangeSet::RootIndexSet::const_iterator iterator = changes.modifiedRootIndices().begin();

    const bool firstPassed = iterator != changes.modifiedRootIndices().end() && *iterator == first;

    if (iterator != changes.modifiedRootIndices().end())
    {
        ++iterator;
    }

    const bool secondPassed = iterator != changes.modifiedRootIndices().end() && *iterator == second;

    if (iterator != changes.modifiedRootIndices().end())
    {
        ++iterator;
    }

    const bool thirdPassed = iterator != changes.modifiedRootIndices().end() && *iterator == third;

    return check(firstPassed && secondPassed && thirdPassed, "VoxelChangeSet root ordering");
}

// 测试清空修改集合。
bool testClear()
{
    MyVoxel::VoxelChangeSet changes;
    const MyVoxel::VoxelCellIndex first = makeRootIndex(0, 0, 0);
    const MyVoxel::VoxelCellIndex second = makeRootIndex(1, 2, 3);

    changes.addModifiedRoot(first);
    changes.addModifiedRoot(second);
    changes.clear();

    const bool passed =
        !changes.hasChanges() &&
        changes.modifiedRootCount() == 0 &&
        changes.modifiedRootIndices().empty() &&
        !changes.containsModifiedRoot(first) &&
        !changes.containsModifiedRoot(second);

    return check(passed, "VoxelChangeSet clear");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testAddModifiedRoot(),
        testDuplicateRoot(),
        testMultipleRoots(),
        testAccumulate(),
        testCopyIndependence(),
        testRootOrdering(),
        testClear()
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
    std::cout << "Voxel change set tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}