#include <cstddef>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Core/VoxelShape.h"

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

// 检查两个双精度值是否完全相等。
bool sameValue(double first, double second)
{
    return first == second;
}

// 测试默认体积状态。
bool testDefaultState()
{
    const MyVoxel::VoxelShape volume;
    const MyMath::Matrix4& transform = volume.transform();

    const bool identityTransform =
        sameValue(transform(0, 0), 1.0) &&
        sameValue(transform(1, 1), 1.0) &&
        sameValue(transform(2, 2), 1.0) &&
        sameValue(transform(3, 3), 1.0) &&
        sameValue(transform(0, 3), 0.0) &&
        sameValue(transform(1, 3), 0.0) &&
        sameValue(transform(2, 3), 0.0);

    const bool passed =
        volume.grid().isValid() &&
        volume.isEmpty() &&
        volume.rootCount() == 0 &&
        volume.forest().isEmpty() &&
        transform.isAffine() &&
        identityTransform;

    return check(passed, "VoxelShape default state");
}

// 测试使用指定体素网格创建体积。
bool testGridConstruction()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(10.0, 20.0, 30.0), 2.0, 3);
    const MyVoxel::VoxelShape volume(grid);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::Bounds3 bounds = volume.grid().cellBounds(root);

    const bool passed =
        volume.grid().isValid() &&
        volume.isEmpty() &&
        sameValue(bounds.minimum().x(), 10.0) &&
        sameValue(bounds.minimum().y(), 20.0) &&
        sameValue(bounds.minimum().z(), 30.0) &&
        sameValue(bounds.maximum().x(), 12.0) &&
        sameValue(bounds.maximum().y(), 22.0) &&
        sameValue(bounds.maximum().z(), 32.0);

    return check(passed, "VoxelShape grid construction");
}

// 测试通过可写森林修改体素数据。
bool testForestEditing()
{
    MyVoxel::VoxelShape volume(2.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(2, -1, 4, 0);

    const bool changed = volume.editForest().setState(root, MyVoxel::VoxelState::Material);

    const bool passed =
        changed &&
        !volume.isEmpty() &&
        volume.rootCount() == 1 &&
        volume.forest().state(root) == MyVoxel::VoxelState::Material;

    return check(passed, "VoxelShape forest editing");
}

// 测试体积复制保持共享数据。
bool testSharedCopy()
{
    MyVoxel::VoxelShape original(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    original.editForest().setState(root, MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelShape copy = original;

    const bool passed =
        original.sharesDataWith(copy) &&
        original.rootCount() == 1 &&
        copy.rootCount() == 1 &&
        original.forest().state(root) == MyVoxel::VoxelState::Material &&
        copy.forest().state(root) == MyVoxel::VoxelState::Material;

    return check(passed, "VoxelShape shared copy");
}

// 测试编辑复制体积时执行两级写时复制。
bool testCopyOnWrite()
{
    MyVoxel::VoxelShape original(1.0, 3);
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    original.editForest().setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelShape copy = original;

    const bool initiallyShared = original.sharesDataWith(copy);
    const bool split = copy.editForest().split(root);

    const bool passed =
        initiallyShared &&
        split &&
        !original.sharesDataWith(copy) &&
        original.forest().state(root) == MyVoxel::VoxelState::Material &&
        original.forest().state(child) == MyVoxel::VoxelState::Material &&
        copy.forest().state(root) == MyVoxel::VoxelState::Subdivided &&
        copy.forest().state(child) == MyVoxel::VoxelState::Material;

    return check(passed, "VoxelShape copy-on-write");
}

// 测试修改体积变换不会分离共享体素数据。
bool testIndependentTransform()
{
    MyVoxel::VoxelShape original(1.0, 3);
    MyVoxel::VoxelShape copy = original;

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 3) = 5.0;
    transform(1, 3) = -2.0;
    transform(2, 3) = 7.0;

    copy.setTransform(transform);

    const bool passed =
        original.sharesDataWith(copy) &&
        sameValue(original.transform()(0, 3), 0.0) &&
        sameValue(original.transform()(1, 3), 0.0) &&
        sameValue(original.transform()(2, 3), 0.0) &&
        sameValue(copy.transform()(0, 3), 5.0) &&
        sameValue(copy.transform()(1, 3), -2.0) &&
        sameValue(copy.transform()(2, 3), 7.0);

    return check(passed, "VoxelShape independent transform");
}

// 测试重置空间变换。
bool testResetTransform()
{
    MyVoxel::VoxelShape volume;

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 3) = 3.0;
    transform(1, 3) = 4.0;
    transform(2, 3) = 5.0;

    volume.setTransform(transform);
    volume.resetTransform();

    const bool passed =
        sameValue(volume.transform()(0, 0), 1.0) &&
        sameValue(volume.transform()(1, 1), 1.0) &&
        sameValue(volume.transform()(2, 2), 1.0) &&
        sameValue(volume.transform()(0, 3), 0.0) &&
        sameValue(volume.transform()(1, 3), 0.0) &&
        sameValue(volume.transform()(2, 3), 0.0);

    return check(passed, "VoxelShape reset transform");
}

// 测试清空复制体积时保持原体积不变。
bool testClearCopy()
{
    MyVoxel::VoxelShape original(1.0, 3);
    const MyVoxel::VoxelCellAddress firstRoot = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress secondRoot = makeAddress(1, 0, 0, 0);

    original.editForest().setState(firstRoot, MyVoxel::VoxelState::Material);
    original.editForest().setState(secondRoot, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelShape copy = original;
    copy.clear();

    const bool passed =
        !original.sharesDataWith(copy) &&
        original.rootCount() == 2 &&
        !original.isEmpty() &&
        copy.rootCount() == 0 &&
        copy.isEmpty() &&
        original.forest().state(firstRoot) == MyVoxel::VoxelState::Material &&
        original.forest().state(secondRoot) == MyVoxel::VoxelState::Material;

    return check(passed, "VoxelShape clear copy");
}

// 测试获取可写森林会显式分离共享体素数据。
bool testExplicitEditDetach()
{
    MyVoxel::VoxelShape original(1.0, 3);
    MyVoxel::VoxelShape copy = original;

    const bool initiallyShared = original.sharesDataWith(copy);
    MyVoxel::VoxelForest& editableForest = copy.editForest();

    const bool passed =
        initiallyShared &&
        !original.sharesDataWith(copy) &&
        editableForest.isEmpty() &&
        original.isEmpty() &&
        copy.isEmpty();

    return check(passed, "VoxelShape explicit edit detach");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testGridConstruction(),
        testForestEditing(),
        testSharedCopy(),
        testCopyOnWrite(),
        testIndependentTransform(),
        testResetTransform(),
        testClearCopy(),
        testExplicitEditDetach()
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
    std::cout << "Voxel volume tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}