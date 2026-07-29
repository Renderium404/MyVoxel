#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Query/VoxelForestQuery.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelForestConstAccessor.h"
#include "MyVoxel/Core/Tree/VoxelTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelGrid.h"

namespace
{

int g_passedCount = 0;
int g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[Passed] " << name << std::endl;
        return;
    }

    ++g_failedCount;
    std::cout << "[Failed] " << name << std::endl;
}

// 返回指定数值对应的体素角点。
MyVoxel::VoxelCorner corner(unsigned int value)
{
    return static_cast<MyVoxel::VoxelCorner>(value);
}

// 返回测试使用的第0层根地址。
MyVoxel::VoxelCellAddress rootAddress()
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(0, 0, 0), MyVoxel::BaseVoxelLevel);
}

// 验证生产森林已经切换到Packed节点布局。
bool testProductionForest()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(MyVoxel::childCellAddress(root, corner(3)), MyVoxel::VoxelState::Empty);

    return forest.state(root) == MyVoxel::VoxelState::Subdivided &&
           forest.state(MyVoxel::childCellAddress(root, corner(3))) == MyVoxel::VoxelState::Empty &&
           forest.state(MyVoxel::childCellAddress(root, corner(0))) == MyVoxel::VoxelState::Material;
}

// 验证生产编辑器和只读游标接口。
bool testEditorAndCursor()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();

    forest.setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelTreeEditor editor(forest, root);

    editor.split();
    editor.child(corner(5)).setState(MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelTreeConstCursor cursor(forest, root);

    return editor.state() == MyVoxel::VoxelState::Subdivided &&
           cursor.state() == MyVoxel::VoxelState::Subdivided &&
           cursor.child(corner(5)).state() == MyVoxel::VoxelState::Empty &&
           cursor.child(corner(0)).state() == MyVoxel::VoxelState::Material;
}

// 验证生产访问器状态和缓存统计。
bool testAccessor()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();
    const MyVoxel::VoxelCellAddress child0 = MyVoxel::childCellAddress(root, corner(0));
    const MyVoxel::VoxelCellAddress child1 = MyVoxel::childCellAddress(root, corner(1));

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(child1, MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    const bool statesValid =
        accessor.state(child0) == MyVoxel::VoxelState::Material &&
        accessor.state(child1) == MyVoxel::VoxelState::Empty;

    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    return statesValid &&
           statistics.rootCacheMissCount == 1 &&
           statistics.rootCacheHitCount == 1 &&
           statistics.nodeVisitCount > 0;
}

// 验证生产区域查询器已经切换到Packed递归实现。
bool testQuery()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();

    forest.setState(root, MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(0.0, 0.0, 0.0), 1.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::VoxelForestQuery query(forest, grid);
    MyVoxel::VoxelForestQueryStatistics statistics;

    const MyVoxel::VoxelRegionRelation relation =
        query.classify(
            MyVoxel::Bounds3(
                MyMath::Vector3(0.2, 0.2, 0.2),
                MyMath::Vector3(0.8, 0.8, 0.8)),
            statistics);

    return relation == MyVoxel::VoxelRegionRelation::Inside &&
           statistics.rootCandidateCount == 1 &&
           statistics.existingRootCount == 1 &&
           statistics.materialNodeCount == 1;
}

// 验证生产森林继续保持根级写时复制。
bool testRootCopyOnWrite()
{
    MyVoxel::VoxelForest source;
    const MyVoxel::VoxelCellAddress root = rootAddress();

    source.setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelForest copy = source;
    MyVoxel::VoxelTreeEditor copyEditor(copy, root);

    copyEditor.split();
    copyEditor.child(corner(7)).setState(MyVoxel::VoxelState::Empty);

    return source.state(root) == MyVoxel::VoxelState::Material &&
           copy.state(root) == MyVoxel::VoxelState::Subdivided &&
           source.state(MyVoxel::childCellAddress(root, corner(7))) == MyVoxel::VoxelState::Material &&
           copy.state(MyVoxel::childCellAddress(root, corner(7))) == MyVoxel::VoxelState::Empty;
}

}

int main()
{
    check(testProductionForest(), "Production forest");
    check(testEditorAndCursor(), "Editor and cursor");
    check(testAccessor(), "Accessor");
    check(testQuery(), "Query");
    check(testRootCopyOnWrite(), "Root copy-on-write");

    std::cout << std::endl;
    std::cout << "Voxel production tree switch tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}