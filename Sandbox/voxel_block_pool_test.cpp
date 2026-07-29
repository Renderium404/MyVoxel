#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "MyVoxel/Core/Storage/VoxelBlockPool.h"
#include "MyVoxel/Core/Storage/VoxelNodeBlock.h"

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

// 验证节点块尺寸、状态掩码和结构约束。
bool testNodeBlockLayout()
{
    MyVoxel::VoxelNodeBlock block;
    block.reset();

    bool passed = sizeof(MyVoxel::VoxelNodeBlock) == 8;
    passed = passed && block.childMask == 0;
    passed = passed && block.leafMask == 0;
    passed = passed && block.userData == 0;
    passed = passed && block.firstChildIndex == MyVoxel::InvalidVoxelIndex;
    passed = passed && block.isValid();

    for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        passed = passed && block.childState(corner) == MyVoxel::VoxelState::Empty;
    }

    block.reset(MyVoxel::VoxelState::Material);

    passed = passed && block.childMask == 0;
    passed = passed && block.leafMask == 0xFF;
    passed = passed && block.isValid();

    for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        passed = passed && block.childState(corner) == MyVoxel::VoxelState::Material;
    }

    return passed;
}

// 验证八节点组连续存储并严格位于64字节对齐地址。
bool testAlignedContiguousAllocation()
{
    MyVoxel::VoxelBlockPool pool;
    std::vector<MyVoxel::VoxelIndex> firstChildIndexes;

    const std::size_t allocationCount = static_cast<std::size_t>(MyVoxel::VoxelBlockPool::GroupsPerChunk) + 1; // 额外分配一组以验证跨Chunk扩容。

    firstChildIndexes.reserve(allocationCount);

    for (std::size_t allocationIndex = 0; allocationIndex < allocationCount; ++allocationIndex)
    {
        const MyVoxel::VoxelIndex firstChildIndex = pool.allocateChildren();

        if ((firstChildIndex % MyVoxel::VoxelBlockPool::ChildrenPerGroup) != 0)
        {
            return false;
        }

        if (!pool.isChildrenAddressAligned(firstChildIndex))
        {
            return false;
        }

        MyVoxel::VoxelNodeBlock* firstNode = &pool.node(firstChildIndex);

        for (MyVoxel::VoxelIndex childOffset = 0; childOffset < MyVoxel::VoxelBlockPool::ChildrenPerGroup; ++childOffset)
        {
            if (&pool.node(firstChildIndex + childOffset) != firstNode + childOffset)
            {
                return false;
            }
        }

        const std::uintptr_t beginAddress = reinterpret_cast<std::uintptr_t>(firstNode);
        const std::uintptr_t endAddress = reinterpret_cast<std::uintptr_t>(firstNode + MyVoxel::VoxelBlockPool::ChildrenPerGroup);

        if (endAddress - beginAddress != MyVoxel::VoxelBlockPool::CacheLineAlignment)
        {
            return false;
        }

        firstChildIndexes.push_back(firstChildIndex);
    }

    return pool.chunkCount() == 2 &&
           pool.allocatedGroupCount() == allocationCount &&
           pool.storageGroupCount() == allocationCount;
}

// 验证释放后的八节点组能够原位复用并清除旧数据。
bool testGroupReuse()
{
    MyVoxel::VoxelBlockPool pool;

    const MyVoxel::VoxelIndex firstIndex = pool.allocateChildren();
    const MyVoxel::VoxelIndex secondIndex = pool.allocateChildren();

    pool.node(firstIndex).leafMask = 0xFF;
    pool.node(firstIndex).userData = 65535;
    pool.releaseUnusedChildren(firstIndex);

    const MyVoxel::VoxelIndex reusedIndex = pool.allocateChildren();
    const MyVoxel::VoxelNodeBlock& reusedNode = pool.node(reusedIndex);

    return reusedIndex == firstIndex &&
           secondIndex != firstIndex &&
           reusedNode.childMask == 0 &&
           reusedNode.leafMask == 0 &&
           reusedNode.userData == 0 &&
           reusedNode.firstChildIndex == MyVoxel::InvalidVoxelIndex &&
           reusedNode.isValid();
}

// 验证释放一个上层八节点组时会迭代回收全部后代节点组。
bool testDescendantRelease()
{
    MyVoxel::VoxelBlockPool pool;

    const MyVoxel::VoxelIndex rootChildren = pool.allocateChildren();
    const MyVoxel::VoxelIndex level1Children = pool.allocateChildren();
    const MyVoxel::VoxelIndex level2Children = pool.allocateChildren();

    MyVoxel::VoxelNodeBlock& level1Parent = pool.node(rootChildren + 2);
    level1Parent.childMask = MyVoxel::VoxelNodeBlock::cornerBit(MyVoxel::VoxelCorner::MaximumY);
    level1Parent.leafMask = 0;
    level1Parent.firstChildIndex = level1Children;

    MyVoxel::VoxelNodeBlock& level2Parent = pool.node(level1Children + 3);
    level2Parent.childMask = MyVoxel::VoxelNodeBlock::cornerBit(MyVoxel::VoxelCorner::MaximumXYZ);
    level2Parent.leafMask = 0;
    level2Parent.firstChildIndex = level2Children;
    MyVoxel::VoxelNodeBlock rootBlock;
    rootBlock.reset();
    rootBlock.firstChildIndex = rootChildren;
    rootBlock.setChildBranch(static_cast<MyVoxel::VoxelCorner>(2));
    if (!level1Parent.isValid() || !level2Parent.isValid())
    {
        return false;
    }

    pool.releaseChildren(rootBlock);
    return pool.allocatedGroupCount() == 0 &&
           !pool.containsChildren(rootChildren) &&
           !pool.containsChildren(level1Children) &&
           !pool.containsChildren(level2Children);
}

// 验证节点池复制为完整深复制，副本修改和释放不影响原池。
bool testDeepCopy()
{
    MyVoxel::VoxelBlockPool source;

    const MyVoxel::VoxelIndex firstChildIndex = source.allocateChildren();

    source.node(firstChildIndex + 4).leafMask = 0xA5;
    source.node(firstChildIndex + 4).userData = 12345;

    MyVoxel::VoxelBlockPool copy = source;

    if (copy.allocatedGroupCount() != source.allocatedGroupCount() ||
        copy.node(firstChildIndex + 4).leafMask != source.node(firstChildIndex + 4).leafMask ||
        copy.node(firstChildIndex + 4).userData != source.node(firstChildIndex + 4).userData ||
        &copy.node(firstChildIndex) == &source.node(firstChildIndex))
    {
        return false;
    }

    copy.node(firstChildIndex + 4).userData = 54321;
    copy.releaseUnusedChildren(firstChildIndex);

    return source.containsChildren(firstChildIndex) &&
           source.node(firstChildIndex + 4).userData == 12345 &&
           copy.allocatedGroupCount() == 0;
}

// 验证赋值运算执行深复制并正确释放原目标存储。
bool testDeepAssignment()
{
    MyVoxel::VoxelBlockPool source;
    MyVoxel::VoxelBlockPool target;

    const MyVoxel::VoxelIndex sourceIndex = source.allocateChildren();
    const MyVoxel::VoxelIndex targetIndex = target.allocateChildren();

    source.node(sourceIndex + 1).userData = 100;
    target.node(targetIndex + 1).userData = 200;

    target = source;

    if (target.allocatedGroupCount() != 1 ||
        target.node(sourceIndex + 1).userData != 100 ||
        &target.node(sourceIndex) == &source.node(sourceIndex))
    {
        return false;
    }

    target.node(sourceIndex + 1).userData = 300;

    return source.node(sourceIndex + 1).userData == 100 &&
           target.node(sourceIndex + 1).userData == 300;
}

}

int main()
{
    check(testNodeBlockLayout(), "Node block layout and masks");
    check(testAlignedContiguousAllocation(), "Aligned contiguous allocation");
    check(testGroupReuse(), "Released group reuse");
    check(testDescendantRelease(), "Descendant group release");
    check(testDeepCopy(), "Pool deep copy");
    check(testDeepAssignment(), "Pool deep assignment");

    std::cout << std::endl;
    std::cout << "Voxel block pool tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}