#include <cstddef>
#include <cstdint>
#include <iostream>

#include "MyVoxel/Core/Storage/VoxelBlockPool.h"
#include "MyVoxel/Core/Storage/VoxelLeafBlock.h"
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

// 返回指定数值对应的体素角点。
MyVoxel::VoxelCorner corner(unsigned int value)
{
    return static_cast<MyVoxel::VoxelCorner>(value);
}

// 验证两组状态位能够表达四种子单元存储状态。
bool testFourStorageStates()
{
    MyVoxel::VoxelNodeBlock block;

    block.reset();

    if (block.childStorageState(corner(0)) != MyVoxel::VoxelStorageState::Empty)
    {
        return false;
    }

    block.setChildLeafState(corner(1), MyVoxel::VoxelState::Material);

    block.firstChildIndex = 0;
    block.setChildBranch(corner(2));
    block.setChildMaskLeaf(corner(3));

    return block.childStorageState(corner(0)) == MyVoxel::VoxelStorageState::Empty &&
           block.childStorageState(corner(1)) == MyVoxel::VoxelStorageState::Material &&
           block.childStorageState(corner(2)) == MyVoxel::VoxelStorageState::Branch &&
           block.childStorageState(corner(3)) == MyVoxel::VoxelStorageState::MaskLeaf &&
           block.childState(corner(2)) == MyVoxel::VoxelState::Subdivided &&
           block.childState(corner(3)) == MyVoxel::VoxelState::Subdivided &&
           block.isValid();
}

// 验证同一个八槽组可以同时保存普通节点块和掩码叶块。
bool testMixedStorageGroup()
{
    MyVoxel::VoxelBlockPool pool;
    MyVoxel::VoxelNodeBlock parent;

    parent.reset();
    parent.firstChildIndex = pool.allocateChildren();
    parent.setChildBranch(corner(2));
    parent.setChildMaskLeaf(corner(5));

    MyVoxel::VoxelNodeBlock& branch =
        pool.initializeNode(parent.childNodeIndex(corner(2)), MyVoxel::VoxelState::Material);

    MyVoxel::VoxelLeafBlock& leaf =
        pool.initializeLeaf(parent.childLeafIndex(corner(5)), MyVoxel::VoxelState::Empty);

    leaf.setMaterial(corner(1), corner(7));
    leaf.setMaterial(corner(4), corner(3));

    return branch.childMask == 0 &&
           branch.leafMask == 0xFF &&
           leaf.materialVoxelCount() == 2 &&
           leaf.state(corner(1), corner(7)) == MyVoxel::VoxelState::Material &&
           leaf.state(corner(4), corner(3)) == MyVoxel::VoxelState::Material &&
           pool.isChildrenAddressAligned(parent.firstChildIndex);
}

// 验证递归释放只下降普通分支，不会将掩码叶块解释为普通节点。
bool testMixedRecursiveRelease()
{
    MyVoxel::VoxelBlockPool pool;
    MyVoxel::VoxelNodeBlock rootBlock;

    rootBlock.reset();
    rootBlock.firstChildIndex = pool.allocateChildren();
    rootBlock.setChildBranch(corner(0));
    rootBlock.setChildMaskLeaf(corner(1));

    MyVoxel::VoxelNodeBlock& branch = pool.initializeNode(rootBlock.childNodeIndex(corner(0)));
    MyVoxel::VoxelLeafBlock& rootLeaf = pool.initializeLeaf(rootBlock.childLeafIndex(corner(1)));

    rootLeaf.materialMask = static_cast<std::uint64_t>(0x55AA55AA55AA55AAULL);

    branch.firstChildIndex = pool.allocateChildren();
    branch.setChildMaskLeaf(corner(4));

    MyVoxel::VoxelLeafBlock& nestedLeaf = pool.initializeLeaf(branch.childLeafIndex(corner(4)));

    nestedLeaf.materialMask = static_cast<std::uint64_t>(0xFFFF0000FFFF0000ULL);

    if (pool.allocatedGroupCount() != 2)
    {
        return false;
    }

    pool.releaseChildren(rootBlock);

    return pool.allocatedGroupCount() == 0 &&
           !pool.containsChildren(rootBlock.firstChildIndex);
}

// 验证混合节点池深复制后普通节点和掩码叶块内容保持一致。
bool testMixedDeepCopy()
{
    MyVoxel::VoxelBlockPool source;
    MyVoxel::VoxelNodeBlock parent;

    parent.reset();
    parent.firstChildIndex = source.allocateChildren();
    parent.setChildBranch(corner(2));
    parent.setChildMaskLeaf(corner(6));

    source.initializeNode(parent.childNodeIndex(corner(2)), MyVoxel::VoxelState::Material);
    source.initializeLeaf(parent.childLeafIndex(corner(6))).materialMask =
        static_cast<std::uint64_t>(0x123456789ABCDEF0ULL);

    MyVoxel::VoxelBlockPool copy = source;

    if (copy.node(parent.childNodeIndex(corner(2))).leafMask != 0xFF ||
        copy.leaf(parent.childLeafIndex(corner(6))).materialMask != static_cast<std::uint64_t>(0x123456789ABCDEF0ULL))
    {
        return false;
    }

    copy.leaf(parent.childLeafIndex(corner(6))).materialMask = 0;

    return source.leaf(parent.childLeafIndex(corner(6))).materialMask ==
               static_cast<std::uint64_t>(0x123456789ABCDEF0ULL) &&
           copy.leaf(parent.childLeafIndex(corner(6))).materialMask == 0;
}

// 验证释放后的混合八槽组能够重新作为普通节点组使用。
bool testMixedGroupReuse()
{
    MyVoxel::VoxelBlockPool pool;
    MyVoxel::VoxelNodeBlock parent;

    parent.reset();
    parent.firstChildIndex = pool.allocateChildren();
    parent.setChildMaskLeaf(corner(7));

    const MyVoxel::VoxelNodeIndex releasedIndex = parent.firstChildIndex;

    pool.initializeLeaf(parent.childLeafIndex(corner(7)), MyVoxel::VoxelState::Material);
    pool.releaseChildren(parent);

    const MyVoxel::VoxelNodeIndex reusedIndex = pool.allocateChildren();

    if (reusedIndex != releasedIndex)
    {
        return false;
    }

    for (MyVoxel::VoxelNodeIndex childOffset = 0; childOffset < MyVoxel::VoxelBlockPool::ChildrenPerGroup; ++childOffset)
    {
        const MyVoxel::VoxelNodeBlock& block = pool.node(reusedIndex + childOffset);

        if (block.childMask != 0 ||
            block.leafMask != 0 ||
            block.userData != 0 ||
            block.firstChildIndex != MyVoxel::InvalidVoxelNodeIndex)
        {
            return false;
        }
    }

    return true;
}

}

int main()
{
    check(testFourStorageStates(), "Four storage states");
    check(testMixedStorageGroup(), "Mixed storage group");
    check(testMixedRecursiveRelease(), "Mixed recursive release");
    check(testMixedDeepCopy(), "Mixed deep copy");
    check(testMixedGroupReuse(), "Mixed group reuse");

    std::cout << std::endl;
    std::cout << "Voxel mixed storage tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}