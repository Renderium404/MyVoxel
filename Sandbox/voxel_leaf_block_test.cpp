#include <cstddef>
#include <cstdint>
#include <iostream>

#include "MyVoxel/Core/Storage/VoxelLeafBlock.h"

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

// 验证叶块严格为8字节并能够初始化为空或完整材料。
bool testLayoutAndReset()
{
    MyVoxel::VoxelLeafBlock leaf;

    leaf.reset();

    if (sizeof(MyVoxel::VoxelLeafBlock) != 8 || !leaf.isEmpty() || leaf.isFull() || leaf.materialVoxelCount() != 0)
    {
        return false;
    }

    leaf.reset(MyVoxel::VoxelState::Material);

    return leaf.isFull() &&
           !leaf.isEmpty() &&
           leaf.materialVoxelCount() == MyVoxel::VoxelLeafBlock::VoxelCount &&
           leaf.materialMask == MyVoxel::VoxelLeafBlock::fullMask();
}

// 验证两级角点路径完整覆盖64个互不重复的位索引。
bool testBitIndexMapping()
{
    std::uint64_t accumulatedMask = 0;

    for (unsigned int coarseIndex = 0; coarseIndex < MyVoxel::VoxelLeafBlock::OctantCount; ++coarseIndex)
    {
        for (unsigned int fineIndex = 0; fineIndex < MyVoxel::VoxelLeafBlock::VoxelsPerOctant; ++fineIndex)
        {
            const unsigned int bitIndex = MyVoxel::VoxelLeafBlock::voxelBitIndex(corner(coarseIndex), corner(fineIndex));
            const std::uint64_t bit = MyVoxel::VoxelLeafBlock::voxelBit(corner(coarseIndex), corner(fineIndex));

            if (bitIndex != coarseIndex * MyVoxel::VoxelLeafBlock::VoxelsPerOctant + fineIndex ||
                bitIndex >= MyVoxel::VoxelLeafBlock::VoxelCount ||
                (accumulatedMask & bit) != 0)
            {
                return false;
            }

            accumulatedMask |= bit;
        }
    }

    return accumulatedMask == MyVoxel::VoxelLeafBlock::fullMask();
}

// 验证单体素设置、清除和状态读取。
bool testSingleVoxelState()
{
    MyVoxel::VoxelLeafBlock leaf;

    leaf.reset();

    for (unsigned int coarseIndex = 0; coarseIndex < MyVoxel::VoxelLeafBlock::OctantCount; ++coarseIndex)
    {
        for (unsigned int fineIndex = 0; fineIndex < MyVoxel::VoxelLeafBlock::VoxelsPerOctant; ++fineIndex)
        {
            leaf.setMaterial(corner(coarseIndex), corner(fineIndex));

            if (leaf.state(corner(coarseIndex), corner(fineIndex)) != MyVoxel::VoxelState::Material)
            {
                return false;
            }
        }
    }

    if (!leaf.isFull() || leaf.materialVoxelCount() != MyVoxel::VoxelLeafBlock::VoxelCount)
    {
        return false;
    }

    for (unsigned int coarseIndex = 0; coarseIndex < MyVoxel::VoxelLeafBlock::OctantCount; ++coarseIndex)
    {
        for (unsigned int fineIndex = 0; fineIndex < MyVoxel::VoxelLeafBlock::VoxelsPerOctant; ++fineIndex)
        {
            leaf.setEmpty(corner(coarseIndex), corner(fineIndex));

            if (leaf.state(corner(coarseIndex), corner(fineIndex)) != MyVoxel::VoxelState::Empty)
            {
                return false;
            }
        }
    }

    return leaf.isEmpty();
}

// 验证一个第一级八分区对应一个连续字节。
bool testOctantLayout()
{
    for (unsigned int coarseIndex = 0; coarseIndex < MyVoxel::VoxelLeafBlock::OctantCount; ++coarseIndex)
    {
        const std::uint64_t expectedMask = static_cast<std::uint64_t>(0xFF) << (coarseIndex * 8);

        if (MyVoxel::VoxelLeafBlock::octantMask(corner(coarseIndex)) != expectedMask)
        {
            return false;
        }
    }

    return true;
}

// 验证八分区的空、完整材料和混合状态摘要。
bool testOctantState()
{
    MyVoxel::VoxelLeafBlock leaf;

    leaf.reset();
    leaf.setOctantState(corner(2), MyVoxel::VoxelState::Material);
    leaf.setMaterial(corner(5), corner(3));

    const std::uint8_t expectedEmptyMask =
        static_cast<std::uint8_t>(
            static_cast<std::uint8_t>(1U << 0) |
            static_cast<std::uint8_t>(1U << 1) |
            static_cast<std::uint8_t>(1U << 3) |
            static_cast<std::uint8_t>(1U << 4) |
            static_cast<std::uint8_t>(1U << 6) |
            static_cast<std::uint8_t>(1U << 7));

    return leaf.octantState(corner(2)) == MyVoxel::VoxelState::Material &&
           leaf.octantState(corner(5)) == MyVoxel::VoxelState::Subdivided &&
           leaf.octantState(corner(0)) == MyVoxel::VoxelState::Empty &&
           leaf.materialOctantMask() == static_cast<std::uint8_t>(1U << 2) &&
           leaf.mixedOctantMask() == static_cast<std::uint8_t>(1U << 5) &&
           leaf.emptyOctantMask() == expectedEmptyMask;
}

// 验证64位差集、并集、交集和异或运算。
bool testBooleanOperations()
{
    MyVoxel::VoxelLeafBlock first;
    MyVoxel::VoxelLeafBlock second;

    first.materialMask = static_cast<std::uint64_t>(0xFFFF0000FFFF0000ULL);
    second.materialMask = static_cast<std::uint64_t>(0xFF00FF00FF00FF00ULL);

    MyVoxel::VoxelLeafBlock cutResult = first;
    MyVoxel::VoxelLeafBlock fuseResult = first;
    MyVoxel::VoxelLeafBlock commonResult = first;
    MyVoxel::VoxelLeafBlock xorResult = first;

    const bool cutChanged = cutResult.cut(second);
    const bool fuseChanged = fuseResult.fuse(second);
    const bool commonChanged = commonResult.common(second);
    const bool xorChanged = xorResult.exclusiveOr(second);

    return cutChanged &&
           fuseChanged &&
           commonChanged &&
           xorChanged &&
           cutResult.materialMask == static_cast<std::uint64_t>(0x00FF000000FF0000ULL) &&
           fuseResult.materialMask == static_cast<std::uint64_t>(0xFFFFFF00FFFFFF00ULL) &&
           commonResult.materialMask == static_cast<std::uint64_t>(0xFF000000FF000000ULL) &&
           xorResult.materialMask == static_cast<std::uint64_t>(0x00FFFF0000FFFF00ULL);
}

// 验证无实际变化的布尔运算返回false。
bool testUnchangedOperations()
{
    MyVoxel::VoxelLeafBlock empty;
    MyVoxel::VoxelLeafBlock full;

    empty.reset();
    full.reset(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelLeafBlock result = full;

    if (result.fuse(empty))
    {
        return false;
    }

    if (result.common(full))
    {
        return false;
    }

    if (result.cut(empty))
    {
        return false;
    }

    return result.isFull();
}

}

int main()
{
    check(testLayoutAndReset(), "Layout and reset");
    check(testBitIndexMapping(), "Bit index mapping");
    check(testSingleVoxelState(), "Single voxel state");
    check(testOctantLayout(), "Octant byte layout");
    check(testOctantState(), "Octant state summary");
    check(testBooleanOperations(), "Boolean mask operations");
    check(testUnchangedOperations(), "Unchanged operations");

    std::cout << std::endl;
    std::cout << "Voxel leaf block tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}