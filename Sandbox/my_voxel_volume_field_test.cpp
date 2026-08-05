#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "MyVoxel/Core/Volume/VolumeField.h"

namespace
{

int g_passed = 0;
int g_failed = 0;

// 记录一项测试结果。
void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[通过] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[失败] " << name << std::endl;
    }
}

// 判断两个单精度数值是否近似相等。
bool nearValue(float first, float second, float epsilon = 1.0e-6f)
{
    return std::fabs(static_cast<double>(first - second)) <= static_cast<double>(epsilon);
}

}

int main()
{
    using namespace MyVoxel;

    std::cout << "开始VolumeField存储与地址测试" << std::endl << std::endl;

    VolumeField field(6);
    check(field.isValid(), "初始结构有效");
    check(field.sampleLevel() == 6 && field.blockLevel() == 4, "采样层级与距离块层级");
    check(field.isEmpty() && field.isCompletelyDirty() && !field.isCurrent(), "初始距离场完全失效");

    const VoxelCellAddress negativeBlock(VoxelCellIndex(-1, -1, -1), field.blockLevel());
    const VoxelCellAddress minimumSample(VoxelCellIndex(-4, -4, -4), field.sampleLevel());
    const VoxelCellAddress maximumSample(VoxelCellIndex(-1, -1, -1), field.sampleLevel());

    check(field.blockAddress(minimumSample) == negativeBlock && field.sampleIndex(minimumSample) == 0, "负索引叶区最小样本映射");
    check(field.blockAddress(maximumSample) == negativeBlock && field.sampleIndex(maximumSample) == 63, "负索引叶区最大样本映射");
    check(field.sampleAddress(negativeBlock, 0) == minimumSample && field.sampleAddress(negativeBlock, 63) == maximumSample, "样本地址双向映射");

    VolumeBlock& block = field.ensureBlock(negativeBlock, 2.5f);
    check(field.blockCount() == 1 && field.containsBlock(negativeBlock), "稀疏距离块创建");
    check(nearValue(volumeDistance(block, 0), 2.5f) && nearValue(volumeDistance(block, 63), 2.5f), "距离块统一初始化");

    field.markAllValid();
    check(field.isCurrent() && !field.isBlockDirty(negativeBlock), "完整重建确认");

    VolumeBlock* edited = field.editBlock(negativeBlock);
    check(edited != nullptr && field.isBlockDirty(negativeBlock), "可写访问自动标记失效");
    volumeDistance(*edited, 7) = -1.25f;
    field.markBlockValid(negativeBlock);
    check(field.isCurrent() && nearValue(volumeDistance(*field.findBlock(negativeBlock), 7), -1.25f), "局部更新确认");

    const VoxelCellAddress absentBlock(VoxelCellIndex(3, 2, 1), field.blockLevel());
    field.markBlockDirty(absentBlock);
    check(field.dirtyBlockCount() == 1 && !field.containsBlock(absentBlock), "缺失窄带块也可记录失效");
    field.markBlockValid(absentBlock);
    check(field.isCurrent(), "缺失块确认位于有效窄带外");

    const std::size_t capacityBeforeReset = field.storageCapacityBytes();
    field.resetPreservingStorage();
    check(field.isEmpty() && field.isCompletelyDirty(), "保留存储重置");
    check(field.storageCapacityBytes() == capacityBeforeReset && capacityBeforeReset >= 64U * 1024U, "重置保留Chunk容量");

    field.markAllValid();
    field.ensureBlock(negativeBlock, -3.0f);
    check(field.isBlockDirty(negativeBlock), "当前距离场新增块自动失效");
    field.markBlockValid(negativeBlock);
    check(field.eraseBlock(negativeBlock) && field.isEmpty() && field.isCurrent(), "距离块删除与有效状态");

    field.releaseStorage();
    check(field.storageCapacityBytes() == 0 && field.isCompletelyDirty(), "完全释放距离块存储");
    check(field.isValid(), "最终结构有效");

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}