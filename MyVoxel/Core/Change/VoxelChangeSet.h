#ifndef MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H
#define MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H

#include <cstddef>
#include <set>

#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

// 记录一次或多次体素修改实际影响的第0层根体素索引。
class VoxelChangeSet
{
public:
    using RootIndexSet = std::set<VoxelCellIndex>;

    /// 状态判断

    // 判断当前是否记录了实际发生变化的根体素。
    bool hasChanges() const;

    // 判断指定第0层根体素是否被记录为已修改。
    bool containsModifiedRoot(const VoxelCellIndex& rootIndex) const;

    /// 修改记录

    // 记录指定第0层根体素发生修改，重复记录不会增加数量。
    void addModifiedRoot(const VoxelCellIndex& rootIndex);

    // 累加另一个修改集合中的全部根体素索引。
    void accumulate(const VoxelChangeSet& other);

    // 清空全部根体素修改记录。
    void clear();

    /// 记录访问

    // 返回当前记录的实际修改根体素数量。
    std::size_t modifiedRootCount() const;

    // 返回全部实际修改根体素索引。
    const RootIndexSet& modifiedRootIndices() const;

private:
    RootIndexSet m_modifiedRootIndices; // 实际发生体素状态或结构变化的第0层根体素索引。
};

}

#endif // MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H