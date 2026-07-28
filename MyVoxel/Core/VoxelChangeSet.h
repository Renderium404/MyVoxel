#ifndef MYVOXEL_VOXELCHANGESET_H
#define MYVOXEL_VOXELCHANGESET_H

#include <cstddef>
#include <set>

#include "VoxelAddress.h"

namespace MyVoxel
{

// 记录一次或多次体素修改实际影响的第0层根节点。
class VoxelChangeSet
{
public:
    using RootIndexSet = std::set<VoxelCellIndex>;
    // 清空全部修改根节点记录。
    void clear();
    // 返回当前是否记录到实际体素变化。
    bool hasChanges() const;
    // 返回实际修改根节点数量。
    std::size_t modifiedRootCount() const;
    // 检查指定第0层根节点是否发生修改。
    bool containsModifiedRoot(const VoxelCellIndex& rootIndex) const;
    // 记录指定第0层根节点发生修改。
    void addModifiedRoot(const VoxelCellIndex& rootIndex);
    // 累加另一个修改集合中的全部根节点。
    void accumulate(const VoxelChangeSet& other);
    // 返回全部实际修改根节点索引。
    const RootIndexSet& modifiedRootIndices() const;

private:
    RootIndexSet m_modifiedRootIndices; // 实际发生体素状态变化的第0层根节点索引。
};

}

#endif // MYVOXEL_VOXELCHANGESET_H