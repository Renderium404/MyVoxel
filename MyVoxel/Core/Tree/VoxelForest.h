#ifndef MYVOXEL_CORE_TREE_VOXELFOREST_H
#define MYVOXEL_CORE_TREE_VOXELFOREST_H

#include <cstddef>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "../VoxelAddress.h"
#include "VoxelTree.h"

namespace MyVoxel
{

class VoxelForestAccessor;
class VoxelForestQuery;
class VoxelTreeCursor;

// 管理全部第0层根体素树，森林复制时各根树共享节点池，修改时只分离目标根树的节点池。
class VoxelForest
{
public:
    VoxelForest() = default;
    // 复制森林中的根树值对象，各对应节点池保持共享。
    VoxelForest(const VoxelForest& other) = default;
    // 复制森林中的根树值对象，各对应节点池保持共享。
    VoxelForest& operator=(const VoxelForest& other) = default;
    // 移动森林中的全部根树并直接转移各根树节点池所有权。
    VoxelForest(VoxelForest&& other);
    // 移动森林中的全部根树并直接转移各根树节点池所有权。
    VoxelForest& operator=(VoxelForest&& other);

    /// 节点状态

    // 返回指定地址对应的逻辑体素状态，不存在的根树统一视为空。
    VoxelState state(const VoxelCellAddress& address) const;
    // 检查指定地址是否在现有根树中具有精确的逻辑节点表示。
    bool hasNode(const VoxelCellAddress& address) const;
    // 将指定地址设置为空或材料状态，不自动合并任何祖先节点。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    /// 节点结构

    // 将指定材料体素细分为八个材料子体素。
    bool split(const VoxelCellAddress& address);
    // 在八个直接子体素状态一致时只合并指定体素，不继续合并任何祖先节点。
    bool merge(const VoxelCellAddress& address);

    /// 森林管理

    // 保存一个第0层根索引及其根树所有权。
    using TreeEntry = std::pair<VoxelCellIndex, VoxelTree>;
    // 返回当前森林是否不包含任何根树。
    bool isEmpty() const;
    // 返回当前第0层根树数量。
    std::size_t rootCount() const;

    // 清空当前森林中的全部根树并释放各根树持有的节点池引用。
    void clear();
    // 将全部根树移动追加到rootTrees，并将当前森林置空。
    // 根树节点池所有权直接转移，不增加引用计数，也不执行物理节点复制。
    void moveTreesTo(std::vector<TreeEntry>& rootTrees);
    // 删除指定第0层根树，返回是否实际删除。
    bool eraseTree(const VoxelCellIndex& rootIndex);
    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelTree* getTree(const VoxelCellIndex& rootIndex) const;
    // 返回指定索引对应的可写根树，不存在时返回空指针。
    // 返回可写树本身不会立即分离节点池，创建VoxelTreeEditor后才执行根树级写时复制。
    VoxelTree* getTree(const VoxelCellIndex& rootIndex);
    // 使用指定非空根树新增或替换当前根树，并返回保存后的根树。
    // VoxelTree复制只共享节点池，不执行物理节点深复制。
    VoxelTree& setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree);
    // 移动指定非空根树新增或替换当前根树，并返回保存后的根树。
    // 根树节点池所有权直接转移，不增加引用计数，也不执行物理节点复制。
    VoxelTree& setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree);

    /// 节点遍历

    using RootCellVisitor = std::function<void(const VoxelCellAddress&)>;
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;
    // 遍历当前森林中的全部第0层根体素，返回实际访问数量。
    std::size_t forEachRootCell(const RootCellVisitor& visitor) const;
    // 遍历指定第0层索引范围内实际存在的根体素，返回实际访问数量。
    std::size_t forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const;
    // 遍历森林中的全部压缩Material体素。
    void forEachMaterialCell(const MaterialCellVisitor& visitor) const;
    // 遍历指定根索引范围内的全部压缩Material体素，返回实际访问的根树数量。
    std::size_t forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const;

private:
    friend class VoxelForestAccessor;
    friend class VoxelForestQuery;

    using TreeMap = std::map<VoxelCellIndex, VoxelTree>;
    using TreeVisitor = std::function<void(const VoxelCellIndex&, const VoxelTree&)>;

    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelTree* findTree(const VoxelCellIndex& rootIndex) const;
    // 返回指定索引对应的可写根树，不存在时返回空指针。
    VoxelTree* findTree(const VoxelCellIndex& rootIndex);
    // 返回指定索引对应的根树，不存在时使用指定终止状态创建。
    VoxelTree& ensureTree(const VoxelCellIndex& rootIndex, VoxelState initialState);
    // 遍历指定索引范围内实际存在的根树。
    std::size_t visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const;
    // 递归遍历指定游标下的全部压缩Material体素。
    static void visitMaterialCells(const VoxelCellAddress& address, const VoxelTreeCursor& cursor, const MaterialCellVisitor& visitor);

private:
    TreeMap m_trees; // 第0层体素索引与根树的对应关系，森林复制时各根树共享节点池。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELFOREST_H