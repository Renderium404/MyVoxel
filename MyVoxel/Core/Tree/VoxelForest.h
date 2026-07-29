#ifndef MYVOXEL_CORE_TREE_VOXELFOREST_H
#define MYVOXEL_CORE_TREE_VOXELFOREST_H

#include <cstddef>
#include <functional>
#include <map>
#include <vector>

#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "VoxelTree.h"

namespace MyVoxel
{

class VoxelForestConstAccessor;
class VoxelForestQuery;
class VoxelTreeConstCursor;
class VoxelTreeEditor;

// 管理第0层稀疏体素树，森林复制时共享树，修改时仅分离目标树。
class VoxelForest
{
public:
    VoxelForest() = default;
    VoxelForest(const VoxelForest&) = default;
    VoxelForest& operator=(const VoxelForest&) = default;

    /// 节点状态

    // 返回指定地址当前表示的节点状态，不存在体素树时返回Empty。
    VoxelState state(const VoxelCellAddress& address) const;

    // 判断指定地址是否存在实际创建的独立节点。
    bool hasNode(const VoxelCellAddress& address) const;

    // 将指定地址设置为叶节点状态，并在设置为空后立即清理空分支。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    // 将指定地址设置为叶节点状态，但不执行空分支向上清理。
    bool setStateDeferred(const VoxelCellAddress& address, VoxelState state);

    /// 节点结构

    // 将指定材料节点细分为八个材料子节点。
    bool split(const VoxelCellAddress& address);

    // 尝试合并指定节点，并在结果为空时立即清理空分支；返回处理后该地址是否为叶节点。
    bool merge(const VoxelCellAddress& address);

    // 尝试合并指定节点但不清理空分支，返回处理后的节点状态。
    VoxelState mergeDeferred(const VoxelCellAddress& address);

    // 从指定空节点开始向上清理可以合并的空分支。
    void pruneEmptyBranch(const VoxelCellAddress& address);

    /// 森林管理

    // 返回当前实际存在的第0层体素树数量。
    std::size_t rootCount() const;

    // 判断当前森林是否不存在体素树。
    bool isEmpty() const;

    // 清空当前森林的全部体素树引用。
    void clear();

    /// 节点遍历

    using RootCellVisitor = std::function<void(const VoxelCellAddress&)>;
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;

    // 按根索引升序遍历指定闭区间内实际存在的第0层根体素，并返回实际访问的根树数量。
    std::size_t forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const;

    // 按根索引和角点顺序遍历当前森林中的全部材料叶节点。
    void forEachMaterialCell(const MaterialCellVisitor& visitor) const;

    // 遍历指定根索引闭区间内的全部材料叶节点，并返回实际访问的根树数量。
    std::size_t forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const;

private:
    friend class VoxelForestConstAccessor;
    friend class VoxelForestQuery;
    friend class VoxelTreeConstCursor;
    friend class VoxelTreeEditor;

    using TreePtr = Foundation::RefPtr<VoxelTree>;
    using TreeMap = std::map<VoxelCellIndex, TreePtr>;
    using TreeVisitor = std::function<void(const VoxelCellIndex&, const VoxelTree&)>;

    // 构造从第0层根节点到目标地址的角点路径。
    static void buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path);

    // 返回指定索引对应的只读体素树，不存在时返回空指针。
    const VoxelTree* findTree(const VoxelCellIndex& rootIndex) const;

    // 返回指定索引对应的可写体素树，共享时先深度复制，不存在时返回空指针。
    VoxelTree* findEditableTree(const VoxelCellIndex& rootIndex);

    // 返回指定索引对应的可写体素树，不存在时使用指定状态创建。
    VoxelTree& ensureEditableTree(const VoxelCellIndex& rootIndex, VoxelState initialState);

    // 返回指定体素树中实际存在的目标节点，不存在时返回空指针。
    static VoxelNode* findNode(VoxelTree& tree, const VoxelCellAddress& address);

    // 返回指定体素树中实际存在的只读目标节点，不存在时返回空指针。
    static const VoxelNode* findNode(const VoxelTree& tree, const VoxelCellAddress& address);

    // 返回指定地址对应的只读实际节点，不存在时返回空指针。
    const VoxelNode* findNode(const VoxelCellAddress& address) const;

    // 在指定可写体素树中创建到目标地址所需的节点结构。
    static VoxelNode& ensureNode(VoxelTree& tree, const VoxelCellAddress& address);

    // 从指定空节点开始向上清理完全为空的节点组。
    void removeEmptyBranch(VoxelCellAddress address);

    // 遍历指定根索引闭区间内实际存在的体素树。
    std::size_t visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const;

    // 递归遍历指定节点及其下级节点中的全部材料叶节点。
    static void visitMaterialCells(const VoxelCellAddress& address, const VoxelNode& node, const VoxelNodePool& nodePool, const MaterialCellVisitor& visitor);

private:
    TreeMap m_trees; // 第0层体素索引与可共享独立体素树的对应关系。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELFOREST_H