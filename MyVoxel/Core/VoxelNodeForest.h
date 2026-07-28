#ifndef MYVOXEL_VOXELNODEFOREST_H
#define MYVOXEL_VOXELNODEFOREST_H

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "VoxelAddress.h"
#include "VoxelRootTree.h"

namespace MyVoxel
{

class VoxelNodeForestConstAccessor;
class VoxelNodeForestConstCursor;
class VoxelNodeForestEditor;
class VoxelNodeForestQuery;

// 管理第0层根树及其节点结构，森林复制时共享根树，修改时只分离目标根树。
class VoxelNodeForest
{
public:
    VoxelNodeForest() = default;

    /// 节点状态

    // 返回指定地址当前对应的节点状态。
    VoxelState state(const VoxelCellAddress& address) const;

    // 检查指定地址是否存在独立节点。
    bool hasNode(const VoxelCellAddress& address) const;

    // 将指定地址设置为空或未分割材料状态，并在设置为空后立即清理空分支。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    // 将指定地址设置为空或未分割材料状态，但不执行空分支向上清理。
    bool setStateDeferred(const VoxelCellAddress& address, VoxelState state);

    /// 节点结构

    // 将指定地址对应的材料节点分割为八个材料子节点。
    bool split(const VoxelCellAddress& address);

    // 在八个子节点状态一致时合并指定已分割节点，并在合并为空后立即清理空分支。
    bool merge(const VoxelCellAddress& address);

    // 尝试合并指定节点但不执行空分支向上清理，返回节点处理后的状态。
    VoxelState mergeDeferred(const VoxelCellAddress& address);

    // 从指定空节点开始统一清理可合并的空分支。
    void pruneEmptyBranch(const VoxelCellAddress& address);

    /// 森林管理

    // 返回当前第0层根节点数量。
    std::size_t rootCount() const;

    // 清空当前森林的全部根树引用。
    void clear();
    // 分离并返回指定第0层根树，不存在时返回空指针。
    VoxelRootTree* detachRootTree(const VoxelCellIndex& rootIndex);

    // 删除指定第0层根树，返回是否实际删除。
    bool eraseRootTree(const VoxelCellIndex& rootIndex);
    
    /// 节点遍历

    using RootCellVisitor = std::function<void(const VoxelCellAddress&)>;
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;

    // 遍历指定第0层索引范围内实际存在的根节点。
    std::size_t forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const;

    // 遍历当前森林中所有状态为Material的节点。
    void forEachMaterialCell(const MaterialCellVisitor& visitor) const;

    // 遍历指定第0层索引范围内状态为Material的节点，并返回实际访问的根节点数量。
    std::size_t forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const;

private:
    friend class VoxelNodeForestConstAccessor;
    friend class VoxelNodeForestConstCursor;
    friend class VoxelNodeForestEditor;
    friend class VoxelNodeForestQuery;

    using RootTreePtr = std::shared_ptr<VoxelRootTree>;
    using RootMap = std::map<VoxelCellIndex, RootTreePtr>;
    using RootTreeVisitor = std::function<void(const VoxelCellIndex&, const VoxelRootTree&)>;

    // 返回指定地址所属的第0层根节点地址。
    static VoxelCellAddress rootCellAddress(const VoxelCellAddress& address);

    // 构造从第0层根节点到目标地址的角点路径，路径顺序为根节点到目标节点。
    static void buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path);

    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelRootTree* findRootTree(const VoxelCellIndex& rootIndex) const;

    // 返回指定索引对应的可写根树，共享时先复制根树，不存在时返回空指针。
    VoxelRootTree* findEditableRootTree(const VoxelCellIndex& rootIndex);

    // 返回指定索引对应的可写根树，不存在时使用指定状态创建。
    VoxelRootTree& ensureEditableRootTree(const VoxelCellIndex& rootIndex, VoxelState initialState);

    // 返回指定根树中的目标独立节点，不存在时返回空指针。
    static VoxelNode* findNode(VoxelRootTree& tree, const VoxelCellAddress& address);
    static const VoxelNode* findNode(const VoxelRootTree& tree, const VoxelCellAddress& address);

    // 返回指定地址对应的可写独立节点，共享根树会先执行根级写时复制。
    VoxelNode* findNode(const VoxelCellAddress& address);

    // 返回指定地址对应的只读独立节点，不存在时返回空指针。
    const VoxelNode* findNode(const VoxelCellAddress& address) const;

    // 在指定可写根树中创建到目标地址所需的节点结构。
    static VoxelNode& ensureNode(VoxelRootTree& tree, const VoxelCellAddress& address);

    // 从指定空节点开始向上清理完全为空的节点组。
    void removeEmptyBranch(VoxelCellAddress address);

    // 遍历指定索引范围内实际存在的根树。
    std::size_t visitRootTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootTreeVisitor& visitor) const;

    // 递归遍历指定根树节点下的全部材料节点。
    static void visitMaterialCells(const VoxelCellAddress& address, const VoxelNode& node, const VoxelNodePool& nodePool, const MaterialCellVisitor& visitor);

    RootMap m_roots; // 第0层体素索引与可共享独立根树的对应关系。
};

}

#endif // MYVOXEL_VOXELNODEFOREST_H
