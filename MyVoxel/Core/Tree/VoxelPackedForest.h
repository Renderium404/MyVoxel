#ifndef MYVOXEL_VOXELPACKEDFOREST_H
#define MYVOXEL_VOXELPACKEDFOREST_H

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "../VoxelAddress.h"
#include "VoxelPackedRootTree.h"
#include "VoxelPackedTreeConstCursor.h"
#include "VoxelPackedTreeEditor.h"

namespace MyVoxel
{
class VoxelPackedForestConstAccessor;
class VoxelPackedForestQuery;
class VoxelTreeConstCursor;
// 管理Packed第0层根树，森林复制时共享根树，修改时只深复制目标根树。
class VoxelPackedForest
{
public:
    VoxelPackedForest() = default;

    /// 节点状态

    // 返回指定地址当前对应的节点状态。
    VoxelState state(const VoxelCellAddress& address) const;

    // 检查指定地址是否具有独立的显式节点表示。
    bool hasNode(const VoxelCellAddress& address) const;

    // 将指定地址设置为空或材料状态，并在设置为空后立即清理空分支。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    // 将指定地址设置为空或材料状态，但不执行空分支向上清理。
    bool setStateDeferred(const VoxelCellAddress& address, VoxelState state);

    /// 节点结构

    // 将指定地址对应的材料节点细分为八个材料子节点。
    bool split(const VoxelCellAddress& address);

    // 在八个直接子单元状态一致时合并指定节点，并在合并为空后清理空分支。
    bool merge(const VoxelCellAddress& address);

    // 尝试合并指定节点但不执行空分支向上清理，返回处理后的节点状态。
    VoxelState mergeDeferred(const VoxelCellAddress& address);

    // 从指定空节点开始统一清理可合并的空分支。
    void pruneEmptyBranch(const VoxelCellAddress& address);

    /// 森林管理

    // 返回当前森林是否不包含任何根树。
    bool isEmpty() const;

    // 返回当前第0层根节点数量。
    std::size_t rootCount() const;

    // 清空当前森林的全部根树引用。
    void clear();

    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelPackedRootTree* rootTree(const VoxelCellIndex& rootIndex) const;

    // 分离并返回指定第0层根树，不存在时返回空指针。
    VoxelPackedRootTree* detachRootTree(const VoxelCellIndex& rootIndex);

    // 删除指定第0层根树，返回是否实际删除。
    bool eraseRootTree(const VoxelCellIndex& rootIndex);

    /// 节点遍历

    using RootCellVisitor = std::function<void(const VoxelCellAddress&)>;
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;

    // 遍历当前森林中的全部第0层根节点。
    std::size_t forEachRootCell(const RootCellVisitor& visitor) const;

    // 遍历指定第0层索引范围内实际存在的根节点。
    std::size_t forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const;
        
    
    // 遍历森林中全部Material节点。
    void forEachMaterialCell(const MaterialCellVisitor& visitor) const;

    // 遍历指定根索引范围内的全部Material节点，并返回实际访问的根树数量。
    std::size_t forEachMaterialCellInRootRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const MaterialCellVisitor& visitor) const;

private:
    friend class VoxelPackedForestConstAccessor;
    friend class VoxelPackedForestQuery;
    friend class VoxelTreeConstCursor;
    using RootTreePtr = std::shared_ptr<VoxelPackedRootTree>;
    using RootMap = std::map<VoxelCellIndex, RootTreePtr>;
    using RootTreeVisitor = std::function<void(const VoxelCellIndex&, const VoxelPackedRootTree&)>;

    // 返回指定地址所属的第0层根节点地址。
    static VoxelCellAddress rootCellAddress(const VoxelCellAddress& address);

    // 构造从第0层根节点到目标节点的角点路径。
    static void buildCornerPath(const VoxelCellAddress& address, std::vector<VoxelCorner>& path);

    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelPackedRootTree* findRootTree(const VoxelCellIndex& rootIndex) const;

    // 返回指定索引对应的可写根树，共享时先执行深复制。
    VoxelPackedRootTree* findEditableRootTree(const VoxelCellIndex& rootIndex);

    // 返回指定索引对应的可写根树，不存在时使用指定状态创建。
    VoxelPackedRootTree& ensureEditableRootTree(const VoxelCellIndex& rootIndex, VoxelState initialState);

    // 返回指定显式节点的只读游标。
    static VoxelPackedTreeConstCursor locateCursor(const VoxelPackedRootTree& tree, const VoxelCellAddress& address);

    // 返回指定显式节点的可写编辑器。
    static VoxelPackedTreeEditor locateEditor(VoxelPackedRootTree& tree, const VoxelCellAddress& address);

    // 创建到目标地址所需的全部中间节点，并返回目标节点编辑器。
    static VoxelPackedTreeEditor ensureEditor(VoxelPackedRootTree& tree, const VoxelCellAddress& address);

    // 检查指定根树中的目标地址是否具有显式节点表示。
    static bool hasNode(const VoxelPackedRootTree& tree, const VoxelCellAddress& address);

    // 从指定空节点开始向上清理完全为空的分支。
    void removeEmptyBranch(VoxelCellAddress address);

    // 遍历指定索引范围内实际存在的根树。
    std::size_t visitRootTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootTreeVisitor& visitor) const;

    // 递归遍历指定游标下的全部Material节点。
    static void visitMaterialCells(const VoxelCellAddress& address, const VoxelPackedTreeConstCursor& cursor, const MaterialCellVisitor& visitor);

    RootMap m_roots; // 第0层体素索引与可共享Packed根树的对应关系。
};

}

#endif // MYVOXEL_VOXELPACKEDFOREST_H