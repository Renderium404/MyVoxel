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

// 管理一个固定截断背景距离下的全部第0层根体素树。
//
// Forest缺失根统一表示+B外部背景。
// 显式根树可以是任意[-B,+B]内的Empty/Material Tile Value或Subdivided树，
// 但显式Empty根Value等于+B时属于冗余背景，不允许作为规范Forest状态保留。
class VoxelForest
{
public:
    // 使用默认截断背景距离1创建空森林。
    VoxelForest();
    // 使用指定固定截断背景距离创建空森林。
    explicit VoxelForest(float backgroundDistance);
    // 复制森林、固定截断背景距离和根树值对象，各根树继续共享BlockPool。
    VoxelForest(const VoxelForest& other) = default;
    // 复制森林、固定截断背景距离和根树值对象，各根树继续共享BlockPool。
    VoxelForest& operator=(const VoxelForest& other) = default;
    // 移动森林中的固定场参数和全部根树。
    VoxelForest(VoxelForest&& other);
    // 移动森林中的固定场参数和全部根树。
    VoxelForest& operator=(VoxelForest&& other);

    /// 场属性

    // 检查固定截断背景距离、全部显式根树和稀疏背景规范是否满足TSDF约束。
    bool isValid() const;
    // 返回当前森林固定使用的正截断背景距离B。
    float backgroundDistance() const;

    /// 节点状态

    // 返回指定地址对应的逻辑体素状态，不存在的根树统一视为空侧背景。
    VoxelState state(const VoxelCellAddress& address) const;
    // 检查指定地址是否在现有根树中具有精确的逻辑节点表示。
    bool hasNode(const VoxelCellAddress& address) const;
    // 将指定逻辑区域设置为+B空侧或-B材料侧终止场，不自动合并祖先节点。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    /// 距离场

    // 返回指定地址的实际Tile Value或LeafBlock距离；目标必须解析到终止Tile或最细叶样本。
    float distance(const VoxelCellAddress& address) const;
    // 修改指定最高层样本距离，必要时将目标上方两级区域无损重建为MaskLeaf。
    // 当前物理布局要求目标层级至少为第3层，调用者负责保证该地址确实是场的最高采样层级。
    bool setDistance(const VoxelCellAddress& address, float distance);

    /// 节点结构

    // 将指定现有终止Tile无损细分为八个同Value子Tile，不改变距离场。
    bool split(const VoxelCellAddress& address);
    // 仅在指定物理节点的直接内容能够无损归约为单一Value时执行一次局部合并，不继续合并祖先。
    bool merge(const VoxelCellAddress& address);
    // 对指定根树执行完整bottom-up无损裁剪，并删除最终归约为+B背景的根树。
    bool pruneTree(const VoxelCellIndex& rootIndex);
    // 对全部现有根树执行完整bottom-up无损裁剪，返回发生结构变化或根删除的数量。
    std::size_t prune();

    /// 森林管理

    using TreeEntry = std::pair<VoxelCellIndex, VoxelTree>;
    // 返回当前森林是否不包含任何显式根树。
    bool isEmpty() const;
    // 返回当前第0层显式根树数量。
    std::size_t rootCount() const;
    // 清空全部根树并保留固定backgroundDistance。
    void clear();
    // 将全部根树移动追加到rootTrees并清空当前森林；接收方必须使用兼容backgroundDistance。
    void moveTreesTo(std::vector<TreeEntry>& rootTrees);
    // 删除指定第0层根树，返回是否实际删除。
    bool eraseTree(const VoxelCellIndex& rootIndex);
    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelTree* getTree(const VoxelCellIndex& rootIndex) const;
    // 返回指定索引对应的可写根树，不存在时返回空指针；创建Editor后才执行根树级COW。
    VoxelTree* getTree(const VoxelCellIndex& rootIndex);
    // 使用指定非背景且满足当前TSDF范围的根树新增或替换当前根树。
    VoxelTree& setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree);
    // 移动指定非背景且满足当前TSDF范围的根树新增或替换当前根树。
    VoxelTree& setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree);

    /// 节点遍历

    using RootCellVisitor = std::function<void(const VoxelCellAddress&)>;
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;
    // 遍历当前森林中的全部显式第0层根体素，返回实际访问数量。
    std::size_t forEachRootCell(const RootCellVisitor& visitor) const;
    // 遍历指定第0层索引范围内实际存在的根体素，返回实际访问数量。
    std::size_t forEachRootCellInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const RootCellVisitor& visitor) const;
    // 遍历森林中的全部压缩Material体素和最细材料样本。
    void forEachMaterialCell(const MaterialCellVisitor& visitor) const;
    // 遍历指定根索引范围内的全部压缩Material体素和最细材料样本，返回实际访问的根树数量。
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
    // 返回指定索引对应的根树，不存在时使用指定终止状态和Value创建。
    VoxelTree& ensureTree(const VoxelCellIndex& rootIndex, VoxelState initialState, float initialValue);
    // 判断指定根树是否与缺失根+B背景完全等价。
    bool isBackgroundTree(const VoxelTree& tree) const;
    // 遍历指定索引范围内实际存在的根树。
    std::size_t visitTreesInRange(const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex, const TreeVisitor& visitor) const;
    // 递归遍历指定游标下的全部压缩Material体素和最细材料样本。
    static void visitMaterialCells(const VoxelCellAddress& address, const VoxelTreeCursor& cursor, const MaterialCellVisitor& visitor);

private:
    float m_backgroundDistance; // 当前森林固定使用的正截断背景距离B。
    TreeMap m_trees; // 第0层体素索引与显式根树的对应关系，缺项统一表示+B外部背景。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELFOREST_H