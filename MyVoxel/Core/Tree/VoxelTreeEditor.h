#ifndef MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H

#include <cstdint>

#include "VoxelTree.h"

namespace MyVoxel
{

// 在可写体素树中访问并修改普通节点、掩码叶块及其内部逻辑体素。
//
// 创建根编辑器时，如果当前节点池由多个体素树共享，则自动复制当前树可达的全部物理节点。
// 编辑器及其子编辑器均为非拥有引用，体素树销毁后不得继续使用。
// 修改祖先结构后，已经取得的兄弟或后代编辑器可能失效。
class VoxelTreeEditor
{
public:
    // 使用指定体素树创建根节点编辑器，并保证当前树独占节点池。
    explicit VoxelTreeEditor(VoxelTree& tree);

    /// 节点状态

    // 返回当前逻辑体素状态。
    VoxelTreeState state() const;

    // 判断当前逻辑体素是否为空。
    bool isEmpty() const;

    // 判断当前逻辑体素是否完全包含材料。
    bool isMaterial() const;

    // 判断当前逻辑体素是否已经细分。
    bool isSubdivided() const;

    // 判断当前逻辑体素是否为终止体素。
    bool isTerminal() const;

    // 判断当前逻辑体素是否能够访问直接子体素。
    bool canAccessChildren() const;

    /// 状态修改

    // 将当前逻辑体素设置为空，并释放其全部普通分支后代。
    void setEmpty();

    // 将当前逻辑体素设置为完全包含材料，并释放其全部普通分支后代。
    void setMaterial();

    // 设置当前逻辑体素状态，Subdivided使用普通节点块细分。
    void setState(VoxelTreeState state);

    // 将当前空或材料体素细分为普通节点块，八个子体素继承当前状态。
    void subdivide();

    // 将当前空或材料子体素细分为掩码叶块，用于压缩最后连续两层。
    void subdivideAsMaskLeaf();

    /// 子节点访问

    // 返回指定角点对应的直接逻辑子体素编辑器。
    //
    // 普通空或材料体素会自动细分为普通节点块。
    // 掩码叶块中的粗层逻辑体素可直接访问其八个最高层体素。
    VoxelTreeEditor child(VoxelCorner corner);

    /// 底层存储

    // 判断当前逻辑体素是否直接使用普通节点块描述子体素。
    bool usesNodeBlock() const;

    // 判断当前逻辑体素是否位于掩码叶块中。
    bool usesMaskLeaf() const;

    // 返回当前逻辑体素使用的可写普通节点块。
    VoxelNodeBlock& nodeBlock();

    // 返回当前逻辑体素使用的可写掩码叶块。
    VoxelLeafBlock& maskLeaf();

private:
    // 表示当前编辑器对应的逻辑位置。
    enum class Source : std::uint8_t
    {
        Root = 0, // 当前编辑器对应体素树根体素。
        NodeChild = 1, // 当前编辑器对应普通节点块中的一个直接子体素。
        MaskLeafGroup = 2, // 当前编辑器对应掩码叶块中的一个粗层体素。
        MaskLeafVoxel = 3 // 当前编辑器对应掩码叶块中的一个最高层体素。
    };

private:
    // 创建普通节点块直接子体素编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelNodeBlock& parentBlock, VoxelCorner childCorner);

    // 创建掩码叶块粗层体素编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner);

    // 创建掩码叶块最高层体素编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner);

    // 返回普通父节点记录的当前子体素物理状态。
    VoxelState nodeChildState() const;

    // 返回当前普通分支使用的节点块。
    VoxelNodeBlock& currentNodeBlock() const;

    // 返回当前掩码叶逻辑体素使用的叶块。
    VoxelLeafBlock& currentLeafBlock() const;

    // 将当前逻辑体素设置为Empty或Material。
    void setTerminalState(VoxelTreeState state);

    // 为普通父节点分配八槽组。
    void ensureParentStorage();

    // 当前普通父节点不再包含物理子节点时回收其八槽组。
    void releaseUnusedParentStorage();

    // 保证当前体素树独占节点池。
    static void detachPool(VoxelTree& tree);

    // 将一个普通节点块及其全部物理后代复制到目标节点池。
    static void cloneNodeBlock(const VoxelNodeBlock& sourceBlock, const VoxelBlockPool& sourcePool,
                               VoxelNodeBlock& targetBlock, VoxelBlockPool& targetPool);

    // 递归释放一个普通节点块记录的全部物理后代。
    static void releaseNodeChildren(VoxelNodeBlock& block, VoxelBlockPool& pool);

    // 设置普通节点块中一个子体素的状态位。
    static void setNodeChildState(VoxelNodeBlock& block, VoxelCorner corner, VoxelState state);

    // 设置掩码叶块中一个粗层体素对应的八位材料掩码。
    static void setLeafGroupMask(VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner, std::uint8_t mask);

private:
    VoxelTree* m_tree; // 当前编辑器所属的可写体素树。
    VoxelNodeBlock* m_parentBlock; // NodeChild位置对应的普通父节点块。
    VoxelLeafBlock* m_leafBlock; // 掩码叶逻辑位置对应的物理叶块。
    VoxelCorner m_childCorner; // 普通父节点或粗层体素中的当前子角点。
    VoxelCorner m_coarseCorner; // 掩码叶块中的粗层角点。
    Source m_source; // 当前编辑器对应的逻辑位置类型。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H