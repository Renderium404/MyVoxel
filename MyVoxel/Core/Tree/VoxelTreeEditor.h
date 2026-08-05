#ifndef MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H

#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"

#include "VoxelTree.h"

namespace MyVoxel
{

// 在可写体素树中访问并修改逻辑体素。
//
// 编辑器及其子编辑器均为非拥有引用，体素树销毁后不得继续使用。
// 修改祖先结构后，已经取得的兄弟或后代编辑器可能失效。
class VoxelTreeEditor
{
public:
    // 使用指定体素树创建根体素编辑器，并保证当前树独占节点池。
    explicit VoxelTreeEditor(VoxelTree& tree);

    /// 体素状态
    // 返回当前逻辑体素状态。
    VoxelState state() const;

    bool isEmpty() const{return state() == VoxelState::Empty;}
    bool isMaterial() const{return state() == VoxelState::Material;}
    bool isSubdivided() const{return state() == VoxelState::Subdivided;}
    bool isTerminal() const{return state() != VoxelState::Subdivided;}

    /// 子体素读取

    // 一次返回当前逻辑体素八个直接子体素的状态掩码。
    // Empty或Material体素不会为了读取而实际细分，而是返回八个继承当前状态的虚拟子体素。
    VoxelChildStateMasks childStateMasks() const;

    /// 体素修改
    // 将当前逻辑体素设置为空，并释放其全部物理后代。
    void setEmpty();
    // 将当前逻辑体素设置为完全包含材料，并释放其全部物理后代。
    void setMaterial();
    // 将当前空或材料体素细分，返回是否实际创建了普通分支。
    bool subdivide();

    // 将指定位置的直接子体素批量设置为空或材料，并正确释放原有物理后代。
    // 当前体素为终止状态且存在实际变化时，会先细分为普通分支。
    // 返回是否存在实际逻辑或存储变化。
    bool setChildrenState(std::uint8_t childMask, VoxelState state);
    // 返回指定角点对应的直接逻辑子体素编辑器。
    // 普通空或材料体素会自动细分；掩码叶块粗层体素可直接访问其八个细层体素。
    VoxelTreeEditor child(VoxelCorner corner);

    /// 压缩材料修改    
    bool canSetMaterialMask() const{return m_source == Source::NodeChild;}
    // 使用指定64位掩码设置当前位置覆盖的后两层材料状态。
    // 全空或全材料掩码自动折叠为终止体素，其余掩码使用压缩叶块保存。
    // 当前编辑器必须对应普通节点块中的一个直接子体素。
    bool setMaterialMask(std::uint64_t materialMask);
private:
    // 表示当前编辑器对应的逻辑位置。
    enum class Source : std::uint8_t
    {
        Root = 0, // 当前编辑器对应体素树根体素。
        NodeChild = 1, // 当前编辑器对应普通节点块中的一个直接子体素。
        MaskLeafGroup = 2, // 当前编辑器对应掩码叶块中的一个粗层体素。
        MaskLeafVoxel = 3 // 当前编辑器对应掩码叶块中的一个细层体素。
    };

private:
    // 创建普通节点块直接子体素编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelNodeBlock& parentBlock, VoxelCorner childCorner);
    // 创建掩码叶块粗层体素编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner);
    // 创建掩码叶块细层体素编辑器。
    VoxelTreeEditor(VoxelTree& tree,
                    VoxelLeafBlock& leafBlock,
                    VoxelCorner coarseCorner,
                    VoxelCorner fineCorner);
    // 返回普通父节点记录的当前子体素存储状态。
    VoxelNodeState nodeChildState() const;
    // 返回当前普通分支使用的可写节点块。
    VoxelNodeBlock& currentNodeBlock();
    // 返回当前普通分支使用的只读节点块。
    const VoxelNodeBlock& currentNodeBlock() const;
    // 返回当前掩码叶逻辑体素使用的可写叶块。
    VoxelLeafBlock& currentLeafBlock();
    // 返回当前掩码叶逻辑体素使用的只读叶块。
    const VoxelLeafBlock& currentLeafBlock() const;
    // 将当前逻辑体素设置为Empty或Material。
    void setTerminal(VoxelState state);
    // 批量修改普通节点块中的直接子体素状态。
    bool setNodeChildren(VoxelNodeBlock& block, std::uint8_t childMask, VoxelState state);
    // 为普通父节点分配八槽组。
    void ensureParentGroup();
    // 当前普通父节点不再包含物理子节点时回收其八槽组。
    void releaseUnusedParentGroup();

private:
    VoxelTree* m_tree; // 当前编辑器所属的可写体素树。
    VoxelNodeBlock* m_parentBlock; // NodeChild位置对应的普通父节点块。
    VoxelLeafBlock* m_leafBlock;   // 掩码叶逻辑位置对应的物理叶块。
    VoxelCorner m_childCorner;  // 父节点中当前角点的位置
    VoxelCorner m_coarseCorner; // 掩码叶块中的粗层角点。
    Source m_source;            // 当前编辑器对应的逻辑位置类型。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H