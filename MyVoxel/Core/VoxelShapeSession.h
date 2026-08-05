#ifndef MYVOXEL_CORE_VOXELSHAPESESSION_H
#define MYVOXEL_CORE_VOXELSHAPESESSION_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

class VoxelCellAddress;
class VoxelGrid;
class VoxelShape;
class VoxelTree;

// 在一次连续修改过程中独占VoxelShape共享数据，并统一执行跨根体素修改。
//
// 创建会话时立即分离VoxelShape共享数据，后续修改不会重复进入Shape级写时复制。
// 各VoxelTree仍在首次创建编辑器时按根树粒度执行节点池写时复制。
// 每个成功修改原语都会同步写入当前VoxelChangeSet。
// 会话不自动合并节点，也不在析构时执行任何隐式修改。
// 会话存续期间不得复制、赋值或通过其他入口修改目标VoxelShape。
class VoxelShapeSession
{
public:
    // 为指定有效体素体创建修改会话，并使用固定层级跟踪材料变化区域。
    VoxelShapeSession(VoxelShape& shape, VoxelLevel changeTrackingLevel);
    VoxelShapeSession(const VoxelShapeSession& other) = delete;
    VoxelShapeSession& operator=(const VoxelShapeSession& other) = delete;
    // 移动修改会话并转移目标体素体绑定关系和变化记录。
    VoxelShapeSession(VoxelShapeSession&& other);
    // 移动修改会话并转移目标体素体绑定关系和变化记录。
    VoxelShapeSession& operator=(VoxelShapeSession&& other);

    ~VoxelShapeSession() = default;

    /// 会话状态
    // 判断当前会话是否已经执行过实际修改。
    bool hasChanges() const;
    // 返回当前尚未取走的只读变化记录。
    const VoxelChangeSet& changes() const;
    // 取走当前变化记录，并继续使用相同跟踪层级收集后续修改。
    VoxelChangeSet takeChanges();

    /// 体素空间
    // 返回目标体素体使用的只读体素网格。
    const VoxelGrid& grid() const;
    // 判断指定体素地址是否位于目标体素体允许的层级范围内。
    bool supportsAddress(const VoxelCellAddress& address) const;

    /// 体素状态
    // 返回指定地址对应的逻辑体素状态。
    VoxelState state(const VoxelCellAddress& address) const;
    // 检查指定地址是否具有独立的显式节点表示。
    bool hasNode(const VoxelCellAddress& address) const;
    // 将指定地址设置为空或材料状态，不自动合并任何祖先节点。
    bool setState(const VoxelCellAddress& address, VoxelState state);

    /// 体素结构
    // 将指定材料体素细分为八个材料子体素。
    bool split(const VoxelCellAddress& address);
    // 在八个直接子体素状态一致时只合并指定体素，不继续合并任何祖先节点。
    bool merge(const VoxelCellAddress& address);

    /// 独立脏区构建
    // 为指定Root创建与当前会话跟踪层级一致的空材料变化区域。
    VoxelChangeSet::DirtyCellRegion createDirtyCellRegion(const VoxelCellIndex& rootIndex) const;
    // 将指定逻辑体素覆盖区域记录到由当前会话创建的材料变化区域。
    void recordMaterialChange(VoxelChangeSet::DirtyCellRegion& dirtyRegion,const VoxelCellAddress& address) const;

    /// 根树操作
    // 返回指定索引对应的只读根树，不存在时返回空指针。
    const VoxelTree* tree(const VoxelCellIndex& rootIndex) const;
    // 使用指定非空根树新增或替换当前根树，并将该根记录为完整变化。
    void setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree);
    // 移动指定非空根树新增或替换当前根树，并将该根记录为完整变化。
    void setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree);
    // 使用指定非空根树替换当前Root，并提交调用者已经计算的精细材料变化区域。
    void setTree( const VoxelCellIndex& rootIndex, const VoxelTree& tree, const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    // 移动指定非空根树替换当前Root，并提交调用者已经计算的精细材料变化区域。
    void setTree( const VoxelCellIndex& rootIndex, VoxelTree&& tree, const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    // 删除指定根树，并在实际删除时将该根记录为完整变化。
    bool eraseTree(const VoxelCellIndex& rootIndex);
    // 删除指定根树，并在实际删除时提交调用者已经计算的精细材料变化区域。
    bool eraseTree(const VoxelCellIndex& rootIndex,const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    // 将全部根树所有权移动追加到指定数组，清空当前体素体并记录全部根变化。
    void moveTreesTo(std::vector<VoxelForest::TreeEntry>& trees);

    /// 体素数据
    // 返回当前会话使用的只读稀疏体素森林。
    const VoxelForest& forest() const;
    // 返回当前保存的第0层根树数量。
    std::size_t rootCount() const;
    // 判断当前是否不包含任何显式根树。
    bool isEmpty() const;
    // 清空全部显式根树，并将删除的每个根记录为完整变化。
    bool clear();

private:
    VoxelShape* m_shape; // 当前会话绑定的目标体素体。
    VoxelForest* m_forest; // 当前会话独占并修改的稀疏体素森林。
    VoxelChangeSet m_changes; // 当前会话尚未取走的只读变化记录。
};

}

#endif // MYVOXEL_CORE_VOXELSHAPESESSION_H