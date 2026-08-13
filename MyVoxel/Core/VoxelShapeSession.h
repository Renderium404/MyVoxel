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
class VoxelFeatureSet;
class VoxelGrid;
class VoxelShape;
class VoxelTree;
enum class VoxelFeatureState;

// 在一次连续修改过程中独占VoxelShape共享数据，并统一修改TSDF森林、显式FeatureSet和记录场变化区域。
//
// 当前VoxelChangeSet仍只记录TSDF Field/Structure变化；Feature变化记录将在切削Feature阶段单独扩展。
// 因此setFeatures/clearFeatures只修改与TSDF共享同一COW边界的FeatureSet，不写入现有VoxelChangeSet。
class VoxelShapeSession
{
public:
    // 为指定有效体素体创建修改会话，并使用固定层级跟踪距离场变化区域。
    VoxelShapeSession(VoxelShape& shape, VoxelLevel changeTrackingLevel);

    VoxelShapeSession(const VoxelShapeSession& other) = delete;
    VoxelShapeSession& operator=(const VoxelShapeSession& other) = delete;
    VoxelShapeSession(VoxelShapeSession&& other);
    VoxelShapeSession& operator=(VoxelShapeSession&& other);
    ~VoxelShapeSession() = default;

    /// 会话状态

    bool hasChanges() const;
    const VoxelChangeSet& changes() const;
    VoxelChangeSet takeChanges();

    /// 体素空间

    const VoxelGrid& grid() const;
    // 返回目标体素体固定使用的正截断背景距离B。
    float backgroundDistance() const;
    bool supportsAddress(const VoxelCellAddress& address) const;

    /// 体素与距离状态

    VoxelState state(const VoxelCellAddress& address) const;
    bool hasNode(const VoxelCellAddress& address) const;
    // 返回最高采样层级指定地址的TSDF距离。
    float distance(const VoxelCellAddress& address) const;
    // 将指定逻辑区域设置为空侧+B或材料侧-B，实际改变场时记录FieldChange。
    bool setState(const VoxelCellAddress& address, VoxelState state);
    // 修改最高采样层级指定地址的有限TSDF距离，并同步MaskBlock与FieldChange。
    bool setDistance(const VoxelCellAddress& address, float distance);

    /// 体素结构

    // 将指定现有终止区域无损细分，不改变距离场。
    bool split(const VoxelCellAddress& address);
    // 在指定物理节点可以无损折叠时执行一次局部合并，不改变距离场。
    bool merge(const VoxelCellAddress& address);
    // 对指定根执行完整bottom-up TSDF裁剪，不改变距离场。
    bool pruneTree(const VoxelCellIndex& rootIndex);
    // 对全部根执行完整bottom-up TSDF裁剪，返回发生结构变化的根数量。
    std::size_t prune();

    /// 独立脏区构建

    // 为指定Root创建与当前会话跟踪层级一致的空场变化区域。
    VoxelChangeSet::DirtyCellRegion createDirtyCellRegion(const VoxelCellIndex& rootIndex) const;
    // 将指定逻辑体素覆盖区域记录到由当前会话创建的场变化区域，不直接修改Shape。
    void recordFieldChange(VoxelChangeSet::DirtyCellRegion& dirtyRegion, const VoxelCellAddress& address) const;
    // 兼容旧Operation调用，语义等同recordFieldChange；旧调用迁移完成后可删除。
    void recordMaterialChange(VoxelChangeSet::DirtyCellRegion& dirtyRegion, const VoxelCellAddress& address) const;

    /// 根树操作

    const VoxelTree* tree(const VoxelCellIndex& rootIndex) const;
    // 插入的显式LeafBlock必须已经使用当前Session相同的backgroundDistance。
    void setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree);
    void setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree);
    void setTree(const VoxelCellIndex& rootIndex, const VoxelTree& tree, const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    void setTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree, const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    bool eraseTree(const VoxelCellIndex& rootIndex);
    bool eraseTree(const VoxelCellIndex& rootIndex, const VoxelChangeSet::DirtyCellRegion& dirtyRegion);
    void moveTreesTo(std::vector<VoxelForest::TreeEntry>& trees);

    /// 显式表面特征

    // 返回当前独占VoxelShape中的只读FeatureSet。
    const VoxelFeatureSet& features() const;
    // 返回当前显式表面特征的完整性状态。
    VoxelFeatureState featureState() const;
    // 判断当前FeatureSet是否已知完整。
    bool hasCompleteFeatures() const;
    // 使用完整有效FeatureSet替换当前显式特征并标记Complete；本阶段不写入VoxelChangeSet。
    void setFeatures(const VoxelFeatureSet& features);
    // 清空全部显式表面特征并标记Complete，表示当前实体确认没有显式特征。
    bool clearFeatures();
    // 清空可能过期的显式特征并标记Unavailable，表示当前TSDF仍有效但Feature需要重新建立。
    void invalidateFeatures();

    /// 体素数据

    const VoxelForest& forest() const;
    std::size_t rootCount() const;
    bool isEmpty() const;
    // 清空TSDF森林和FeatureSet；返回任一部分实际发生改变。
    bool clear();

private:
    // 当TSDF实际发生场变化而调用方没有同步提供新Feature时使显式特征失效。
    void invalidateFeaturesForFieldChange();

private:
    VoxelShape* m_shape; // 当前会话绑定并独占共享数据的目标体素体。
    VoxelForest* m_forest; // 当前会话独占并修改的TSDF森林。
    VoxelChangeSet m_changes; // 当前会话尚未取走的场和结构变化记录。
};

}

#endif // MYVOXEL_CORE_VOXELSHAPESESSION_H
