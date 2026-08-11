#ifndef MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H
#define MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

class VoxelShapeSession;

// 记录一次体素会话实际产生的结构变化根，以及距离场变化覆盖的固定层级区域。
//
// VoxelChangeSet由调用者指定跟踪层级创建空结果容器，实际变化内容只由VoxelShapeSession维护。
// 修改根集合记录距离场或显式结构发生变化的全部第0层根。
// 脏区域只记录距离场变化，不记录仅由split或merge产生的纯结构变化。
class VoxelChangeSet
{
public:
    using RootIndexSet = std::set<VoxelCellIndex>;

    // 保存一个第0层根中发生距离场变化的固定跟踪层级体素位图。
    class DirtyCellRegion
    {
    public:
        /// 状态判断

        // 判断当前根的全部跟踪层级体素是否均被标记为距离场变化。
        bool isFullRoot() const;
        // 判断指定根内局部跟踪层级体素是否被标记为距离场变化。
        bool containsLocalCell(std::size_t localX, std::size_t localY, std::size_t localZ) const;

        /// 区域属性

        // 返回当前脏区域所属的第0层根索引。
        const VoxelCellIndex& rootIndex() const;
        // 返回当前脏区域使用的固定跟踪层级。
        VoxelLevel trackingLevel() const;
        // 返回当前根单轴包含的跟踪层级体素数量。
        std::size_t axisCellCount() const;
        // 返回当前根中被标记为距离场变化的唯一跟踪层级体素数量。
        std::size_t changedCellCount() const;

        /// 区域访问

        // 按XYZ局部顺序追加全部变化体素的根内线性位索引。
        void appendChangedLocalCellIndices(std::vector<std::size_t>& cellIndices) const;
        // 按XYZ局部顺序追加全部变化体素的全局跟踪层级索引。
        void appendChangedCellIndices(std::vector<VoxelCellIndex>& cellIndices) const;

    private:
        friend class VoxelChangeSet;
        friend class VoxelShapeSession;

        // 为指定根和固定跟踪层级创建空距离场变化区域。
        DirtyCellRegion(const VoxelCellIndex& rootIndex, VoxelLevel trackingLevel);
        // 将指定逻辑体素覆盖的全部跟踪层级体素标记为距离场变化。
        void recordFieldChange(const VoxelCellAddress& address);
        // 将当前根的全部跟踪层级体素标记为距离场变化。
        void markFullRoot();
        // 累加同一根、同一跟踪层级的另一个距离场变化区域。
        void accumulate(const DirtyCellRegion& other);
        // 将指定根内局部跟踪层级体素标记为距离场变化。
        void addLocalCell(std::size_t localX, std::size_t localY, std::size_t localZ);
        // 返回指定局部体素对应的线性位索引。
        std::size_t cellBitIndex(std::size_t localX, std::size_t localY, std::size_t localZ) const;
        // 标记指定闭区间内的全部局部跟踪层级体素。
        void markLocalRange(std::size_t minimumX, std::size_t maximumX, std::size_t minimumY, std::size_t maximumY, std::size_t minimumZ, std::size_t maximumZ);
        // 在第一次记录局部距离场变化时创建稠密位图。
        void ensureWords();

    private:
        VoxelCellIndex m_rootIndex; // 当前脏区域所属的第0层根索引。
        VoxelLevel m_trackingLevel; // 当前距离场变化区域使用的固定跟踪层级。
        std::size_t m_axisCellCount; // 当前根单轴包含的跟踪层级体素数量。
        std::size_t m_totalCellCount; // 当前根包含的跟踪层级体素总数量。
        std::size_t m_changedCellCount; // 当前区域中被标记的唯一变化体素数量。
        bool m_fullRoot; // 当前是否直接表示整个根均发生距离场变化。
        std::vector<std::uint64_t> m_changedWords; // 按XYZ局部顺序保存距离场变化位图。
    };

    using DirtyCellRegionMap = std::map<VoxelCellIndex, DirtyCellRegion>;

    // 使用固定跟踪层级创建空变化结果容器。
    explicit VoxelChangeSet(VoxelLevel trackingLevel);

    // 复制只读变化结果。
    VoxelChangeSet(const VoxelChangeSet& other) = default;
    // 移动只读变化结果。
    VoxelChangeSet(VoxelChangeSet&& other);
    // 复制只读变化结果。
    VoxelChangeSet& operator=(const VoxelChangeSet& other) = default;
    // 移动只读变化结果。
    VoxelChangeSet& operator=(VoxelChangeSet&& other);

    /// 状态判断

    // 判断当前是否记录了实际发生变化的根体素。
    bool hasChanges() const;
    // 判断指定第0层根是否发生过距离场或显式结构变化。
    bool containsModifiedRoot(const VoxelCellIndex& rootIndex) const;
    // 判断指定根是否具有距离场变化脏区域。
    bool containsDirtyRegion(const VoxelCellIndex& rootIndex) const;

    /// 记录属性

    // 返回当前距离场变化区域使用的固定跟踪层级。
    VoxelLevel trackingLevel() const;
    // 返回当前记录的实际修改根体素数量。
    std::size_t modifiedRootCount() const;
    // 返回具有距离场变化脏区域的根数量。
    std::size_t dirtyRegionCount() const;

    /// 记录访问

    // 返回全部发生距离场或显式结构变化的第0层根索引。
    const RootIndexSet& modifiedRootIndices() const;
    // 返回指定根的距离场变化脏区域，不存在时返回空指针。
    const DirtyCellRegion* dirtyRegion(const VoxelCellIndex& rootIndex) const;
    // 返回全部距离场变化脏区域。
    const DirtyCellRegionMap& dirtyRegions() const;

private:
    friend class VoxelShapeSession;

    // 记录指定地址所属根发生纯结构变化。
    void recordStructureChange(const VoxelCellAddress& address);
    // 记录指定地址覆盖区域发生距离场变化，并同时记录所属根。
    void recordFieldChange(const VoxelCellAddress& address);
    // 记录指定根的距离场和结构均可能整体发生变化。
    void recordFullRootChange(const VoxelCellIndex& rootIndex);
    // 累加一个由会话创建的同层级Root距离场变化区域。
    void recordDirtyRegion(const DirtyCellRegion& dirtyRegion);
    // 累加同一跟踪层级的另一个会话变化集合。
    void accumulate(const VoxelChangeSet& other);
    // 清空全部变化记录，保留固定跟踪层级。
    void clear();
    // 交换同一跟踪层级的两个变化集合。
    void swap(VoxelChangeSet& other);
    // 返回指定根的距离场变化区域，不存在时创建。
    DirtyCellRegion& ensureDirtyRegion(const VoxelCellIndex& rootIndex);

private:
    VoxelLevel m_trackingLevel; // 当前变化集合使用的固定距离场变化跟踪层级。
    RootIndexSet m_modifiedRootIndices; // 发生距离场或显式结构变化的根索引。
    DirtyCellRegionMap m_dirtyRegions; // 各根在固定层级中的距离场变化区域。
};

}

#endif // MYVOXEL_CORE_CHANGE_VOXELCHANGESET_H