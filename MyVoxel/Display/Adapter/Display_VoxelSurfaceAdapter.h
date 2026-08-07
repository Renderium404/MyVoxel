#ifndef MYVOXEL_DISPLAY_ADAPTER_DISPLAY_VOXELSURFACEADAPTER_H
#define MYVOXEL_DISPLAY_ADAPTER_DISPLAY_VOXELSURFACEADAPTER_H

#include <cstddef>
#include <cstdint>
#include <map>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshUpdate.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"

namespace MyVoxel
{

// 将VoxelSurfaceCache的Root方向网格转换为通用Display全量快照和增量更新。
//
// 每个非空Root方向网格对应一个稳定的Display_MeshPartId。
// Part ID由适配器内部单调分配，显示后端不需要理解VoxelCellIndex和VoxelFaceDirection。
// 适配器只在方向Mesh版本变化、方向从空变为非空或从非空变为空时创建显示更新。
class Display_VoxelSurfaceAdapter
{
public:
    // 为指定非零显示对象创建体素表面适配器。
    explicit Display_VoxelSurfaceAdapter(Display_MeshObjectId objectId);

    /// 状态管理

    // 清空全部Root方向映射和已缓存显示资源，保留显示对象标识。
    void clear();
    // 判断适配器是否已经通过完整快照建立当前前端状态。
    bool isInitialized() const;
    // 返回目标显示对象标识。
    Display_MeshObjectId objectId() const;
    // 返回已经分配过稳定Part ID的Root方向数量，包括当前已删除方向。
    std::size_t partCount() const;
    // 返回当前具有非空显示资源的Root方向数量。
    std::size_t activePartCount() const;
    // 返回指定Root方向已经分配的稳定Part ID，尚未分配时返回零。
    Display_MeshPartId partId(const VoxelCellIndex& rootIndex, VoxelFaceDirection direction) const;

    /// 全量显示数据

    // 根据完整表面缓存建立对象全量快照，并同步适配器内部Root方向状态。
    Display_MeshObjectSnapshot buildSnapshot(const VoxelSurfaceCache& cache, const MyMath::Matrix4& localToWorld,
                                             bool visible);

    /// 增量显示数据

    // 根据表面缓存更新结果转换实际变化Root，只提交版本发生变化的方向。
    Display_MeshUpdate buildUpdate(const VoxelSurfaceCache& cache, const VoxelSurfaceCacheUpdate& surfaceUpdate);
    // 根据颜色修改等接口返回的变化Root集合，只提交版本发生变化的方向。
    Display_MeshUpdate buildUpdate(const VoxelSurfaceCache& cache,
                                   const VoxelSurfaceCache::RootIndexSet& changedRootIndices);

private:
    // 唯一标识一个体素Root中的一个方向网格。
    struct PartKey
    {
        PartKey();
        PartKey(const VoxelCellIndex& rootIndexValue, VoxelFaceDirection directionValue);

        bool operator<(const PartKey& other) const;

        VoxelCellIndex rootIndex; // 当前分片所属第0层Root索引。
        VoxelFaceDirection direction; // 当前分片对应的外表面方向。
    };

    // 保存一个稳定Part ID及其当前显示资源状态。
    struct PartState
    {
        PartState();

        Display_MeshPartId partId; // 当前Root方向稳定显示分片标识。
        std::uint64_t sourceVersion; // 最近一次转换的VoxelSurfaceCache方向Mesh版本。
        std::uint64_t displayVersion; // 发送给显示后端的单调版本。
        bool active; // 当前方向是否具有非空显示资源。
        Foundation::RefPtr<const Display_MeshResource> resource; // 当前方向不可变CPU显示资源。
    };

    typedef std::map<PartKey, PartState> PartStateMap;

    // 返回或创建指定Root方向状态，并为新状态分配稳定Part ID。
    static PartState& ensurePartState(PartStateMap& states, Display_MeshPartId& nextPartId, const PartKey& key);
    // 根据非空可渲染Mesh替换状态资源并推进显示版本。
    static void replacePartResource(PartState& state, const Mesh& mesh, std::uint64_t sourceVersion);
    // 推进非零显示版本，极端溢出时回到一。
    static void advanceDisplayVersion(std::uint64_t& version);

private:
    Display_MeshObjectId m_objectId; // 当前适配器对应的全局显示对象标识。
    Display_MeshPartId m_nextPartId; // 下一个可分配稳定分片标识，零表示标识空间已经耗尽。
    bool m_initialized; // 是否已经通过完整快照建立前端状态。
    PartStateMap m_parts; // Root方向与稳定显示分片状态映射。
};

}

#endif // MYVOXEL_DISPLAY_ADAPTER_DISPLAY_VOXELSURFACEADAPTER_H
