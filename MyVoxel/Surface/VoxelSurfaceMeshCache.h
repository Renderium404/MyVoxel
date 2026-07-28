#ifndef MYVOXEL_VOXELSURFACEMESHCACHE_H
#define MYVOXEL_VOXELSURFACEMESHCACHE_H

#include <cstddef>
#include <map>
#include <set>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Surface/VoxelSurfaceMesh.h"
#include "MyVoxel/Core/VoxelChangeSet.h"
namespace MyVoxel
{

// 按第0层根索引缓存贪心体素表面网格，并支持局部根网格重建。
class VoxelSurfaceMeshCache
{
public:
    using RootIndexSet = VoxelChangeSet::RootIndexSet;
    using MeshMap = std::map<VoxelCellIndex, VoxelSurfaceMesh>;

    VoxelSurfaceMeshCache();

    /// 缓存管理

    // 清空全部根网格和缓存来源信息。
    void clear();

    // 根据完整体素形体重新建立全部根网格缓存。
    void rebuild(const VoxelShape& shape, const VoxelSurfaceColor& color);

    // 根据实际修改根集合重建修改根及其六个面邻根，返回本次处理的全部脏根。
    // 根据实际体素修改集合重建修改根及其六个面邻根，返回本次处理的全部脏根。
    RootIndexSet update(const VoxelShape& shape, const VoxelChangeSet& changes, const VoxelSurfaceColor& color);

    /// 缓存状态

    // 返回缓存是否尚未建立或当前没有任何根网格。
    bool isEmpty() const;

    // 返回缓存是否已经根据某个体素形体建立。
    bool isInitialized() const;

    // 返回当前缓存的非空根网格数量。
    std::size_t meshCount() const;

    // 返回全部根网格的顶点数量。
    std::size_t vertexCount() const;

    // 返回全部根网格的三角形数量。
    std::size_t triangleCount() const;

    /// 根网格访问

    // 检查指定根索引是否存在非空网格。
    bool contains(const VoxelCellIndex& rootIndex) const;

    // 返回指定根索引对应的网格，不存在时返回空指针。
    const VoxelSurfaceMesh* mesh(const VoxelCellIndex& rootIndex) const;

    // 返回全部根网格。
    const MeshMap& meshes() const;

    // 将全部根网格按根索引顺序拼接为一个完整网格。
    VoxelSurfaceMesh combinedMesh() const;

private:
    // 检查指定形体配置和颜色是否与当前缓存来源一致。
    bool isCompatible(const VoxelShape& shape, const VoxelSurfaceColor& color) const;

    // 保存当前缓存使用的形体配置和颜色。
    void setSource(const VoxelShape& shape, const VoxelSurfaceColor& color);

    // 收集完整形体中实际包含材料的第0层根索引。
    static RootIndexSet collectMaterialRootIndices(const VoxelShape& shape);

    // 将指定根及其有效六个面邻根加入脏根集合。
    static void appendRootAndFaceNeighbors(RootIndexSet& rootIndices, const VoxelCellIndex& rootIndex);

    // 使用指定根索引重建或删除一个缓存网格。
    void rebuildRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndex, const VoxelSurfaceColor& color);

    MeshMap m_meshes; // 第0层根索引与非空根网格的对应关系。
    double m_baseVoxelEdgeLength = 1.0; // 当前缓存来源的第0层体素边长。
    VoxelLevel m_maximumLevel = BaseVoxelLevel; // 当前缓存来源的最高细分层级。
    MyMath::Matrix4 m_transform; // 当前缓存来源的体素局部坐标到世界坐标变换。
    VoxelSurfaceColor m_color; // 当前缓存网格使用的顶点颜色。
    bool m_initialized = false; // 当前是否已经保存有效缓存来源信息。
};

}

#endif // MYVOXEL_VOXELSURFACEMESHCACHE_H