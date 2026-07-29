#ifndef MYVOXEL_VOXELPACKEDFORESTQUERY_H
#define MYVOXEL_VOXELPACKEDFORESTQUERY_H

#include "VoxelRegionQuery.h"
#include "../Tree/VoxelPackedForest.h"
#include "../Tree/VoxelPackedTreeConstCursor.h"
#include "../VoxelGrid.h"

namespace MyVoxel
{

// 指定体素森林区域查询使用的根节点坐标计算方式。
enum class VoxelForestQueryCoordinateMode
{
    GridAware,        // 使用VoxelGrid原点、层级和单元包围盒。
    LegacyZeroOrigin // 使用旧版零原点坐标规则，仅用于性能和兼容性诊断。
};

// 使用Packed森林原有层级结构执行只读空间区域查询。
class VoxelPackedForestQuery
{
public:
    // 绑定指定Packed森林和体素网格，默认使用完整VoxelGrid坐标语义。
    VoxelPackedForestQuery(
        const VoxelPackedForest& forest,
        const VoxelGrid& grid,
        VoxelForestQueryCoordinateMode coordinateMode = VoxelForestQueryCoordinateMode::GridAware);

    // 返回指定局部包围盒与Packed森林材料区域之间的保守关系。
    VoxelRegionRelation classify(const Bounds3& bounds) const;

    // 返回指定局部包围盒与Packed森林材料区域之间的保守关系，并记录查询统计。
    VoxelRegionRelation classify(const Bounds3& bounds, VoxelForestQueryStatistics& statistics) const;

private:
    VoxelRegionRelation classifyImpl(const Bounds3& bounds, VoxelForestQueryStatistics* statistics) const;

    VoxelRegionRelation classifyNode(
        const Bounds3& bounds,
        const Bounds3& nodeBounds,
        const VoxelPackedTreeConstCursor& cursor,
        VoxelForestQueryStatistics* statistics) const;

    // 返回查询包围盒覆盖的第0层根节点索引范围。
    VoxelCellRange queryRootRange(const Bounds3& bounds) const;

    // 返回指定第0层根节点在当前查询坐标模式下的空间包围盒。
    Bounds3 queryRootBounds(const VoxelCellIndex& rootIndex) const;

    // 判断两个有效包围盒是否相交或接触。
    static bool intersects(const Bounds3& first, const Bounds3& second);

    // 判断外部包围盒是否完整包含内部包围盒。
    static bool contains(const Bounds3& outer, const Bounds3& inner);

    // 返回指定父包围盒和角点对应的子包围盒。
    static Bounds3 childBounds(const Bounds3& parentBounds, VoxelCorner corner);

    const VoxelPackedForest* m_forest; // 被查询的只读Packed森林。
    const VoxelGrid* m_grid; // Packed森林使用的局部体素网格。
    VoxelForestQueryCoordinateMode m_coordinateMode; // 当前根节点坐标计算方式。
    double m_baseVoxelEdgeLength; // 第0层体素边长，旧版零原点模式使用。
};

}

#endif // MYVOXEL_VOXELPACKEDFORESTQUERY_H