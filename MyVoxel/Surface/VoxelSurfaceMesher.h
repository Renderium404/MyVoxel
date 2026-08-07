#ifndef MYVOXEL_SURFACE_VOXELSURFACEMESHER_H
#define MYVOXEL_SURFACE_VOXELSURFACEMESHER_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Mesh/Mesh.h"

#include "VoxelFaceColorMap.h"
#include "VoxelFaceSet.h"
#include "VoxelRootFaceMasks.h"

namespace MyVoxel
{


// 保存一个Root中按方向和切片分组的稀疏单位面颜色。
//
// 数据由Root局部VoxelFaceColorMap一次性转换，方向并行任务只读取自己的切片颜色范围。
class VoxelRootSurfaceColorData
{
public:
    // 保存方向切片中的一个单独上色单位面。
    struct Cell
    {
        Cell();
        Cell(std::size_t uValue, std::size_t vValue, const Display_Color& colorValue);

        std::size_t u; // 当前方向平面内第一坐标。
        std::size_t v; // 当前方向平面内第二坐标。
        Display_Color color; // 当前单位面的单独颜色。
    };

    using CellContainer = std::vector<Cell>;
    using ConstIterator = CellContainer::const_iterator;

    VoxelRootSurfaceColorData();

    // 根据Root面掩码和Root局部颜色映射重新建立方向切片颜色数据。
    void build(const VoxelRootFaceMasks& faceMasks, const VoxelFaceColorMap& colors);

    // 判断颜色数据是否已经绑定到一个Root分辨率。
    bool isInitialized() const;

    // 返回全部方向中的单独颜色数量。
    std::size_t colorCount() const;

    // 返回指定方向切片颜色范围的起始迭代器。
    ConstIterator begin(VoxelFaceDirection direction, std::size_t slice) const;

    // 返回指定方向切片颜色范围的末尾迭代器。
    ConstIterator end(VoxelFaceDirection direction, std::size_t slice) const;

private:
    struct DirectionData
    {
        CellContainer cells; // 当前方向按切片连续保存的单独颜色。
        std::vector<std::size_t> sliceOffsets; // 每个切片在cells中的起止偏移。
    };

    std::size_t m_axisCellCount; // 当前Root单轴最高层体素数量。
    DirectionData m_directions[VoxelFaceDirectionCount]; // 六个方向的切片颜色数据。
};

// 保存一次Root或单方向贪心网格构建的阶段耗时和结果规模。
struct VoxelSurfaceMeshingStatistics
{
    VoxelSurfaceMeshingStatistics();

    // 清空全部阶段耗时和计数。
    void clear();

    // 累加另一次Root或单方向网格构建统计。
    void add(const VoxelSurfaceMeshingStatistics& other);

    double facePlaneBuildMilliseconds; // 为方向切片准备活动位图和单位面颜色的耗时。
    double greedyMergeMilliseconds; // 使用活动位行贪心合并同色矩形并写入Mesh的耗时。
    std::size_t sourceFaceCount; // 当前构建消费的单位面数量。
    std::size_t nonEmptyPlaneCount; // 当前构建实际处理的非空方向切片数量。
    std::size_t mergedQuadCount; // 当前构建输出的贪心四边形数量。
};

// 从VoxelShape识别最高层单位外表面，并按根、方向、平面和颜色执行贪心合并生成局部空间三角网格。
//
// 自动构建接口负责提取体素外表面；已有面集合接口保留通用兼容路径；
// Root方向面掩码接口直接使用提取阶段生成的切片位图，避免单位面重新分桶。
// 生成的顶点使用VoxelGrid局部坐标，不应用VoxelShape::transform。
// 完整构建按第0层根分别合并，不跨根生成四边形，便于根级表面缓存独立更新。
class VoxelSurfaceMesher
{
public:
    /// 自动提取完整表面

    // 使用统一默认颜色提取并构建当前体素形体的完整局部空间表面网格。
    static Mesh build(const VoxelShape& shape, const Display_Color& defaultColor);

    // 使用稀疏单位面颜色和默认颜色提取并构建当前体素形体的完整局部空间表面网格。
    static Mesh build(const VoxelShape& shape,
                                const VoxelFaceColorMap& colors,
                                const Display_Color& defaultColor);

    /// 已有面集合完整构建

    // 将全部体素面按所属第0层根分别构建并合并为一个完整网格。
    static Mesh build(const VoxelShape& shape,
                                const VoxelFaceSet& faces,
                                const VoxelFaceColorMap& colors,
                                const Display_Color& defaultColor);

    /// 自动提取根级表面

    // 使用统一默认颜色提取并构建指定第0层根拥有的局部空间表面网格。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const Display_Color& defaultColor);

    // 使用稀疏单位面颜色和默认颜色提取并构建指定第0层根拥有的局部空间表面网格。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const VoxelFaceColorMap& colors,
                                    const Display_Color& defaultColor);

    /// 已有面集合根级构建

    // 构建指定第0层根拥有的体素面网格，faces中的全部面必须属于该根。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const VoxelFaceSet& faces,
                                    const VoxelFaceColorMap& colors,
                                    const Display_Color& defaultColor);

    // 构建指定第0层根拥有的体素面网格，并返回阶段统计。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const VoxelFaceSet& faces,
                                    const VoxelFaceColorMap& colors,
                                    const Display_Color& defaultColor,
                                    VoxelSurfaceMeshingStatistics* statistics);

    /// 已有Root面掩码构建

    // 直接使用Root方向面掩码构建局部网格，不再执行面地址到方向平面的重新分桶。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const VoxelRootFaceMasks& faceMasks,
                                    const VoxelFaceColorMap& colors,
                                    const Display_Color& defaultColor);

    // 直接使用Root方向面掩码构建局部网格，并返回阶段统计。
    static Mesh buildRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    const VoxelRootFaceMasks& faceMasks,
                                    const VoxelFaceColorMap& colors,
                                    const Display_Color& defaultColor,
                                    VoxelSurfaceMeshingStatistics* statistics);

    /// 单方向Root面掩码构建

    // 只构建指定Root方向的局部网格，用于Root与方向扁平并行调度。
    static Mesh buildRootDirection(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceDirection direction,
                                             const VoxelRootFaceMasks& faceMasks,
                                             const VoxelFaceColorMap& colors,
                                             const Display_Color& defaultColor);

    // 只构建指定Root方向的局部网格，并返回该方向的阶段统计。
    static Mesh buildRootDirection(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceDirection direction,
                                             const VoxelRootFaceMasks& faceMasks,
                                             const VoxelFaceColorMap& colors,
                                             const Display_Color& defaultColor,
                                             VoxelSurfaceMeshingStatistics* statistics);

    // 使用已经按Root准备的颜色数据构建指定方向，避免方向任务重复查询面颜色。
    static Mesh buildRootDirection(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceDirection direction,
                                             const VoxelRootFaceMasks& faceMasks,
                                             const VoxelRootSurfaceColorData& colorData,
                                             const Display_Color& defaultColor);

    // 使用已经按Root准备的颜色数据构建指定方向，并返回该方向的阶段统计。
    static Mesh buildRootDirection(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceDirection direction,
                                             const VoxelRootFaceMasks& faceMasks,
                                             const VoxelRootSurfaceColorData& colorData,
                                             const Display_Color& defaultColor,
                                             VoxelSurfaceMeshingStatistics* statistics);

    // 使用历史四边形数量预留方向临时Mesh容量，并返回该方向的阶段统计。
    static Mesh buildRootDirection(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceDirection direction,
                                             const VoxelRootFaceMasks& faceMasks,
                                             const VoxelRootSurfaceColorData& colorData,
                                             const Display_Color& defaultColor,
                                             std::size_t reserveQuadCount,
                                             VoxelSurfaceMeshingStatistics* statistics);

private:
    VoxelSurfaceMesher() = delete;
};

}

#endif // MYVOXEL_SURFACE_VOXELSURFACEMESHER_H
