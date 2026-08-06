#include "VolumeFieldBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Storage/VolumeBlock.h"
#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Mesh/MeshQuery.h"
#include "MyVoxel/Surface/VoxelSurfaceMesher.h"

namespace
{

const double BlockRadiusScale = 0.86602540378443864676; // 轴对齐立方体包围球半径相对边长的sqrt(3)/2比例。

// 返回网格顶点的局部空间位置。
MyMath::Vector3 vertexPosition(const MyVoxel::Geometry::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 将指定同层级闭区间中的全部体素索引加入集合。
void insertRange(const MyVoxel::VoxelCellRange& range, std::set<MyVoxel::VoxelCellIndex>& indices)
{
    MYVOXEL_ASSERT_MESSAGE(range.isValid(), "VolumeFieldBuilder candidate range must be valid.");

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                indices.insert(MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                       static_cast<MyVoxel::VoxelIndex>(y),
                                                       static_cast<MyVoxel::VoxelIndex>(z)));
            }
        }
    }
}

// 根据每个表面三角形扩展后的窄带包围盒收集候选距离块。
void collectCandidateBlocks(const MyVoxel::Geometry::Mesh& mesh,
                            const MyVoxel::VoxelGrid& grid,
                            MyVoxel::VoxelLevel blockLevel,
                            double bandDistance,
                            std::set<MyVoxel::VoxelCellIndex>& blockIndices)
{
    const std::vector<MyVoxel::Geometry::MeshVertex>& vertices = mesh.vertices();
    const std::vector<std::uint32_t>& indices = mesh.indices();
    const MyMath::Vector3 padding(bandDistance, bandDistance, bandDistance);

    blockIndices.clear();

    for (std::size_t indexPosition = 0; indexPosition < indices.size(); indexPosition += 3)
    {
        const MyMath::Vector3 point0 = vertexPosition(vertices[indices[indexPosition]]);
        const MyMath::Vector3 point1 = vertexPosition(vertices[indices[indexPosition + 1]]);
        const MyMath::Vector3 point2 = vertexPosition(vertices[indices[indexPosition + 2]]);
        MyVoxel::Bounds3 triangleBounds;
        triangleBounds.include(point0);
        triangleBounds.include(point1);
        triangleBounds.include(point2);

        const MyVoxel::Bounds3 expandedBounds(triangleBounds.minimum() - padding, triangleBounds.maximum() + padding);
        insertRange(grid.cellRange(expandedBounds, blockLevel), blockIndices);
    }
}

// 根据材料状态返回截断后的有符号距离。
float truncatedSignedDistance(bool material, double unsignedDistance, double interiorDistance, double exteriorDistance)
{
    return material ? static_cast<float>(-(std::min)(unsignedDistance, interiorDistance))
                    : static_cast<float>((std::min)(unsignedDistance, exteriorDistance));
}

// 计算一个候选块的64个距离样本，并返回是否至少一个样本位于有效窄带内。
bool buildBlock(const MyVoxel::VoxelShape& shape,
                const MyVoxel::Geometry::MeshQuery& query,
                const MyVoxel::VoxelGrid& grid,
                const MyVoxel::VolumeField& field,
                const MyVoxel::VoxelCellAddress& blockAddress,
                double interiorDistance,
                double exteriorDistance,
                MyVoxel::VolumeBlock& block,
                MyVoxel::VolumeFieldBuildStatistics* statistics)
{
    bool intersectsBand = false;

    for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::VolumeBlockSampleCount; ++sampleIndex)
    {
        const MyVoxel::VoxelCellAddress sampleAddress = field.sampleAddress(blockAddress, sampleIndex);
        const double unsignedDistance = query.distanceToPoint(grid.cellCenter(sampleAddress));
        const bool material = shape.state(sampleAddress) == MyVoxel::VoxelState::Material;
        const double bandDistance = material ? interiorDistance : exteriorDistance;

        MyVoxel::volumeDistance(block, sampleIndex) = truncatedSignedDistance(material, unsignedDistance, interiorDistance, exteriorDistance);
        intersectsBand = unsignedDistance <= bandDistance || intersectsBand;

        if (statistics) ++statistics->sampleQueryCount;
    }

    return intersectsBand;
}

}

namespace MyVoxel
{

VolumeFieldBuildStatistics::VolumeFieldBuildStatistics()
{
    clear();
}

void VolumeFieldBuildStatistics::clear()
{
    surfaceTriangleCount = 0;
    candidateBlockCount = 0;
    distanceTestedBlockCount = 0;
    allocatedBlockCount = 0;
    sampleQueryCount = 0;
}

void VolumeFieldBuilder::build(VoxelShape& shape,
                               const VolumeFieldBuildOptions& options,
                               VolumeFieldBuildStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "VolumeFieldBuilder requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(shape.supportsVolumeField(), "VolumeFieldBuilder requires a Shape with maximum level at least 2.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.exteriorBandWidth) && options.exteriorBandWidth > 0.0,
                           "VolumeFieldBuilder exterior band width must be finite and greater than zero.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.interiorBandWidth) && options.interiorBandWidth > 0.0,
                           "VolumeFieldBuilder interior band width must be finite and greater than zero.");

    if (statistics) statistics->clear();

    const VoxelShape& source = shape;
    const VoxelGrid& grid = source.grid();
    const double sampleEdgeLength = grid.minimumCellEdgeLength();
    const double exteriorDistance = options.exteriorBandWidth * sampleEdgeLength;
    const double interiorDistance = options.interiorBandWidth * sampleEdgeLength;
    const double maximumBandDistance = (std::max)(exteriorDistance, interiorDistance);
    VolumeField result(grid.maximumLevel());
    result.setBackgroundDistances(static_cast<float>(-interiorDistance), static_cast<float>(exteriorDistance));

    if (source.isEmpty())
    {
        result.markAllValid();
        VolumeField* target = shape.editVolumeField();
        MYVOXEL_ASSERT_MESSAGE(target, "Supported VoxelShape must provide a writable VolumeField.");
        target->swap(result);
        return;
    }

    const Geometry::Mesh surface = VoxelSurfaceMesher::build(source, Geometry::MeshColor());
    MYVOXEL_ASSERT_MESSAGE(surface.isValid() && !surface.isEmpty(),
                           "Non-empty VoxelShape must produce a valid non-empty boundary mesh.");
    const Geometry::MeshQuery query(surface);
    MYVOXEL_ASSERT_MESSAGE(query.isValid(), "VolumeFieldBuilder failed to create a valid boundary MeshQuery.");

    if (statistics) statistics->surfaceTriangleCount = surface.triangleCount();

    std::set<VoxelCellIndex> candidateBlockIndices;
    collectCandidateBlocks(surface, grid, result.blockLevel(), maximumBandDistance, candidateBlockIndices);

    if (statistics) statistics->candidateBlockCount = candidateBlockIndices.size();

    const double blockRadius = grid.cellEdgeLength(result.blockLevel()) * BlockRadiusScale;

    for (std::set<VoxelCellIndex>::const_iterator iterator = candidateBlockIndices.begin();
         iterator != candidateBlockIndices.end(); ++iterator)
    {
        const VoxelCellAddress blockAddress(*iterator, result.blockLevel());
        const double centerDistance = query.distanceToPoint(grid.cellCenter(blockAddress));

        if (statistics) ++statistics->distanceTestedBlockCount;
        if (centerDistance > maximumBandDistance + blockRadius) continue;

        VolumeBlock block;

        if (!buildBlock(source, query, grid, result, blockAddress, interiorDistance, exteriorDistance,
                        block, statistics))
        {
            continue;
        }

        VolumeBlock& destination = result.ensureBlock(blockAddress, result.exteriorBackgroundDistance());
        destination = block;
    }

    result.markAllValid();
    MYVOXEL_ASSERT_MESSAGE(result.isValid() && result.isCurrent(),
                           "VolumeFieldBuilder produced an invalid or stale distance field.");

    if (statistics) statistics->allocatedBlockCount = result.blockCount();

    VolumeField* target = shape.editVolumeField();
    MYVOXEL_ASSERT_MESSAGE(target, "Supported VoxelShape must provide a writable VolumeField.");
    target->swap(result);

    MYVOXEL_ASSERT_MESSAGE(target->isValid() && target->isCurrent(),
                           "VolumeFieldBuilder failed to commit a valid current distance field.");
}

}