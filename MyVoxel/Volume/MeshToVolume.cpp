#include "MeshToVolume.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Mesh/MeshQuery.h"

namespace
{

// 算法结构参考OpenVDB MeshToVolume。
// Copyright Contributors to the OpenVDB Project.
// OpenVDB源代码使用Apache License 2.0。

const double RangePaddingVoxelCount = 2.0; // 等值面提取时在窄带之外额外保留两层采样。

}

namespace MyVoxel
{
namespace Conversion
{

LevelSetVolume MeshToVolume::convert(const Geometry::Mesh& mesh,
                                     const VoxelGrid& grid,
                                     const MeshToVolumeOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "MeshToVolume requires a valid Geometry::Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!mesh.isEmpty(), "MeshToVolume requires a non-empty Geometry::Mesh.");
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "MeshToVolume requires a valid VoxelGrid.");

    MYVOXEL_ASSERT_MESSAGE(
        std::isfinite(options.exteriorBandWidth) && options.exteriorBandWidth > 0.0,
        "MeshToVolume exterior band width must be finite and greater than zero.");

    MYVOXEL_ASSERT_MESSAGE(
        std::isfinite(options.interiorBandWidth) && options.interiorBandWidth > 0.0,
        "MeshToVolume interior band width must be finite and greater than zero.");

    const Geometry::MeshQuery query(mesh);

    MYVOXEL_ASSERT_MESSAGE(query.isValid(), "MeshToVolume failed to create a valid MeshQuery.");

    const Bounds3& meshBounds = query.queryBounds();
    const double voxelSize = grid.minimumCellEdgeLength();
    const double maximumBandWidth = (std::max)(options.exteriorBandWidth, options.interiorBandWidth);
    const double padding = (maximumBandWidth + RangePaddingVoxelCount) * voxelSize;

    const MyMath::Vector3 paddingVector(padding, padding, padding);
    const Bounds3 expandedBounds(meshBounds.minimum() - paddingVector, meshBounds.maximum() + paddingVector);
    const VoxelCellRange sampleRange = grid.cellRange(expandedBounds, grid.maximumLevel());

    const double exteriorDistance = options.exteriorBandWidth * voxelSize;
    const double interiorDistance = options.interiorBandWidth * voxelSize;

    LevelSetVolume volume(grid, sampleRange, static_cast<float>(exteriorDistance));

    for (std::int64_t z = static_cast<std::int64_t>(sampleRange.minimum.z);
         z <= static_cast<std::int64_t>(sampleRange.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(sampleRange.minimum.y);
             y <= static_cast<std::int64_t>(sampleRange.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(sampleRange.minimum.x);
                 x <= static_cast<std::int64_t>(sampleRange.maximum.x);
                 ++x)
            {
                const VoxelCellIndex index(
                    static_cast<VoxelIndex>(x),
                    static_cast<VoxelIndex>(y),
                    static_cast<VoxelIndex>(z));

                const MyMath::Vector3 point = volume.samplePosition(index);
                const double unsignedDistance = query.distanceToPoint(point);

                double value = (std::min)(unsignedDistance, exteriorDistance);

                if (!options.unsignedDistance && query.containsPoint(point))
                {
                    value = -(std::min)(unsignedDistance, interiorDistance);
                }

                volume.setValue(index, static_cast<float>(value));
            }
        }
    }

    return volume;
}

}
}