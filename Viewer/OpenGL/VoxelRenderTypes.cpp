#include "VoxelRenderTypes.h"

#include <algorithm>
#include <cassert>

namespace MyVoxelViewer
{

MeshPartIndex::MeshPartIndex()
    : kind(Single)
    , direction(MyVoxel::VoxelFaceDirection::NegativeX)
{
}

MeshPartIndex::MeshPartIndex(const MyVoxel::VoxelCellIndex& rootIndexValue, MyVoxel::VoxelFaceDirection directionValue)
    : kind(RootDirection)
    , rootIndex(rootIndexValue)
    , direction(directionValue)
{
    assert(MyVoxel::isValidVoxelFaceDirection(directionValue));
}




bool MeshPartIndex::operator<(const MeshPartIndex& other) const
{
    if (kind != other.kind)
    {
        return kind < other.kind;
    }

    if (kind == Single)
    {
        return false;
    }

    if (rootIndex != other.rootIndex)
    {
        return rootIndex < other.rootIndex;
    }

    return static_cast<unsigned int>(direction) < static_cast<unsigned int>(other.direction);
}

MeshPartSnapshot::MeshPartSnapshot()
    : version(0)
    , removed(true)
{
}

MeshPartSnapshot::MeshPartSnapshot(const MeshPartIndex& partIndexValue, std::uint64_t versionValue, const MyVoxel::Geometry::Mesh& meshValue)
    : partIndex(partIndexValue)
    , version(versionValue)
    , removed(meshValue.isEmpty())
    , mesh(meshValue)
{
}

MeshObjectSnapshot::MeshObjectSnapshot()
    : objectId(0)
    , kind(MeshObjectKind::Mesh)
    , visible(true)
{
    modelMatrix.setToIdentity();
}

RenderCameraState::RenderCameraState()
    : center(0.0f, 0.0f, 0.0f)
    , viewOffset(0.0f, 0.0f, 0.0f)
    , radius(1.0f)
    , cameraScale(2.8f)
    , yaw(45.0f)
    , pitch(35.264f)
    , viewportSize(1, 1)
{
}

MeshUpdateStatistics::MeshUpdateStatistics()
{
    clear();
}

void MeshUpdateStatistics::clear()
{
    totalMilliseconds = 0.0;
    cacheAndVersionMilliseconds = 0.0;
    cpuStagingCopyMilliseconds = 0.0;
    contextAcquireMilliseconds = 0.0;
    vertexExpansionMilliseconds = 0.0;
    gpuUploadMilliseconds = 0.0;
    gpuRemovalMilliseconds = 0.0;
    contextReleaseMilliseconds = 0.0;
    renderMilliseconds = 0.0;
    uploadedPartCount = 0;
    removedPartCount = 0;
    createdGpuPartCount = 0;
    reusedGpuPartCount = 0;
    stagedCpuPartCount = 0;
    drawCallCount = 0;
    triangleCount = 0;
    frameVersion = 0;
}

void MeshUpdateStatistics::add(const MeshUpdateStatistics& other)
{
    totalMilliseconds += other.totalMilliseconds;
    cacheAndVersionMilliseconds += other.cacheAndVersionMilliseconds;
    cpuStagingCopyMilliseconds += other.cpuStagingCopyMilliseconds;
    contextAcquireMilliseconds += other.contextAcquireMilliseconds;
    vertexExpansionMilliseconds += other.vertexExpansionMilliseconds;
    gpuUploadMilliseconds += other.gpuUploadMilliseconds;
    gpuRemovalMilliseconds += other.gpuRemovalMilliseconds;
    contextReleaseMilliseconds += other.contextReleaseMilliseconds;
    renderMilliseconds += other.renderMilliseconds;
    uploadedPartCount += other.uploadedPartCount;
    removedPartCount += other.removedPartCount;
    createdGpuPartCount += other.createdGpuPartCount;
    reusedGpuPartCount += other.reusedGpuPartCount;
    stagedCpuPartCount += other.stagedCpuPartCount;
    drawCallCount += other.drawCallCount;
    triangleCount += other.triangleCount;
    frameVersion = (std::max)(frameVersion, other.frameVersion);
}

}