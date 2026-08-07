#include "VoxelRenderTypes.h"

#include <algorithm>

namespace MyVoxelViewer
{

QMatrix4x4 toQMatrix4x4(const MyMath::Matrix4& source)
{
    QMatrix4x4 result;

    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            result(row, column) = static_cast<float>(source.value(row, column));
        }
    }

    return result;
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
    lineDrawCallCount = 0;
    lineSegmentCount = 0;
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
    lineDrawCallCount += other.lineDrawCallCount;
    lineSegmentCount += other.lineSegmentCount;
    frameVersion = (std::max)(frameVersion, other.frameVersion);
}

}
