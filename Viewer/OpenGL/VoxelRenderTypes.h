#ifndef MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H
#define MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H

#include <cstddef>
#include <cstdint>

#include <QMatrix4x4>
#include <QQuaternion>
#include <QSize>
#include <QVector3D>

#include "MyMath/Matrix4.h"

namespace MyVoxelViewer
{

// 将MyMath行列访问矩阵转换为Qt矩阵。
QMatrix4x4 toQMatrix4x4(const MyMath::Matrix4& source);

// 保存后台绘制一帧所需的相机和视口状态。
struct RenderCameraState
{
    RenderCameraState();

    QVector3D center;       // 当前场景观察包围中心。
    QVector3D viewOffset;   // 用户平移产生的观察中心偏移。
    float radius;           // 当前场景观察包围球半径。
    float cameraScale;      // 当前相机距离相对包围球半径的比例。
    QQuaternion rotation;   // 相机局部坐标系到世界坐标系的完整旋转姿态。
    QSize viewportSize;     // 后台离屏颜色纹理尺寸。
};

// 保存最近一次后台Display资源同步和完整离屏绘制的阶段耗时与资源计数。
struct MeshUpdateStatistics
{
    MeshUpdateStatistics();

    // 清空全部阶段耗时和计数。
    void clear();
    // 累加另一次后台同步统计。
    void add(const MeshUpdateStatistics& other);

    double totalMilliseconds; // 从显示命令合并到后台离屏帧完成的已测量总耗时。
    double cacheAndVersionMilliseconds; // 调用线程显示对象查找、版本过滤和轻量元数据维护耗时。
    double cpuStagingCopyMilliseconds; // 调用线程提交不可变Display资源引用的耗时。
    double contextAcquireMilliseconds; // 后台线程首次获取共享OpenGL上下文的耗时。
    double vertexExpansionMilliseconds; // 保留兼容统计项，Display资源当前已经完成顶点展开。
    double gpuUploadMilliseconds; // 创建或复用GPU资源并上传顶点数据的耗时。
    double gpuRemovalMilliseconds; // 删除失效GPU分片资源的耗时。
    double contextReleaseMilliseconds; // 后台线程最终释放共享OpenGL上下文的耗时。
    double renderMilliseconds; // 将完整场景绘制到后台FBO并完成GPU同步的耗时。

    std::size_t uploadedPartCount; // 实际上传的Mesh和Line非空分片总数。
    std::size_t removedPartCount; // 实际删除的GPU分片数量。
    std::size_t createdGpuPartCount; // 新建VAO和VBO的分片数量。
    std::size_t reusedGpuPartCount; // 复用已有VAO和VBO的分片数量。
    std::size_t stagedCpuPartCount; // 当前批次提交的非空不可变Display资源数量。
    std::size_t drawCallCount; // 后台完整场景绘制使用的总Draw Call数量。
    std::size_t triangleCount; // 后台完整场景绘制提交的Mesh三角形数量。
    std::size_t lineDrawCallCount; // 后台完整场景绘制使用的Line Draw Call数量。
    std::size_t lineSegmentCount; // 后台完整场景绘制提交的独立线段数量。
    std::uint64_t frameVersion; // 当前统计对应的后台完成帧版本。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H