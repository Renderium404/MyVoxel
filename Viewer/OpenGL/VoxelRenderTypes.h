#ifndef MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H
#define MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <QMatrix4x4>
#include <QSize>
#include <QVector3D>

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"

namespace MyVoxelViewer
{

typedef std::uint64_t MeshObjectId;

// 标识普通连续网格对象和支持Root方向增量更新的体素网格对象。
enum class MeshObjectKind
{
    Mesh,           //网格
    MeshCache       //网格缓存
};

// 标识普通对象唯一分片或体素对象的Root方向分片。
struct MeshPartIndex
{
    enum Kind
    {
        Single,             //唯一分片
        RootDirection       //Root方向分片。
    };

    MeshPartIndex();
    MeshPartIndex(const MyVoxel::VoxelCellIndex& rootIndexValue, MyVoxel::VoxelFaceDirection directionValue);

    // 返回普通网格对象使用的唯一分片标识。
    static MeshPartIndex single()
    {
        return MeshPartIndex();
    }

    // 返回指定Root和方向对应的体素分片标识。
    static MeshPartIndex rootDirection(const MyVoxel::VoxelCellIndex& rootIndex, MyVoxel::VoxelFaceDirection direction)
    {
        return MeshPartIndex(rootIndex, direction);
    }

    bool operator<(const MeshPartIndex& other) const;

    Kind kind;                              // 当前分片标识类型。
    MyVoxel::VoxelCellIndex rootIndex;      // Root方向分片对应的第0层根索引。
    MyVoxel::VoxelFaceDirection direction;  // Root方向分片对应的单位面方向。
};

// 保存提交给后台渲染线程的不可变网格分片快照。
struct MeshPartSnapshot
{
    MeshPartSnapshot();
    MeshPartSnapshot(const MeshPartIndex& partIndexValue, std::uint64_t versionValue, const MyVoxel::Geometry::Mesh& meshValue);

    MeshPartIndex partIndex;                // 当前快照对应的对象分片。
    std::uint64_t version;                  // 当前分片来源版本，普通网格对象使用对象内部递增版本。
    bool removed;                           // 是否删除当前分片。
    MyVoxel::Geometry::Mesh mesh;           // 非删除更新携带的完整不可变CPU网格副本。
};

// 保存完整替换后台网格对象所需的数据快照。
struct MeshObjectSnapshot
{
    MeshObjectSnapshot();

    MeshObjectId objectId;                      // 待替换对象标识。
    MeshObjectKind kind;                        // 待替换对象类型。
    QMatrix4x4 modelMatrix;                     // 对象局部空间到显示世界空间的模型矩阵。
    bool visible;                               // 对象是否参与后台绘制。
    std::vector<MeshPartSnapshot> parts;        // 对象当前全部非空分片。
};

// 保存后台绘制一帧所需的相机和视口状态。
struct RenderCameraState
{
    RenderCameraState();

    QVector3D center;                   // 当前场景观察包围中心。
    QVector3D viewOffset;               // 用户平移产生的观察中心偏移。
    float radius;                       // 当前场景观察包围球半径。
    float cameraScale;                  // 相机距离相对包围球半径的比例。
    float yaw;                          // 相机绕世界Z轴的方位角，单位为度。
    float pitch;                        // 相机俯仰角，单位为度。
    QSize viewportSize;                 // 后台离屏颜色纹理尺寸。
};

// 保存最近一次后台网格同步和完整离屏绘制的阶段耗时与资源计数。
struct MeshUpdateStatistics
{
    MeshUpdateStatistics();

    // 清空全部阶段耗时和计数。
    void clear();

    // 累加另一次后台同步统计。
    void add(const MeshUpdateStatistics& other);

    double totalMilliseconds; // 从CPU快照准备到后台离屏帧完成的已测量总耗时。
    double cacheAndVersionMilliseconds; // 调用线程缓存查找、版本判断和映射维护耗时。
    double cpuStagingCopyMilliseconds; // 调用线程生成不可变Mesh快照的耗时。
    double contextAcquireMilliseconds; // 后台线程首次获取共享OpenGL上下文的耗时。
    double vertexExpansionMilliseconds; // 将索引网格展开为逐顶点OpenGL数据的耗时。
    double gpuUploadMilliseconds; // 创建或复用GPU资源并上传顶点数据的耗时。
    double gpuRemovalMilliseconds; // 删除失效GPU方向分片资源的耗时。
    double contextReleaseMilliseconds; // 后台线程最终释放共享OpenGL上下文的耗时。
    double renderMilliseconds; // 将完整场景绘制到后台FBO并完成GPU同步的耗时。

    std::size_t uploadedPartCount;          // 实际上传的非空分片数量。
    std::size_t removedPartCount;           // 实际删除的非空分片数量。
    std::size_t createdGpuPartCount;        // 新建VAO和VBO的分片数量。
    std::size_t reusedGpuPartCount;         // 复用已有VAO和VBO的分片数量。
    std::size_t stagedCpuPartCount;         // 当前批次生成CPU快照的非空分片数量。
    std::size_t drawCallCount;              // 后台完整场景绘制使用的网格Draw Call数量。
    std::size_t triangleCount;              // 后台完整场景绘制提交的三角形数量。
    std::uint64_t frameVersion;             // 当前统计对应的后台完成帧版本。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELRENDERTYPES_H