#ifndef MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H
#define MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"

class QKeyEvent;
class QMouseEvent;
class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

namespace MyVoxelViewer
{

typedef std::uint64_t MeshObjectId;

// 保存最近一次体素方向分片同步的阶段耗时和资源更新数量。
struct MeshUpdateStatistics
{
    MeshUpdateStatistics();

    // 清空全部阶段耗时和计数。
    void clear();

    // 累加另一次方向分片同步统计。
    void add(const MeshUpdateStatistics& other);

    double totalMilliseconds; // updateRootMeshes入口到提交重绘请求的总耗时。
    double cacheAndVersionMilliseconds; // 缓存查找、方向版本判断、映射维护和包围范围更新耗时。
    double cpuStagingCopyMilliseconds; // OpenGL尚未初始化时暂存CPU Mesh副本的耗时。
    double contextAcquireMilliseconds; // 获取当前OpenGL上下文的耗时。
    double vertexExpansionMilliseconds; // 将索引网格展开为逐顶点OpenGL数据的耗时。
    double gpuUploadMilliseconds; // 创建或复用GPU资源并上传顶点数据的耗时。
    double gpuRemovalMilliseconds; // 删除失效GPU方向分片资源的耗时。
    double contextReleaseMilliseconds; // 释放当前OpenGL上下文的耗时。

    std::size_t uploadedPartCount; // 本次实际上传的非空方向分片数量。
    std::size_t removedPartCount; // 本次实际删除的非空方向分片数量。
    std::size_t createdGpuPartCount; // 本次新建VAO和VBO的方向分片数量。
    std::size_t reusedGpuPartCount; // 本次复用已有VAO和VBO的方向分片数量。
    std::size_t stagedCpuPartCount; // OpenGL尚未初始化时暂存的CPU方向分片数量。
};

// 使用OpenGL 3.3显示多个相互独立的三角网格对象。
//
// 每个对象具有独立模型矩阵、可见状态和一个或多个网格分片。
// 普通连续几何对象只包含一个分片，体素对象按Root与方向保存分片并支持方向级增量更新。
// Mesh中的三角形颜色在上传时展开为逐顶点颜色，CPU侧仍保持逐三角形颜色语义。
// OpenGL初始化后不保留体素方向CPU Mesh副本；上下文被外部重建时应重新调用setMeshCache提交完整缓存。
class VoxelOpenGLWidget : public QOpenGLWidget
{
public:
    explicit VoxelOpenGLWidget(QWidget* parent = nullptr);
    ~VoxelOpenGLWidget() override;

    /// 通用网格对象

    // 返回无效网格对象标识。
    static MeshObjectId invalidMeshObjectId();

    // 添加一个普通三角网格对象并返回对象标识。
    MeshObjectId addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 替换指定普通网格对象的数据，对象不存在或不是普通网格对象时返回false。
    bool setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh);

    // 添加一个支持根级增量更新的体素网格缓存对象并返回对象标识。
    MeshObjectId addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 使用完整缓存替换指定体素网格对象，对象不存在或不是体素网格对象时返回false。
    bool setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache);

    // 根据缓存更新指定体素对象的根集合，对象不存在或不是体素网格对象时返回false。
    bool updateRootMeshes(MeshObjectId objectId,
                          const MyVoxel::VoxelSurfaceCache& cache,
                          const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    // 设置指定对象的模型矩阵，不重新上传网格且不自动适应视图。
    bool setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix);

    // 返回指定对象的模型矩阵，对象不存在时返回空指针。
    const QMatrix4x4* meshObjectMatrix(MeshObjectId objectId) const;

    // 设置指定对象是否参与绘制和场景包围范围计算。
    bool setMeshObjectVisible(MeshObjectId objectId, bool visible);

    // 判断指定对象是否可见，对象不存在时返回false。
    bool isMeshObjectVisible(MeshObjectId objectId) const;

    // 删除指定网格对象及其CPU和GPU资源，对象不存在时返回false。
    bool removeMeshObject(MeshObjectId objectId);

    // 清空全部网格对象及其CPU和GPU资源。
    void clearMeshes();

    // 判断是否存在指定网格对象。
    bool containsMeshObject(MeshObjectId objectId) const;

    // 返回当前网格对象数量。
    std::size_t meshObjectCount() const;

    // 返回指定对象的网格分片数量，对象不存在时返回0。
    std::size_t meshObjectPartCount(MeshObjectId objectId) const;

    // 返回指定对象全部分片的三角形数量，对象不存在时返回0。
    std::size_t meshObjectTriangleCount(MeshObjectId objectId) const;

    // 返回最近一次Root增量更新实际上传或删除的方向分片数量。
    std::size_t lastUpdatedMeshPartCount() const;

    // 返回最近一次Root增量更新的显示同步阶段统计。
    const MeshUpdateStatistics& lastMeshUpdateStatistics() const;

    /// 单体素对象兼容入口

    // 使用完整VoxelSurfaceCache替换默认体素对象并重新适应视图。
    void setMeshCache(const MyVoxel::VoxelSurfaceCache& cache);

    // 根据缓存更新默认体素对象的指定根集合。
    void updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    // 返回默认体素对象当前保存的根网格数量。
    std::size_t rootMeshCount() const;

    // 设置默认体素对象的模型矩阵并重新适应视图。
    void setModelMatrix(const QMatrix4x4& matrix);

    // 返回默认体素对象模型矩阵。
    const QMatrix4x4& modelMatrix() const;

    /// 显示模式

    // 设置是否使用线框模式绘制全部网格对象。
    void setWireframe(bool enabled);

    // 判断当前是否使用线框模式绘制。
    bool isWireframe() const;

    /// 相机控制

    // 重新计算全部可见对象的世界空间包围范围并适应窗口。
    void fitAll();

    // 切换到等轴测观察方向。
    void setIsometricView();

    // 切换到前视方向。
    void setFrontView();

    // 切换到顶视方向。
    void setTopView();

    // 切换到右视方向。
    void setRightView();

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;

    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    enum class MeshObjectKind
    {
        Mesh,
        MeshCache
    };

    struct MeshPartIndex;
    struct RenderMesh;
    struct MeshObject;

    typedef std::map<MeshObjectId, std::unique_ptr<MeshObject>> MeshObjectMap;

    /// 对象管理

    // 创建一个不含网格分片的对象并返回对象标识。
    MeshObjectId createMeshObject(MeshObjectKind kind, const QMatrix4x4& modelMatrix);

    // 返回指定可修改对象，不存在时返回空指针。
    MeshObject* findMeshObject(MeshObjectId objectId);

    // 返回指定只读对象，不存在时返回空指针。
    const MeshObject* findMeshObject(MeshObjectId objectId) const;

    // 使用一个普通网格替换对象全部分片。
    void replaceObjectMesh(MeshObject& object, const MyVoxel::Geometry::Mesh& mesh);

    // 使用完整体素缓存替换对象全部Root方向分片。
    void replaceObjectMeshCache(MeshObject& object, const MyVoxel::VoxelSurfaceCache& cache);

    // 根据体素缓存版本更新对象中指定Root实际变化的方向分片，并返回上传或删除数量。
    std::size_t updateObjectRootMeshes(
        MeshObject& object,
        const MyVoxel::VoxelSurfaceCache& cache,
        const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    /// OpenGL资源

    // 创建背景和网格表面Shader。
    void createShaderPrograms();

    // 创建全屏背景三角形使用的VAO。
    void createBackgroundResources();

    // 将一个CPU网格分片转换并上传到独立GPU缓冲区，已有分片复用原VAO和VBO。
    void uploadMeshPart(MeshObject& object,
                        const MeshPartIndex& partIndex,
                        const MyVoxel::Geometry::Mesh& mesh,
                        MeshUpdateStatistics* statistics = nullptr);

    // 删除指定对象的一个GPU网格分片。
    void removeRenderMesh(MeshObject& object,
                          const MeshPartIndex& partIndex,
                          MeshUpdateStatistics* statistics = nullptr);

    // 删除指定对象的全部GPU网格分片。
    void clearObjectRenderMeshes(MeshObject& object);

    // 删除全部对象的GPU网格分片。
    void clearAllRenderMeshes();

    /// 绘制

    // 判断当前是否存在至少一个可见GPU网格分片。
    bool hasVisibleRenderMeshes() const;

    // 绘制渐变背景。
    void drawBackground();

    // 绘制全部可见网格对象。
    void drawMeshObjects(const QVector3D& eye, const QVector3D& viewCenter);

    /// 相机

    // 根据全部可见CPU网格对象更新世界空间观察包围范围。
    void updateBounds();

    // 返回当前由观察中心指向相机的单位方向。
    QVector3D cameraDirection() const;

    // 返回当前屏幕水平方向。
    QVector3D cameraRight() const;

    // 返回当前屏幕竖直方向。
    QVector3D cameraUp() const;

private:
    bool m_initialized = false; // OpenGL上下文和共享资源是否已经初始化。
    bool m_wireframe = false; // 是否以线框模式绘制全部网格对象。

    QOpenGLFunctions_3_3_Core* m_functions = nullptr; // 当前OpenGL 3.3 Core函数表。
    QOpenGLShaderProgram m_backgroundProgram; // 全屏渐变背景Shader。
    QOpenGLShaderProgram m_meshProgram; // 网格表面光照Shader。
    QOpenGLVertexArrayObject m_backgroundVao; // Core Profile背景绘制使用的空VAO。

    MeshObjectMap m_meshObjects; // 对象标识到独立CPU和GPU网格对象的映射。
    MeshObjectId m_nextMeshObjectId = 1; // 下一个待分配网格对象标识，0保留为无效值。
    MeshObjectId m_defaultVoxelObjectId = 0; // 旧版单体素接口对应的默认对象标识。
    std::size_t m_lastUpdatedMeshPartCount = 0; // 最近一次Root增量更新实际同步的方向分片数量。
    MeshUpdateStatistics m_lastMeshUpdateStatistics; // 最近一次Root增量更新的显示同步阶段统计。
    std::vector<float> m_uploadVertexData; // 方向分片展开为OpenGL逐顶点数据时复用的临时连续缓冲。
    QMatrix4x4 m_defaultModelMatrix; // 默认体素对象尚未创建时仍需保存的兼容模型矩阵。

    QVector3D m_center = QVector3D(0.0f, 0.0f, 0.0f); // 当前可见对象世界空间包围中心。
    QVector3D m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f); // 用户平移产生的观察中心偏移。
    float m_radius = 1.0f; // 当前可见对象世界空间包围球半径。
    float m_cameraScale = 2.8f; // 相机距离相对包围球半径的比例。
    float m_yaw = 45.0f; // 相机绕世界Z轴的方位角，单位为度。
    float m_pitch = 35.264f; // 相机俯仰角，单位为度。

    QPoint m_lastMousePosition; // 上一次鼠标位置。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H