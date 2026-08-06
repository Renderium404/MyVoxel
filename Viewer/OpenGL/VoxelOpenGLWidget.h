#ifndef MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H
#define MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H

#include <cstddef>
#include <cstdint>
#include <map>

#include <QAtomicInt>
#include <QMatrix4x4>
#include <QMutex>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSize>
#include <QString>
#include <QVector3D>

#include "VoxelRenderThread.h"
#include "VoxelRenderTypes.h"

class QKeyEvent;
class QMouseEvent;
class QOffscreenSurface;
class QOpenGLContext;
class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

namespace MyVoxelViewer
{

// 使用GUI线程呈现后台共享颜色纹理，并将网格同步和完整场景绘制交给独立OpenGL线程。
//
// 普通网格和体素增量网格修改接口只生成不可变CPU快照并提交后台队列，不执行GUI OpenGL调用。
// 调用方必须保证传入Mesh或VoxelSurfaceCache在对应接口返回前不被其他线程并发修改。
// 相机、窗口生命周期和最终纹理呈现接口仍由当前控件所属GUI线程调用。
// QOpenGLWidget上下文被外部重建后，调用方必须重新提交普通网格和完整VoxelSurfaceCache。
class VoxelOpenGLWidget : public QOpenGLWidget
{
public:
    explicit VoxelOpenGLWidget(QWidget* parent = nullptr);
    ~VoxelOpenGLWidget() override;

    /// 通用网格对象

    // 返回无效网格对象标识。
    static MeshObjectId invalidMeshObjectId();

    // 添加一个普通三角网格对象并异步提交后台渲染，返回对象标识。
    MeshObjectId addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 异步替换指定普通网格对象，对象不存在或不是普通网格对象时返回false。
    bool setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh);

    // 添加一个支持Root方向增量更新的体素网格缓存对象并异步提交后台渲染。
    MeshObjectId addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 使用完整缓存异步替换指定体素网格对象，对象不存在或不是体素网格对象时返回false。
    bool setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache);

    // 根据缓存版本生成指定Root的不可变分片快照并异步提交后台更新。
    bool updateRootMeshes(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    // 异步设置指定对象模型矩阵，不重新上传网格且不自动适应视图。
    bool setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix);

    // 返回指定对象模型矩阵，对象不存在时返回空指针；不得与对象删除并发调用。
    const QMatrix4x4* meshObjectMatrix(MeshObjectId objectId) const;

    // 将指定对象模型矩阵复制到输出参数，对象不存在时返回false。
    bool meshObjectMatrix(MeshObjectId objectId, QMatrix4x4& matrix) const;

    // 异步设置指定对象是否参与绘制和场景包围范围计算。
    bool setMeshObjectVisible(MeshObjectId objectId, bool visible);

    // 判断指定对象是否可见，对象不存在时返回false。
    bool isMeshObjectVisible(MeshObjectId objectId) const;

    // 异步删除指定网格对象及其后台GPU资源，对象不存在时返回false。
    bool removeMeshObject(MeshObjectId objectId);

    // 异步清空全部网格对象及其后台GPU资源。
    void clearMeshes();

    // 判断是否存在指定网格对象。
    bool containsMeshObject(MeshObjectId objectId) const;

    // 返回当前网格对象数量。
    std::size_t meshObjectCount() const;

    // 返回指定对象当前非空网格分片数量，对象不存在时返回0。
    std::size_t meshObjectPartCount(MeshObjectId objectId) const;

    // 返回指定对象全部非空分片的三角形数量，对象不存在时返回0。
    std::size_t meshObjectTriangleCount(MeshObjectId objectId) const;

    // 返回最近一次Root增量提交实际包含的上传或删除分片数量。
    std::size_t lastUpdatedMeshPartCount() const;

    // 返回最近一张后台完成帧累计的网格同步和离屏绘制统计。
    const MeshUpdateStatistics& lastMeshUpdateStatistics() const;

    // 返回最近一次后台OpenGL初始化或运行失败信息，无错误时返回空字符串。
    const QString& backgroundRenderError() const;

    /// 单体素对象兼容入口

    // 使用完整VoxelSurfaceCache异步替换默认体素对象并重新适应视图。
    void setMeshCache(const MyVoxel::VoxelSurfaceCache& cache);

    // 根据缓存异步更新默认体素对象的指定根集合。
    void updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    // 返回默认体素对象当前保存的Root数量。
    std::size_t rootMeshCount() const;

    // 设置默认体素对象模型矩阵并重新适应视图。
    void setModelMatrix(const QMatrix4x4& matrix);

    // 返回默认体素对象模型矩阵。
    const QMatrix4x4& modelMatrix() const;

    /// 显示模式

    // 异步设置是否使用线框模式绘制全部网格对象。
    void setWireframe(bool enabled);

    // 判断当前是否使用线框模式绘制。
    bool isWireframe() const;

    /// 相机控制

    // 重新计算全部可见对象的世界空间包围范围并异步提交适应窗口后的相机。
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
    bool event(QEvent* event) override;
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;

    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct SceneObjectInfo
    {
        typedef std::map<MeshPartIndex, std::uint64_t> PartVersionMap;
        typedef std::map<MeshPartIndex, std::size_t> PartTriangleCountMap;

        SceneObjectInfo(MeshObjectKind kindValue, const QMatrix4x4& modelMatrixValue);

        MeshObjectKind kind; // 普通连续网格对象或体素增量网格对象。
        QMatrix4x4 modelMatrix; // 对象局部空间到显示世界空间的模型矩阵。
        bool visible; // 对象是否参与绘制和场景范围计算。
        MyVoxel::Bounds3 localBounds; // 当前对象全部分片形成的局部轴对齐包围盒。
        PartVersionMap partVersions; // 当前GUI侧已经提交的分片版本。
        PartTriangleCountMap partTriangleCounts; // 当前GUI侧非空分片三角形数量。
        std::uint64_t nextMeshVersion; // 普通网格完整替换使用的递增版本。
    };

    typedef std::map<MeshObjectId, SceneObjectInfo*> SceneObjectMap;

    /// CPU场景和快照

    // 创建不含网格分片的GUI侧对象并返回对象标识，调用方必须持有场景互斥锁。
    MeshObjectId createSceneObjectLocked(MeshObjectKind kind, const QMatrix4x4& modelMatrix);

    // 返回指定GUI侧可修改对象，调用方必须持有场景互斥锁。
    SceneObjectInfo* findSceneObjectLocked(MeshObjectId objectId);

    // 返回指定GUI侧只读对象，调用方必须持有场景互斥锁。
    const SceneObjectInfo* findSceneObjectLocked(MeshObjectId objectId) const;

    // 建立普通网格对象完整快照并同步GUI侧元数据，调用方必须持有场景互斥锁。
    MeshObjectSnapshot buildMeshSnapshotLocked(MeshObjectId objectId, SceneObjectInfo& object, const MyVoxel::Geometry::Mesh& mesh);

    // 建立体素缓存对象完整快照并同步GUI侧元数据，调用方必须持有场景互斥锁。
    MeshObjectSnapshot buildMeshCacheSnapshotLocked(MeshObjectId objectId, SceneObjectInfo& object, const MyVoxel::VoxelSurfaceCache& cache);

    /// 后台OpenGL生命周期

    // 创建GUI线程最终纹理呈现Shader和VAO。
    bool createPresentResources();

    // 创建共享OpenGL上下文和离屏表面并启动后台渲染线程。
    bool startBackgroundRenderer();

    // 结束后台渲染线程并在GUI线程释放共享上下文和离屏表面。
    void stopBackgroundRenderer();

    // 释放当前QOpenGLWidget上下文相关资源，可由上下文销毁信号重复调用。
    void cleanupOpenGL();

    /// 相机和场景范围

    // 根据全部可见GUI侧对象更新世界空间观察包围范围。
    void updateBounds();

    // 将当前相机和视口快照提交后台线程。
    void submitCameraState();

    // 在非GUI线程修改场景时向GUI事件队列合并提交一次包围范围更新。
    void requestBoundsUpdate();

    // 返回当前由观察中心指向相机的单位方向。
    QVector3D cameraDirection() const;

    // 返回当前屏幕水平方向。
    QVector3D cameraRight() const;

    // 返回当前屏幕竖直方向。
    QVector3D cameraUp() const;

private:
    mutable QMutex m_sceneMutex; // 保护GUI侧对象元数据和异步统计。
    SceneObjectMap m_sceneObjects; // 对象标识到GUI侧轻量场景元数据的映射。
    MeshObjectId m_nextMeshObjectId; // 下一个待分配网格对象标识，0保留为无效值。
    MeshObjectId m_defaultVoxelObjectId; // 旧版单体素接口对应的默认对象标识。
    std::size_t m_lastUpdatedMeshPartCount; // 最近一次Root增量提交的实际分片数量。
    MeshUpdateStatistics m_lastMeshUpdateStatistics; // 最近一张后台完成帧累计同步统计。
    QString m_backgroundRenderError; // 最近一次后台OpenGL失败信息。
    QAtomicInt m_sceneBoundsEventPending; // 是否已经向GUI队列提交场景范围更新事件。

    VoxelRenderThread* m_renderThread; // 独立共享OpenGL上下文后台线程。
    QOpenGLContext* m_renderContext; // 与当前QOpenGLWidget上下文共享资源的后台上下文。
    QOffscreenSurface* m_offscreenSurface; // 在GUI线程创建并供后台上下文使用的离屏表面。

    bool m_glInitialized; // GUI呈现Shader、VAO和后台线程是否完成初始化。
    bool m_wireframe; // 当前提交后台的线框状态。
    QOpenGLFunctions_3_3_Core* m_functions; // GUI当前OpenGL 3.3 Core函数表。
    QOpenGLShaderProgram m_presentProgram; // 将后台共享颜色纹理绘制到QOpenGLWidget的Shader。
    QOpenGLVertexArrayObject m_presentVao; // Core Profile全屏纹理绘制使用的空VAO。
    unsigned int m_presentTextureId; // 当前GUI显示的后台共享颜色纹理。
    std::uint64_t m_presentFrameId; // 当前GUI确认显示的后台帧资源标识。
    std::uint64_t m_presentFrameVersion; // 当前GUI确认显示的后台帧版本。
    QSize m_presentTextureSize; // 当前GUI显示共享纹理尺寸。

    QMatrix4x4 m_defaultModelMatrix; // 默认体素对象尚未创建时仍需保存的兼容模型矩阵。
    QVector3D m_center; // 当前可见对象世界空间包围中心。
    QVector3D m_viewOffset; // 用户平移产生的观察中心偏移。
    float m_radius; // 当前可见对象世界空间包围球半径。
    float m_cameraScale; // 相机距离相对包围球半径的比例。
    float m_yaw; // 相机绕世界Z轴的方位角，单位为度。
    float m_pitch; // 相机俯仰角，单位为度。
    QSize m_viewportSize; // 后台离屏纹理目标尺寸。
    QPoint m_lastMousePosition; // 上一次鼠标位置。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H