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

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Line/Display_LineSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshUpdate.h"
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

// 使用GUI线程呈现后台共享颜色纹理，并将Mesh和Line Display对象同步及场景绘制交给独立OpenGL线程。
class VoxelOpenGLWidget : public QOpenGLWidget
{
public:
    explicit VoxelOpenGLWidget(QWidget* parent = nullptr);
    ~VoxelOpenGLWidget() override;

    /// Display对象提交

    // 提交完整Mesh对象快照；同ID已有Line对象时会完整替换。
    bool submitDisplaySnapshot(const MyVoxel::Display_MeshObjectSnapshot& snapshot);
    // 提交完整Line对象快照；同ID已有Mesh对象时会完整替换。
    bool submitDisplaySnapshot(const MyVoxel::Display_LineObjectSnapshot& snapshot);
    // 提交Mesh对象分片增量更新；Line对象不接受此接口。
    bool submitDisplayUpdate(const MyVoxel::Display_MeshUpdate& update);
    // 提交对象变换或可见状态更新；当前接口沿用Mesh状态对象，但可作用于同ID Line对象。
    bool submitDisplayStateUpdate(const MyVoxel::Display_MeshStateUpdate& update);
    // 删除指定Display对象及后台GPU资源。
    bool removeDisplayObject(std::uint64_t objectId);
    // 清空全部Mesh和Line Display对象及后台GPU资源。
    void clearDisplayObjects();

    /// Display对象查询

    bool containsDisplayObject(std::uint64_t objectId) const;
    std::size_t displayObjectCount() const;
    std::size_t displayObjectPartCount(std::uint64_t objectId) const;
    std::size_t displayObjectTriangleCount(std::uint64_t objectId) const;
    std::size_t displayObjectSegmentCount(std::uint64_t objectId) const;
    bool isDisplayObjectVisible(std::uint64_t objectId) const;
    std::size_t lastUpdatedMeshPartCount() const;
    const MeshUpdateStatistics& lastMeshUpdateStatistics() const;
    const QString& backgroundRenderError() const;

    /// 显示模式

    void setWireframe(bool enabled);
    bool isWireframe() const;

    /// 相机控制

    void fitAll();
    void setIsometricView();
    void setFrontView();
    void setTopView();
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
    enum SceneObjectKind
    {
        SceneMeshObject,
        SceneLineObject
    };

    struct PartInfo
    {
        PartInfo();

        std::uint64_t version; // GUI侧最近接受的分片版本。
        bool active; // 当前分片是否具有非空显示资源。
        std::size_t triangleCount; // Mesh分片三角形数量。
        std::size_t segmentCount; // Line分片独立线段数量。
        MyVoxel::Bounds3 localBounds; // 当前活动资源局部包围盒。
    };

    struct SceneObjectInfo
    {
        typedef std::map<std::uint64_t, PartInfo> PartMap;

        SceneObjectInfo();

        SceneObjectKind kind; // 当前对象显示资源类型。
        QMatrix4x4 modelMatrix; // 对象局部空间到显示世界空间的模型矩阵。
        bool visible; // 对象是否参与绘制和场景范围计算。
        MyVoxel::Bounds3 localBounds; // 当前全部活动分片形成的局部轴对齐包围盒。
        PartMap parts; // 当前已知分片版本和活动资源元数据。
    };

    typedef std::map<std::uint64_t, SceneObjectInfo*> SceneObjectMap;

    /// CPU场景元数据

    SceneObjectInfo* findSceneObjectLocked(std::uint64_t objectId);
    const SceneObjectInfo* findSceneObjectLocked(std::uint64_t objectId) const;
    static void rebuildObjectBounds(SceneObjectInfo& object);

    /// 后台OpenGL生命周期

    bool createPresentResources();
    bool startBackgroundRenderer();
    void stopBackgroundRenderer();
    void cleanupOpenGL();

    /// 相机和场景范围

    void updateBounds();
    void submitCameraState();
    void requestBoundsUpdate();
    QVector3D cameraDirection() const;
    QVector3D cameraRight() const;
    QVector3D cameraUp() const;

private:
    mutable QMutex m_sceneMutex;
    SceneObjectMap m_sceneObjects;
    std::size_t m_lastUpdatedMeshPartCount;
    MeshUpdateStatistics m_lastMeshUpdateStatistics;
    QString m_backgroundRenderError;
    QAtomicInt m_sceneBoundsEventPending;

    VoxelRenderThread* m_renderThread;
    QOpenGLContext* m_renderContext;
    QOffscreenSurface* m_offscreenSurface;

    bool m_glInitialized;
    bool m_wireframe;
    QOpenGLFunctions_3_3_Core* m_functions;
    QOpenGLShaderProgram m_presentProgram;
    QOpenGLVertexArrayObject m_presentVao;
    unsigned int m_presentTextureId;
    std::uint64_t m_presentFrameId;
    std::uint64_t m_presentFrameVersion;
    QSize m_presentTextureSize;

    QVector3D m_center;
    QVector3D m_viewOffset;
    float m_radius;
    float m_cameraScale;
    float m_yaw;
    float m_pitch;
    QSize m_viewportSize;
    QPoint m_lastMousePosition;
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H
