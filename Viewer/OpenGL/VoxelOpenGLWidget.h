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
#include <QQuaternion>
#include <QSize>
#include <QString>
#include <QVector3D>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Display/Object/Display_ObjectUpdate.h"
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

// 使用GUI线程呈现后台共享颜色纹理，并将统一Display_Object同步和场景绘制交给独立OpenGL线程。
class VoxelOpenGLWidget : public QOpenGLWidget
{
public:
    explicit VoxelOpenGLWidget(QWidget* parent = nullptr);
    ~VoxelOpenGLWidget() override;

    /// Display对象提交

    // 提交完整Display_Object快照，同ID已有对象时完整替换。
    bool submitDisplaySnapshot(const MyVoxel::Display_ObjectSnapshot& snapshot);
    // 提交对象资源分片增量更新，支持Mesh和Line Replace/Remove。
    bool submitDisplayPartsUpdate(const MyVoxel::Display_ObjectPartsUpdate& update);
    // 提交对象Usage、变换、可见状态或Line线宽更新。
    bool submitDisplayStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update);
    // 删除指定Display对象及后台GPU资源。
    bool removeDisplayObject(MyVoxel::Display_ObjectId objectId);
    // 清空全部Display对象及后台GPU资源。
    void clearDisplayObjects();

    /// Display对象查询

    bool containsDisplayObject(MyVoxel::Display_ObjectId objectId) const;
    std::size_t displayObjectCount() const;
    std::size_t displayObjectPartCount(MyVoxel::Display_ObjectId objectId) const;
    std::size_t displayObjectTriangleCount(MyVoxel::Display_ObjectId objectId) const;
    std::size_t displayObjectSegmentCount(MyVoxel::Display_ObjectId objectId) const;
    bool isDisplayObjectVisible(MyVoxel::Display_ObjectId objectId) const;
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
    struct PartInfo
    {
        PartInfo();

        std::uint64_t version; // GUI侧最近接受的分片版本，删除后仍保留作为版本墓碑。
        bool active; // 当前分片是否具有活动显示资源。
        std::size_t triangleCount; // Mesh分片三角形数量。
        std::size_t segmentCount; // Line分片独立线段数量。
        MyVoxel::Bounds3 localBounds; // 当前活动资源局部包围盒。
    };

    struct SceneObjectInfo
    {
        typedef std::map<MyVoxel::Display_ObjectPartId, PartInfo> PartMap;

        SceneObjectInfo();

        MyVoxel::Display_ResourceKind resourceKind; // 当前对象统一显示资源类型。
        MyVoxel::Display_ObjectUsage usage; // 当前资源预期GPU更新频率。
        std::uint64_t stateVersion; // GUI侧最近接受的对象状态版本。
        QMatrix4x4 modelMatrix; // 对象局部空间到显示世界空间的模型矩阵。
        bool visible; // 对象是否参与绘制和场景范围计算。
        float lineWidth; // Line对象期望显示线宽。
        MyVoxel::Bounds3 localBounds; // 当前全部活动分片形成的局部轴对齐包围盒。
        PartMap parts; // 当前已知分片状态和版本墓碑。
    };

    typedef std::map<MyVoxel::Display_ObjectId, SceneObjectInfo*> SceneObjectMap;

    /// CPU场景元数据

    SceneObjectInfo* findSceneObjectLocked(MyVoxel::Display_ObjectId objectId);
    const SceneObjectInfo* findSceneObjectLocked(MyVoxel::Display_ObjectId objectId) const;
    static void setPartResourceInfo(PartInfo& part, const MyVoxel::Display_Resource& resource);
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
    QQuaternion m_cameraRotation; // 相机局部坐标系到世界坐标系的完整旋转姿态。
    QSize m_viewportSize;
    QPoint m_lastMousePosition;
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELOPENGLWIDGET_H