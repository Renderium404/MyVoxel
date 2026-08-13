#ifndef MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H
#define MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H

#include <cstdint>
#include <map>

#include <QEvent>
#include <QMatrix4x4>
#include <QMutex>
#include <QString>
#include <QThread>
#include <QWaitCondition>

#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Display/Object/Display_ObjectUpdate.h"
#include "VoxelRenderTypes.h"

class QObject;
class QOffscreenSurface;
class QOpenGLContext;

namespace MyVoxelViewer
{

// 携带后台完成共享颜色纹理和同步统计的GUI线程事件。
class RenderFrameReadyEvent : public QEvent
{
public:
    RenderFrameReadyEvent(std::uint64_t frameId, unsigned int textureId, std::uint64_t frameVersion,
                          const QSize& textureSize, const MeshUpdateStatistics& statistics);

    static QEvent::Type eventType();

    std::uint64_t frameId() const;
    unsigned int textureId() const;
    std::uint64_t frameVersion() const;
    const QSize& textureSize() const;
    const MeshUpdateStatistics& statistics() const;

private:
    std::uint64_t m_frameId; // 后台双缓冲资源的全局唯一标识。
    unsigned int m_textureId; // GUI上下文可直接采样的共享二维颜色纹理。
    std::uint64_t m_frameVersion; // 后台完成帧递增版本。
    QSize m_textureSize; // 当前共享颜色纹理尺寸。
    MeshUpdateStatistics m_statistics; // 当前后台完成帧累计同步统计。
};

// 携带后台OpenGL初始化失败信息的GUI线程事件。
class RenderFailureEvent : public QEvent
{
public:
    explicit RenderFailureEvent(const QString& message);

    static QEvent::Type eventType();
    const QString& message() const;

private:
    QString m_message; // 后台共享上下文或OpenGL资源初始化失败信息。
};

// 使用独立共享OpenGL上下文消费统一Display_Object命令并完成GPU同步和双纹理发布。
class VoxelRenderThread : public QThread
{
public:
    explicit VoxelRenderThread(QObject* parent = nullptr);
    ~VoxelRenderThread() override;

    /// 生命周期

    bool startRendering(QOpenGLContext* context, QOffscreenSurface* surface, QObject* frameReceiver, QThread* guiThread);
    void stopRendering();
    bool isRendering() const;

    /// Display场景命令

    // 使用完整Display_Object快照替换后台对象。
    void enqueueReplaceObject(const MyVoxel::Display_ObjectSnapshot& snapshot,
                              double cacheMilliseconds = 0.0, double cpuCopyMilliseconds = 0.0);
    // 合并一个Display_Object的资源分片增量更新。
    void enqueueUpdateParts(const MyVoxel::Display_ObjectPartsUpdate& update,
                            double cacheMilliseconds = 0.0, double cpuCopyMilliseconds = 0.0);
    // 合并Display_Object的Usage、变换、可见状态和Line线宽更新。
    void enqueueStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update);
    // 删除指定后台Display对象。
    void enqueueRemoveObject(MyVoxel::Display_ObjectId objectId);
    // 清空全部后台Display对象。
    void enqueueClearObjects();

    /// 绘制状态

    void enqueueCameraState(const RenderCameraState& state);
    void enqueueWireframe(bool enabled);
    void requestFrame();
    void acceptPresentedFrame(std::uint64_t frameId, std::uint64_t frameVersion);

protected:
    void run() override;

private:
    enum PendingObjectAction
    {
        PendingNone,
        PendingReplace,
        PendingRemove
    };

    struct PendingObjectChange
    {
        PendingObjectChange();

        PendingObjectAction action; // 当前对象待执行的结构操作。
        MyVoxel::Display_ResourceKind resourceKind; // 当前资源分片统一类型。
        bool hasResourceKind; // 当前命令是否包含资源类型约束。
        std::uint64_t stateVersion; // 当前合并后的最新对象状态版本。
        bool hasStateVersion; // 是否包含对象状态修改或完整快照。
        bool dynamicUsage; // GPU缓冲是否按动态资源使用。
        bool hasUsage; // 是否修改Usage。
        QMatrix4x4 modelMatrix; // 最新对象模型矩阵。
        bool hasModelMatrix; // 是否提交了模型矩阵修改。
        bool visible; // 最新对象可见状态。
        bool hasVisible; // 是否提交了可见状态修改。
        float lineWidth; // Line对象期望OpenGL线宽。
        bool hasLineWidth; // 是否修改Line线宽。
        std::map<MyVoxel::Display_ObjectPartId, MyVoxel::Display_ObjectPartUpdate> parts; // 按partId和版本合并后的最终资源修改。
    };

    struct PendingBatch
    {
        PendingBatch();

        bool clearObjects; // 是否先清空全部后台对象。
        std::map<MyVoxel::Display_ObjectId, PendingObjectChange> objects; // 按对象合并后的最新场景修改。
        bool hasCamera; // 是否包含新相机状态。
        RenderCameraState camera; // 最新相机和视口状态。
        bool hasWireframe; // 是否包含新Mesh线框状态。
        bool wireframe; // 最新Mesh线框状态。
        bool requestFrame; // 是否需要在场景修改后生成新帧。
        double cacheMilliseconds; // 调用线程版本过滤和映射维护累计耗时。
        double cpuCopyMilliseconds; // 调用线程不可变资源引用提交累计耗时。
        std::size_t stagedCpuPartCount; // 当前批次提交的非空资源数量。

        bool isEmpty() const;
        void clear();
    };

    struct RenderState;

    PendingBatch takePendingBatch();
    bool hasRunnableWorkLocked() const;
    void postFailure(const QString& message) const;

private:
    mutable QMutex m_mutex; // 保护生命周期、待处理命令和共享帧握手状态。
    QWaitCondition m_waitCondition; // 后台线程等待新命令、停止请求或GUI帧确认。
    PendingBatch m_pending; // 尚未由后台线程取走的最新合并命令。
    bool m_stopRequested; // 是否请求后台线程结束。
    bool m_started; // 后台线程是否已经启动。
    bool m_readyFrameOutstanding; // 是否存在尚未被GUI确认的共享纹理帧。
    bool m_frameAcceptancePending; // 是否存在尚未由后台场景处理的GUI帧确认。
    std::uint64_t m_readyFrameId; // 尚未被GUI确认的共享纹理资源标识。
    std::uint64_t m_readyFrameVersion; // 尚未被GUI确认的后台帧版本。
    std::uint64_t m_acceptedFrameId; // GUI最近确认显示的共享纹理资源标识。
    std::uint64_t m_acceptedFrameVersion; // GUI最近确认显示的后台帧版本。

    QOpenGLContext* m_context; // 与QOpenGLWidget共享资源且属于后台线程的OpenGL上下文。
    QOffscreenSurface* m_surface; // 在GUI线程创建并供后台上下文绑定的离屏表面。
    QObject* m_frameReceiver; // 接收后台完成帧和失败事件的GUI对象。
    QThread* m_guiThread; // 后台结束前接收OpenGL上下文所有权的GUI线程。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H