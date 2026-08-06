#ifndef MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H
#define MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H

#include <cstdint>
#include <map>
#include <vector>

#include <QEvent>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>

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
    RenderFrameReadyEvent(std::uint64_t frameId, unsigned int textureId, std::uint64_t frameVersion, const QSize& textureSize, const MeshUpdateStatistics& statistics);

    // 返回后台完成帧事件类型。
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

    // 返回后台渲染失败事件类型。
    static QEvent::Type eventType();

    const QString& message() const;

private:
    QString m_message; // 后台共享上下文或OpenGL资源初始化失败信息。
};

// 使用独立共享OpenGL上下文完成网格上传、场景绘制和双纹理发布。
class VoxelRenderThread : public QThread
{
public:
    explicit VoxelRenderThread(QObject* parent = nullptr);
    ~VoxelRenderThread() override;

    /// 生命周期

    // 接收GUI线程创建的共享上下文和离屏表面，将上下文移动到当前线程并启动后台渲染。
    bool startRendering(QOpenGLContext* context, QOffscreenSurface* surface, QObject* frameReceiver, QThread* guiThread);

    // 请求后台线程结束并等待全部OpenGL资源在共享上下文中释放。
    void stopRendering();

    // 判断后台线程是否已经启动且尚未结束。
    bool isRendering() const;

    /// 场景命令

    // 使用完整对象快照替换后台对象，连续提交同一对象时只保留最新完整状态。
    void enqueueReplaceObject(MeshObjectSnapshot snapshot, double cacheMilliseconds = 0.0, double cpuCopyMilliseconds = 0.0);

    // 合并指定对象的分片增量更新，同一分片只保留最后一次提交。
    void enqueueUpdateParts(MeshObjectId objectId, std::vector<MeshPartSnapshot> parts, double cacheMilliseconds = 0.0, double cpuCopyMilliseconds = 0.0);

    // 设置指定对象模型矩阵。
    void enqueueSetObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix);

    // 设置指定对象可见状态。
    void enqueueSetObjectVisible(MeshObjectId objectId, bool visible);

    // 删除指定后台对象。
    void enqueueRemoveObject(MeshObjectId objectId);

    // 清空全部后台对象。
    void enqueueClearObjects();

    /// 绘制状态

    // 提交最新相机和视口状态，尚未处理的旧状态会被直接覆盖。
    void enqueueCameraState(const RenderCameraState& state);

    // 提交最新线框状态。
    void enqueueWireframe(bool enabled);

    // 请求使用当前最新场景状态生成一帧。
    void requestFrame();

    // 通知后台线程GUI已经切换到指定共享纹理，使上一显示纹理可安全复用或释放。
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
        MeshObjectKind kind; // 完整替换时使用的对象类型。
        QMatrix4x4 modelMatrix; // 最新对象模型矩阵。
        bool hasModelMatrix; // 是否提交了模型矩阵修改。
        bool visible; // 最新对象可见状态。
        bool hasVisible; // 是否提交了可见状态修改。
        std::map<MeshPartIndex, MeshPartSnapshot> parts; // 完整替换或增量更新后的最终分片集合。
    };

    struct PendingBatch
    {
        PendingBatch();

        bool clearObjects; // 是否先清空全部后台对象。
        std::map<MeshObjectId, PendingObjectChange> objects; // 按对象合并后的最新场景修改。
        bool hasCamera; // 是否包含新相机状态。
        RenderCameraState camera; // 最新相机和视口状态。
        bool hasWireframe; // 是否包含新线框状态。
        bool wireframe; // 最新线框状态。
        bool requestFrame; // 是否需要在场景修改后生成新帧。
        double cacheMilliseconds; // 调用线程版本过滤和映射维护累计耗时。
        double cpuCopyMilliseconds; // 调用线程CPU Mesh快照累计耗时。
        std::size_t stagedCpuPartCount; // 调用线程生成的非空CPU Mesh快照数量。

        bool isEmpty() const;
        void clear();
    };

    struct RenderState;

    // 将共享命令队列移动到工作线程局部批次。
    PendingBatch takePendingBatch();

    // 判断当前是否存在待处理命令或可立即执行的绘制请求。
    bool hasRunnableWorkLocked() const;

    // 将后台失败信息投递到GUI线程。
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

    QOpenGLContext* m_context; // 已与QOpenGLWidget上下文共享资源且属于后台线程的OpenGL上下文。
    QOffscreenSurface* m_surface; // 在GUI线程创建并供后台上下文绑定的离屏表面。
    QObject* m_frameReceiver; // 接收后台完成帧和失败事件的GUI对象。
    QThread* m_guiThread; // 后台结束前接收OpenGL上下文所有权的GUI线程。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELRENDERTHREAD_H