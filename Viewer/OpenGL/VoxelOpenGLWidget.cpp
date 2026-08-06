#include "VoxelOpenGLWidget.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <QCoreApplication>
#include <QDebug>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QSurfaceFormat>
#include <QThread>
#include <QWheelEvent>

namespace
{

const float Pi = 3.14159265358979323846f; // 角度与弧度转换使用的圆周率。
const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const float MinimumCameraScale = 0.35f; // 滚轮缩放允许的最小相机距离比例。
const float MaximumCameraScale = 50.0f; // 滚轮缩放允许的最大相机距离比例。

typedef std::chrono::steady_clock SnapshotClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const SnapshotClock::time_point& begin, const SnapshotClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 将浮点值限制在指定闭区间。
float clampFloat(float value, float minimum, float maximum)
{
    return (std::max)(minimum, (std::min)(value, maximum));
}

// 将角度转换为弧度。
float degreeToRadian(float degree)
{
    return degree * Pi / 180.0f;
}

// 将局部轴对齐包围盒的八个角点变换到世界空间并扩展场景范围。
void expandWorldBounds(const MyVoxel::Bounds3& localBounds, const QMatrix4x4& modelMatrix, bool& hasPoint, QVector3D& minimum, QVector3D& maximum)
{
    if (!localBounds.isValid())
    {
        return;
    }

    const MyMath::Vector3& localMinimum = localBounds.minimum();
    const MyMath::Vector3& localMaximum = localBounds.maximum();
    const float x[2] = {static_cast<float>(localMinimum.x()), static_cast<float>(localMaximum.x())};
    const float y[2] = {static_cast<float>(localMinimum.y()), static_cast<float>(localMaximum.y())};
    const float z[2] = {static_cast<float>(localMinimum.z()), static_cast<float>(localMaximum.z())};

    for (unsigned int xIndex = 0; xIndex < 2; ++xIndex)
    {
        for (unsigned int yIndex = 0; yIndex < 2; ++yIndex)
        {
            for (unsigned int zIndex = 0; zIndex < 2; ++zIndex)
            {
                const QVector3D point = modelMatrix.map(QVector3D(x[xIndex], y[yIndex], z[zIndex]));

                if (!hasPoint)
                {
                    minimum = point;
                    maximum = point;
                    hasPoint = true;
                    continue;
                }

                minimum.setX((std::min)(minimum.x(), point.x()));
                minimum.setY((std::min)(minimum.y(), point.y()));
                minimum.setZ((std::min)(minimum.z(), point.z()));
                maximum.setX((std::max)(maximum.x(), point.x()));
                maximum.setY((std::max)(maximum.y(), point.y()));
                maximum.setZ((std::max)(maximum.z(), point.z()));
            }
        }
    }
}

// 返回GUI侧场景范围更新事件类型。
QEvent::Type sceneBoundsChangedEventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

}

namespace MyVoxelViewer
{

VoxelOpenGLWidget::SceneObjectInfo::SceneObjectInfo(MeshObjectKind kindValue, const QMatrix4x4& modelMatrixValue)
    : kind(kindValue)
    , modelMatrix(modelMatrixValue)
    , visible(true)
    , nextMeshVersion(1)
{
}

VoxelOpenGLWidget::VoxelOpenGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_nextMeshObjectId(1)
    , m_defaultVoxelObjectId(0)
    , m_lastUpdatedMeshPartCount(0)
    , m_sceneBoundsEventPending(0)
    , m_renderThread(new VoxelRenderThread())
    , m_renderContext(nullptr)
    , m_offscreenSurface(nullptr)
    , m_glInitialized(false)
    , m_wireframe(false)
    , m_functions(nullptr)
    , m_presentTextureId(0)
    , m_presentFrameId(0)
    , m_presentFrameVersion(0)
    , m_center(0.0f, 0.0f, 0.0f)
    , m_viewOffset(0.0f, 0.0f, 0.0f)
    , m_radius(1.0f)
    , m_cameraScale(2.8f)
    , m_yaw(45.0f)
    , m_pitch(35.264f)
    , m_viewportSize(1, 1)
{
    // OpenGL 3.3 Core提供共享纹理、VAO、现代Shader和稳定的跨平台顶点接口。
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(0); // 多重采样由后台FBO扩展阶段决定，当前保持共享颜色纹理直接可采样。
    setFormat(format);

    m_defaultModelMatrix.setToIdentity();
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(false);
    setMinimumSize(480, 320);
}

VoxelOpenGLWidget::~VoxelOpenGLWidget()
{
    cleanupOpenGL();

    {
        QMutexLocker locker(&m_sceneMutex);

        for (SceneObjectMap::iterator iterator = m_sceneObjects.begin(); iterator != m_sceneObjects.end(); ++iterator)
        {
            delete iterator->second;
        }

        m_sceneObjects.clear();
    }

    delete m_renderThread;
    m_renderThread = nullptr;
}

/// 通用网格对象

MeshObjectId VoxelOpenGLWidget::invalidMeshObjectId()
{
    return static_cast<MeshObjectId>(0);
}

MeshObjectId VoxelOpenGLWidget::addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix)
{
    assert(mesh.isValid());
    const SnapshotClock::time_point begin = SnapshotClock::now();
    MeshObjectSnapshot snapshot;
    MeshObjectId objectId = invalidMeshObjectId();

    {
        QMutexLocker locker(&m_sceneMutex);
        objectId = createSceneObjectLocked(MeshObjectKind::Mesh, modelMatrix);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);
        assert(object);
        snapshot = buildMeshSnapshotLocked(objectId, *object, mesh);
    }

    const double copyMilliseconds = elapsedMilliseconds(begin, SnapshotClock::now());
    m_renderThread->enqueueReplaceObject(std::move(snapshot), 0.0, copyMilliseconds);
    requestBoundsUpdate();
    return objectId;
}

bool VoxelOpenGLWidget::setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh)
{
    assert(mesh.isValid());
    const SnapshotClock::time_point begin = SnapshotClock::now();
    MeshObjectSnapshot snapshot;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);

        if (!object || object->kind != MeshObjectKind::Mesh)
        {
            return false;
        }

        snapshot = buildMeshSnapshotLocked(objectId, *object, mesh);
    }

    const double copyMilliseconds = elapsedMilliseconds(begin, SnapshotClock::now());
    m_renderThread->enqueueReplaceObject(std::move(snapshot), 0.0, copyMilliseconds);
    requestBoundsUpdate();
    return true;
}

MeshObjectId VoxelOpenGLWidget::addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix)
{
    const SnapshotClock::time_point begin = SnapshotClock::now();
    MeshObjectSnapshot snapshot;
    MeshObjectId objectId = invalidMeshObjectId();

    {
        QMutexLocker locker(&m_sceneMutex);
        objectId = createSceneObjectLocked(MeshObjectKind::MeshCache, modelMatrix);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);
        assert(object);
        snapshot = buildMeshCacheSnapshotLocked(objectId, *object, cache);
    }

    const double copyMilliseconds = elapsedMilliseconds(begin, SnapshotClock::now());
    m_renderThread->enqueueReplaceObject(std::move(snapshot), 0.0, copyMilliseconds);
    requestBoundsUpdate();
    return objectId;
}

bool VoxelOpenGLWidget::setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache)
{
    const SnapshotClock::time_point begin = SnapshotClock::now();
    MeshObjectSnapshot snapshot;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);

        if (!object || object->kind != MeshObjectKind::MeshCache)
        {
            return false;
        }

        snapshot = buildMeshCacheSnapshotLocked(objectId, *object, cache);
    }

    const double copyMilliseconds = elapsedMilliseconds(begin, SnapshotClock::now());
    m_renderThread->enqueueReplaceObject(std::move(snapshot), 0.0, copyMilliseconds);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::updateRootMeshes(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    const SnapshotClock::time_point totalStart = SnapshotClock::now();
    double copyMilliseconds = 0.0;
    std::vector<MeshPartSnapshot> snapshots;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);

        if (!object || object->kind != MeshObjectKind::MeshCache)
        {
            m_lastUpdatedMeshPartCount = 0;
            return false;
        }

        snapshots.reserve(changedRootIndices.size() * MyVoxel::VoxelFaceDirectionCount);

        for (MyVoxel::VoxelSurfaceCache::RootIndexSet::const_iterator rootIterator = changedRootIndices.begin(); rootIterator != changedRootIndices.end(); ++rootIterator)
        {
            const MyVoxel::VoxelSurfaceCache::RootEntry* rootEntry = cache.rootEntry(*rootIterator);

            for (unsigned int directionValue = 0; directionValue < MyVoxel::VoxelFaceDirectionCount; ++directionValue)
            {
                const MyVoxel::VoxelFaceDirection direction = static_cast<MyVoxel::VoxelFaceDirection>(directionValue);
                const MeshPartIndex partIndex = MeshPartIndex::rootDirection(*rootIterator, direction);

                if (!rootEntry)
                {
                    const bool hadPart = object->partTriangleCounts.erase(partIndex) > 0;
                    object->partVersions.erase(partIndex);

                    if (hadPart)
                    {
                        MeshPartSnapshot snapshot;
                        snapshot.partIndex = partIndex;
                        snapshot.removed = true;
                        snapshots.push_back(snapshot);
                    }

                    continue;
                }

                const std::uint64_t currentVersion = rootEntry->directionMeshVersions[directionValue];
                const SceneObjectInfo::PartVersionMap::const_iterator versionIterator = object->partVersions.find(partIndex);

                if (versionIterator != object->partVersions.end() && versionIterator->second == currentVersion)
                {
                    continue;
                }

                object->partVersions[partIndex] = currentVersion;
                const MyVoxel::Geometry::Mesh& directionMesh = rootEntry->directionMeshes[directionValue];

                if (directionMesh.isEmpty())
                {
                    const bool hadPart = object->partTriangleCounts.erase(partIndex) > 0;

                    if (hadPart)
                    {
                        MeshPartSnapshot snapshot;
                        snapshot.partIndex = partIndex;
                        snapshot.version = currentVersion;
                        snapshot.removed = true;
                        snapshots.push_back(snapshot);
                    }

                    continue;
                }

                object->partTriangleCounts[partIndex] = directionMesh.triangleCount();
                const SnapshotClock::time_point copyStart = SnapshotClock::now();
                snapshots.push_back(MeshPartSnapshot(partIndex, currentVersion, directionMesh));
                copyMilliseconds += elapsedMilliseconds(copyStart, SnapshotClock::now());
            }
        }

        object->localBounds = cache.localBounds();
        m_lastUpdatedMeshPartCount = snapshots.size();
    }

    const double totalMilliseconds = elapsedMilliseconds(totalStart, SnapshotClock::now());
    const double cacheMilliseconds = (std::max)(0.0, totalMilliseconds - copyMilliseconds);

    if (!snapshots.empty())
    {
        m_renderThread->enqueueUpdateParts(objectId, std::move(snapshots), cacheMilliseconds, copyMilliseconds);
    }

    return true;
}

bool VoxelOpenGLWidget::setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix)
{
    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);

        if (!object)
        {
            return false;
        }

        object->modelMatrix = matrix;
    }

    m_renderThread->enqueueSetObjectMatrix(objectId, matrix);
    return true;
}

const QMatrix4x4* VoxelOpenGLWidget::meshObjectMatrix(MeshObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);
    return object ? &object->modelMatrix : nullptr;
}

bool VoxelOpenGLWidget::meshObjectMatrix(MeshObjectId objectId, QMatrix4x4& matrix) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object)
    {
        return false;
    }

    matrix = object->modelMatrix;
    return true;
}

bool VoxelOpenGLWidget::setMeshObjectVisible(MeshObjectId objectId, bool visible)
{
    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(objectId);

        if (!object)
        {
            return false;
        }

        if (object->visible == visible)
        {
            return true;
        }

        object->visible = visible;
    }

    m_renderThread->enqueueSetObjectVisible(objectId, visible);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::isMeshObjectVisible(MeshObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);
    return object && object->visible;
}

bool VoxelOpenGLWidget::removeMeshObject(MeshObjectId objectId)
{
    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectMap::iterator iterator = m_sceneObjects.find(objectId);

        if (iterator == m_sceneObjects.end())
        {
            return false;
        }

        delete iterator->second;
        m_sceneObjects.erase(iterator);

        if (m_defaultVoxelObjectId == objectId)
        {
            m_defaultVoxelObjectId = invalidMeshObjectId();
        }
    }

    m_renderThread->enqueueRemoveObject(objectId);
    requestBoundsUpdate();
    return true;
}

void VoxelOpenGLWidget::clearMeshes()
{
    {
        QMutexLocker locker(&m_sceneMutex);
        for (SceneObjectMap::iterator iterator = m_sceneObjects.begin(); iterator != m_sceneObjects.end(); ++iterator)
        {
            delete iterator->second;
        }

        m_sceneObjects.clear();
        m_defaultVoxelObjectId = invalidMeshObjectId();
        m_lastUpdatedMeshPartCount = 0;
    }

    m_renderThread->enqueueClearObjects();

    if (QThread::currentThread() == thread())
    {
        fitAll();
    }
    else
    {
        requestBoundsUpdate();
    }
}

bool VoxelOpenGLWidget::containsMeshObject(MeshObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    return findSceneObjectLocked(objectId) != nullptr;
}

std::size_t VoxelOpenGLWidget::meshObjectCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    return m_sceneObjects.size();
}

std::size_t VoxelOpenGLWidget::meshObjectPartCount(MeshObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);
    return object ? object->partTriangleCounts.size() : 0;
}

std::size_t VoxelOpenGLWidget::meshObjectTriangleCount(MeshObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object)
    {
        return 0;
    }

    std::size_t triangleCount = 0;

    for (SceneObjectInfo::PartTriangleCountMap::const_iterator iterator = object->partTriangleCounts.begin(); iterator != object->partTriangleCounts.end(); ++iterator)
    {
        triangleCount += iterator->second;
    }

    return triangleCount;
}

std::size_t VoxelOpenGLWidget::lastUpdatedMeshPartCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    return m_lastUpdatedMeshPartCount;
}

const MeshUpdateStatistics& VoxelOpenGLWidget::lastMeshUpdateStatistics() const
{
    return m_lastMeshUpdateStatistics;
}

const QString& VoxelOpenGLWidget::backgroundRenderError() const
{
    return m_backgroundRenderError;
}

/// 单体素对象兼容入口

void VoxelOpenGLWidget::setMeshCache(const MyVoxel::VoxelSurfaceCache& cache)
{
    MeshObjectId objectId = invalidMeshObjectId();

    {
        QMutexLocker locker(&m_sceneMutex);
        objectId = m_defaultVoxelObjectId;
    }

    if (objectId == invalidMeshObjectId())
    {
        objectId = addMeshCache(cache, m_defaultModelMatrix);
        QMutexLocker locker(&m_sceneMutex);
        m_defaultVoxelObjectId = objectId;
    }
    else
    {
        const bool replaced = setMeshCache(objectId, cache);
        assert(replaced);
    }

    if (QThread::currentThread() == thread())
    {
        fitAll();
    }
}

void VoxelOpenGLWidget::updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    MeshObjectId objectId = invalidMeshObjectId();

    {
        QMutexLocker locker(&m_sceneMutex);
        objectId = m_defaultVoxelObjectId;
    }

    if (objectId == invalidMeshObjectId())
    {
        setMeshCache(cache);
        return;
    }

    const bool updated = updateRootMeshes(objectId, cache, changedRootIndices);
    assert(updated);
}

std::size_t VoxelOpenGLWidget::rootMeshCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(m_defaultVoxelObjectId);

    if (!object || object->kind != MeshObjectKind::MeshCache)
    {
        return 0;
    }

    std::size_t rootCount = 0;
    bool hasPreviousRoot = false;
    MyVoxel::VoxelCellIndex previousRoot;

    for (SceneObjectInfo::PartVersionMap::const_iterator iterator = object->partVersions.begin(); iterator != object->partVersions.end(); ++iterator)
    {
        if (iterator->first.kind != MeshPartIndex::RootDirection)
        {
            continue;
        }

        if (!hasPreviousRoot || iterator->first.rootIndex != previousRoot)
        {
            previousRoot = iterator->first.rootIndex;
            hasPreviousRoot = true;
            ++rootCount;
        }
    }

    return rootCount;
}

void VoxelOpenGLWidget::setModelMatrix(const QMatrix4x4& matrix)
{
    m_defaultModelMatrix = matrix;
    MeshObjectId objectId = invalidMeshObjectId();

    {
        QMutexLocker locker(&m_sceneMutex);
        objectId = m_defaultVoxelObjectId;
    }

    if (objectId != invalidMeshObjectId())
    {
        const bool updated = setMeshObjectMatrix(objectId, matrix);
        assert(updated);
    }

    fitAll();
}

const QMatrix4x4& VoxelOpenGLWidget::modelMatrix() const
{
    return m_defaultModelMatrix;
}

/// 显示模式

void VoxelOpenGLWidget::setWireframe(bool enabled)
{
    if (m_wireframe == enabled)
    {
        return;
    }

    m_wireframe = enabled;
    m_renderThread->enqueueWireframe(enabled);
}

bool VoxelOpenGLWidget::isWireframe() const
{
    return m_wireframe;
}

/// 相机控制

void VoxelOpenGLWidget::fitAll()
{
    assert(QThread::currentThread() == thread());
    updateBounds();
    m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f);
    m_cameraScale = 2.8f;
    submitCameraState();
}

void VoxelOpenGLWidget::setIsometricView()
{
    m_yaw = 45.0f;
    m_pitch = 35.264f;
    fitAll();
}

void VoxelOpenGLWidget::setFrontView()
{
    m_yaw = 90.0f;
    m_pitch = 0.0f;
    fitAll();
}

void VoxelOpenGLWidget::setTopView()
{
    m_yaw = 90.0f;
    m_pitch = 89.0f;
    fitAll();
}

void VoxelOpenGLWidget::setRightView()
{
    m_yaw = 0.0f;
    m_pitch = 0.0f;
    fitAll();
}

/// Qt事件和OpenGL生命周期

bool VoxelOpenGLWidget::event(QEvent* event)
{
    if (event->type() == RenderFrameReadyEvent::eventType())
    {
        RenderFrameReadyEvent* frameEvent = static_cast<RenderFrameReadyEvent*>(event);
        m_presentTextureId = frameEvent->textureId();
        m_presentFrameId = frameEvent->frameId();
        m_presentFrameVersion = frameEvent->frameVersion();
        m_presentTextureSize = frameEvent->textureSize();

        {
            QMutexLocker locker(&m_sceneMutex);
            m_lastMeshUpdateStatistics = frameEvent->statistics();
            m_backgroundRenderError.clear();
        }

        m_renderThread->acceptPresentedFrame(m_presentFrameId, m_presentFrameVersion);
        update();
        return true;
    }

    if (event->type() == RenderFailureEvent::eventType())
    {
        RenderFailureEvent* failureEvent = static_cast<RenderFailureEvent*>(event);

        {
            QMutexLocker locker(&m_sceneMutex);
            m_backgroundRenderError = failureEvent->message();
        }

        qWarning() << "MyVoxel background OpenGL renderer:" << failureEvent->message();
        update();
        return true;
    }

    if (event->type() == sceneBoundsChangedEventType())
    {
        m_sceneBoundsEventPending.store(0);
        updateBounds();
        submitCameraState();
        return true;
    }

    return QOpenGLWidget::event(event);
}

void VoxelOpenGLWidget::initializeGL()
{
    m_functions = context()->versionFunctions<QOpenGLFunctions_3_3_Core>();

    if (!m_functions)
    {
        m_backgroundRenderError = QStringLiteral("OpenGL 3.3 Core function table is unavailable in the GUI context.");
        return;
    }

    m_functions->initializeOpenGLFunctions();
    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

    if (!createPresentResources() || !startBackgroundRenderer())
    {
        return;
    }

    m_glInitialized = true;
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, [this]() { cleanupOpenGL(); }, Qt::DirectConnection);
    submitCameraState();
    m_renderThread->enqueueWireframe(m_wireframe);
    m_renderThread->requestFrame();
}

void VoxelOpenGLWidget::resizeGL(int width, int height)
{
    if (m_functions)
    {
        m_functions->glViewport(0, 0, width, height);
    }

    m_viewportSize = QSize((std::max)(width, 1), (std::max)(height, 1));
    submitCameraState();
}

void VoxelOpenGLWidget::paintGL()
{
    if (!m_functions)
    {
        return;
    }

    m_functions->glClear(GL_COLOR_BUFFER_BIT);

    if (!m_glInitialized || m_presentTextureId == 0)
    {
        return;
    }

    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    m_presentProgram.bind();
    m_presentProgram.setUniformValue("u_texture", 0);
    m_presentVao.bind();
    m_functions->glActiveTexture(GL_TEXTURE0);
    m_functions->glBindTexture(GL_TEXTURE_2D, m_presentTextureId);
    m_functions->glDrawArrays(GL_TRIANGLES, 0, 3);
    m_functions->glBindTexture(GL_TEXTURE_2D, 0);
    m_presentVao.release();
    m_presentProgram.release();
}

void VoxelOpenGLWidget::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
    case Qt::Key_F:
        fitAll();
        event->accept();
        return;

    case Qt::Key_1:
        setIsometricView();
        event->accept();
        return;

    case Qt::Key_2:
        setFrontView();
        event->accept();
        return;

    case Qt::Key_3:
        setTopView();
        event->accept();
        return;

    case Qt::Key_4:
        setRightView();
        event->accept();
        return;

    case Qt::Key_W:
        setWireframe(!m_wireframe);
        event->accept();
        return;

    default:
        break;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void VoxelOpenGLWidget::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePosition = event->pos();
    setFocus();
    event->accept();
}

void VoxelOpenGLWidget::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();

    if (event->buttons() & Qt::LeftButton)
    {
        m_yaw -= static_cast<float>(delta.x()) * 0.35f;
        m_pitch += static_cast<float>(delta.y()) * 0.35f;
        m_pitch = clampFloat(m_pitch, -89.0f, 89.0f);
        submitCameraState();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
        const float moveScale = cameraDistance * 0.0015f;
        m_viewOffset -= cameraRight() * static_cast<float>(delta.x()) * moveScale;
        m_viewOffset += cameraUp() * static_cast<float>(delta.y()) * moveScale;
        submitCameraState();
    }

    event->accept();
}

void VoxelOpenGLWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        fitAll();
        event->accept();
        return;
    }

    QOpenGLWidget::mouseDoubleClickEvent(event);
}

void VoxelOpenGLWidget::wheelEvent(QWheelEvent* event)
{
    if (event->angleDelta().y() > 0)
    {
        m_cameraScale *= 0.90f;
    }
    else if (event->angleDelta().y() < 0)
    {
        m_cameraScale *= 1.10f;
    }

    m_cameraScale = clampFloat(m_cameraScale, MinimumCameraScale, MaximumCameraScale);
    submitCameraState();
    event->accept();
}

/// CPU场景和快照

MeshObjectId VoxelOpenGLWidget::createSceneObjectLocked(MeshObjectKind kind, const QMatrix4x4& modelMatrix)
{
    assert(m_nextMeshObjectId != invalidMeshObjectId());
    const MeshObjectId objectId = m_nextMeshObjectId;

    if (m_nextMeshObjectId == (std::numeric_limits<MeshObjectId>::max)())
    {
        m_nextMeshObjectId = invalidMeshObjectId();
    }
    else
    {
        ++m_nextMeshObjectId;
    }

    SceneObjectInfo* object = new SceneObjectInfo(kind, modelMatrix);
    const std::pair<SceneObjectMap::iterator, bool> inserted = m_sceneObjects.insert(std::make_pair(objectId, object));

    if (!inserted.second)
    {
        delete object;
        return invalidMeshObjectId();
    }

    return objectId;
}

VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(MeshObjectId objectId)
{
    SceneObjectMap::iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

const VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(MeshObjectId objectId) const
{
    SceneObjectMap::const_iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

MeshObjectSnapshot VoxelOpenGLWidget::buildMeshSnapshotLocked(MeshObjectId objectId, SceneObjectInfo& object, const MyVoxel::Geometry::Mesh& mesh)
{
    assert(mesh.isValid());
    object.partVersions.clear();
    object.partTriangleCounts.clear();
    object.localBounds = mesh.localBounds();

    MeshObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.kind = MeshObjectKind::Mesh;
    snapshot.modelMatrix = object.modelMatrix;
    snapshot.visible = object.visible;

    if (!mesh.isEmpty())
    {
        const MeshPartIndex partIndex = MeshPartIndex::single();
        const std::uint64_t version = object.nextMeshVersion++;
        object.partVersions[partIndex] = version;
        object.partTriangleCounts[partIndex] = mesh.triangleCount();
        snapshot.parts.push_back(MeshPartSnapshot(partIndex, version, mesh));
    }

    return snapshot;
}

MeshObjectSnapshot VoxelOpenGLWidget::buildMeshCacheSnapshotLocked(MeshObjectId objectId, SceneObjectInfo& object, const MyVoxel::VoxelSurfaceCache& cache)
{
    object.partVersions.clear();
    object.partTriangleCounts.clear();
    object.localBounds = cache.localBounds();

    MeshObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.kind = MeshObjectKind::MeshCache;
    snapshot.modelMatrix = object.modelMatrix;
    snapshot.visible = object.visible;
    snapshot.parts.reserve(cache.rootCount() * MyVoxel::VoxelFaceDirectionCount);

    for (MyVoxel::VoxelSurfaceCache::RootEntryMap::const_iterator rootIterator = cache.rootEntries().begin(); rootIterator != cache.rootEntries().end(); ++rootIterator)
    {
        for (unsigned int directionValue = 0; directionValue < MyVoxel::VoxelFaceDirectionCount; ++directionValue)
        {
            const MyVoxel::VoxelFaceDirection direction = static_cast<MyVoxel::VoxelFaceDirection>(directionValue);
            const MeshPartIndex partIndex = MeshPartIndex::rootDirection(rootIterator->first, direction);
            const std::uint64_t version = rootIterator->second.directionMeshVersions[directionValue];
            const MyVoxel::Geometry::Mesh& directionMesh = rootIterator->second.directionMeshes[directionValue];
            object.partVersions[partIndex] = version;

            if (directionMesh.isEmpty())
            {
                continue;
            }

            object.partTriangleCounts[partIndex] = directionMesh.triangleCount();
            snapshot.parts.push_back(MeshPartSnapshot(partIndex, version, directionMesh));
        }
    }

    return snapshot;
}

/// 后台OpenGL生命周期

bool VoxelOpenGLWidget::createPresentResources()
{
    const char* vertexShader =
        "#version 330 core\n"
        "out vec2 v_uv;\n"
        "void main()\n"
        "{\n"
        "    vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));\n"
        "    vec2 position = positions[gl_VertexID];\n"
        "    gl_Position = vec4(position, 0.0, 1.0);\n"
        "    v_uv = position * 0.5 + 0.5;\n"
        "}\n";

    const char* fragmentShader =
        "#version 330 core\n"
        "in vec2 v_uv;\n"
        "uniform sampler2D u_texture;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    fragColor = texture(u_texture, clamp(v_uv, vec2(0.0), vec2(1.0)));\n"
        "}\n";

    if (!m_presentProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader) || !m_presentProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader) || !m_presentProgram.link())
    {
        m_backgroundRenderError = QStringLiteral("GUI presentation shader initialization failed: ") + m_presentProgram.log();
        return false;
    }

    if (!m_presentVao.create())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the GUI presentation vertex array object.");
        return false;
    }

    return true;
}

bool VoxelOpenGLWidget::startBackgroundRenderer()
{
    if (!QOpenGLContext::supportsThreadedOpenGL())
    {
        m_backgroundRenderError = QStringLiteral("The current Qt platform or OpenGL driver does not support threaded OpenGL.");
        return false;
    }

    m_offscreenSurface = new QOffscreenSurface();
    m_offscreenSurface->setFormat(context()->format());
    m_offscreenSurface->create();

    if (!m_offscreenSurface->isValid())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the background OpenGL offscreen surface.");
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    m_renderContext = new QOpenGLContext();
    m_renderContext->setFormat(context()->format());
    m_renderContext->setShareContext(context());

    if (!m_renderContext->create() || !m_renderContext->isValid())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the background shared OpenGL context.");
        delete m_renderContext;
        m_renderContext = nullptr;
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    if (!m_renderThread->startRendering(m_renderContext, m_offscreenSurface, this, thread()))
    {
        m_backgroundRenderError = QStringLiteral("Failed to start the background OpenGL render thread.");
        delete m_renderContext;
        m_renderContext = nullptr;
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    return true;
}

void VoxelOpenGLWidget::stopBackgroundRenderer()
{
    if (m_renderThread)
    {
        m_renderThread->stopRendering();
    }

    delete m_renderContext;
    m_renderContext = nullptr;
    delete m_offscreenSurface;
    m_offscreenSurface = nullptr;
    m_presentTextureId = 0;
    m_presentFrameId = 0;
    m_presentFrameVersion = 0;
    m_presentTextureSize = QSize();
}

void VoxelOpenGLWidget::cleanupOpenGL()
{
    if (!m_glInitialized && !m_renderContext && !m_offscreenSurface && !m_presentVao.isCreated())
    {
        return;
    }

    stopBackgroundRenderer();
    QCoreApplication::removePostedEvents(this, RenderFrameReadyEvent::eventType());
    QCoreApplication::removePostedEvents(this, RenderFailureEvent::eventType());

    if (context())
    {
        makeCurrent();

        if (m_presentVao.isCreated())
        {
            m_presentVao.destroy();
        }

        m_presentProgram.removeAllShaders();
        doneCurrent();
    }

    m_glInitialized = false;
    m_functions = nullptr;
}

/// 相机和场景范围

void VoxelOpenGLWidget::updateBounds()
{
    bool hasPoint = false;
    QVector3D minimum;
    QVector3D maximum;

    {
        QMutexLocker locker(&m_sceneMutex);

        for (SceneObjectMap::const_iterator iterator = m_sceneObjects.begin(); iterator != m_sceneObjects.end(); ++iterator)
        {
            const SceneObjectInfo& object = *iterator->second;

            if (!object.visible)
            {
                continue;
            }

            expandWorldBounds(object.localBounds, object.modelMatrix, hasPoint, minimum, maximum);
        }
    }

    if (!hasPoint)
    {
        m_center = QVector3D(0.0f, 0.0f, 0.0f);
        m_radius = 1.0f;
        return;
    }

    m_center = (minimum + maximum) * 0.5f;
    m_radius = (std::max)((maximum - minimum).length() * 0.5f, MinimumRadius);
}

void VoxelOpenGLWidget::submitCameraState()
{
    RenderCameraState state;
    state.center = m_center;
    state.viewOffset = m_viewOffset;
    state.radius = m_radius;
    state.cameraScale = m_cameraScale;
    state.yaw = m_yaw;
    state.pitch = m_pitch;
    state.viewportSize = m_viewportSize;
    m_renderThread->enqueueCameraState(state);
}

void VoxelOpenGLWidget::requestBoundsUpdate()
{
    if (QThread::currentThread() == thread())
    {
        updateBounds();
        submitCameraState();
        return;
    }

    if (m_sceneBoundsEventPending.testAndSetOrdered(0, 1))
    {
        QCoreApplication::postEvent(this, new QEvent(sceneBoundsChangedEventType()));
    }
}

QVector3D VoxelOpenGLWidget::cameraDirection() const
{
    const float yaw = degreeToRadian(m_yaw);
    const float pitch = degreeToRadian(m_pitch);
    return QVector3D(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraRight() const
{
    const QVector3D forward = -cameraDirection();
    return QVector3D::crossProduct(forward, QVector3D(0.0f, 0.0f, 1.0f)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraUp() const
{
    const QVector3D forward = -cameraDirection();
    return QVector3D::crossProduct(cameraRight(), forward).normalized();
}

}