#include "VoxelOpenGLWidget.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QSurfaceFormat>
#include <QWheelEvent>

namespace
{

const float Pi = 3.14159265358979323846f; // 角度与弧度转换使用的圆周率。
const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const float MinimumCameraScale = 0.35f; // 滚轮缩放允许的最小相机距离比例。
const float MaximumCameraScale = 50.0f; // 滚轮缩放允许的最大相机距离比例。

typedef std::chrono::steady_clock UpdateClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const UpdateClock::time_point& begin,
                           const UpdateClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

const std::size_t OpenGLVertexFloatCount = 10; // 每个展开顶点包含位置3、法线3和颜色4个float。
const int OpenGLPositionFloatOffset = 0; // 展开顶点中位置属性的float偏移。
const int OpenGLNormalFloatOffset = 3; // 展开顶点中法线属性的float偏移。
const int OpenGLColorFloatOffset = 6; // 展开顶点中颜色属性的float偏移。

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

// 将字节数量转换为QOpenGLBuffer使用的有符号整数。
int checkedBufferByteSize(std::size_t byteSize)
{
    assert(byteSize <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
    return static_cast<int>(byteSize);
}
// 将局部轴对齐包围盒的八个角点变换到世界空间并扩展场景范围。
void expandWorldBounds(const MyVoxel::Bounds3& localBounds,
                       const QMatrix4x4& modelMatrix,
                       bool& hasPoint,
                       QVector3D& minimum,
                       QVector3D& maximum)
{
    if (!localBounds.isValid())
    {
        return;
    }

    const MyMath::Vector3& localMinimum = localBounds.minimum();
    const MyMath::Vector3& localMaximum = localBounds.maximum();

    const float x[2] =
    {
        static_cast<float>(localMinimum.x()),
        static_cast<float>(localMaximum.x())
    };

    const float y[2] =
    {
        static_cast<float>(localMinimum.y()),
        static_cast<float>(localMaximum.y())
    };

    const float z[2] =
    {
        static_cast<float>(localMinimum.z()),
        static_cast<float>(localMaximum.z())
    };

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
// 将三角形颜色网格展开到可复用的OpenGL逐顶点float数组。
void buildOpenGLVertexData(
    const MyVoxel::Geometry::Mesh& mesh,
    std::vector<float>& vertexData)
{
    assert(mesh.isValid());

    vertexData.resize(
        mesh.indexCount() *
        OpenGLVertexFloatCount);

    std::size_t outputOffset = 0;

    for (std::size_t triangleIndex = 0;
         triangleIndex < mesh.triangleCount();
         ++triangleIndex)
    {
        const MyVoxel::Geometry::MeshColor& color =
            mesh.triangleColor(triangleIndex);
        const std::size_t firstIndexPosition =
            triangleIndex * 3;

        for (unsigned int corner = 0;
             corner < 3;
             ++corner)
        {
            const std::uint32_t sourceIndex =
                mesh.indices()[
                    firstIndexPosition +
                    corner];
            const MyVoxel::Geometry::MeshVertex& sourceVertex =
                mesh.vertices()[sourceIndex];

            vertexData[outputOffset++] = sourceVertex.x;
            vertexData[outputOffset++] = sourceVertex.y;
            vertexData[outputOffset++] = sourceVertex.z;
            vertexData[outputOffset++] = sourceVertex.normalX;
            vertexData[outputOffset++] = sourceVertex.normalY;
            vertexData[outputOffset++] = sourceVertex.normalZ;
            vertexData[outputOffset++] = color.red;
            vertexData[outputOffset++] = color.green;
            vertexData[outputOffset++] = color.blue;
            vertexData[outputOffset++] = color.alpha;
        }
    }

    assert(outputOffset == vertexData.size());
}

}

namespace MyVoxelViewer
{

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

    uploadedPartCount = 0;
    removedPartCount = 0;
    createdGpuPartCount = 0;
    reusedGpuPartCount = 0;
    stagedCpuPartCount = 0;
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

    uploadedPartCount += other.uploadedPartCount;
    removedPartCount += other.removedPartCount;
    createdGpuPartCount += other.createdGpuPartCount;
    reusedGpuPartCount += other.reusedGpuPartCount;
    stagedCpuPartCount += other.stagedCpuPartCount;
}

struct VoxelOpenGLWidget::MeshPartIndex
{
    enum Kind
    {
        Single,
        RootDirection
    };

    MeshPartIndex()
        : kind(Single)
        , direction(MyVoxel::VoxelFaceDirection::NegativeX)
    {
    }

    MeshPartIndex(
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        MyVoxel::VoxelFaceDirection directionValue)
        : kind(RootDirection)
        , rootIndex(rootIndexValue)
        , direction(directionValue)
    {
        assert(
            MyVoxel::isValidVoxelFaceDirection(
                directionValue));
    }

    static MeshPartIndex single()
    {
        return MeshPartIndex();
    }

    static MeshPartIndex rootDirection(
        const MyVoxel::VoxelCellIndex& rootIndex,
        MyVoxel::VoxelFaceDirection direction)
    {
        return MeshPartIndex(
            rootIndex,
            direction);
    }

    bool operator<(const MeshPartIndex& other) const
    {
        if (kind != other.kind)
        {
            return kind < other.kind;
        }

        if (kind == Single)
        {
            return false;
        }

        if (rootIndex != other.rootIndex)
        {
            return rootIndex < other.rootIndex;
        }

        return static_cast<unsigned int>(direction) <
            static_cast<unsigned int>(other.direction);
    }

    Kind kind; // 当前分片是普通对象唯一分片还是Root方向分片。
    MyVoxel::VoxelCellIndex rootIndex; // Root方向分片对应的第0层根索引。
    MyVoxel::VoxelFaceDirection direction; // Root方向分片对应的单位面方向。
};

struct VoxelOpenGLWidget::RenderMesh
{
    RenderMesh()
        : vertexBuffer(QOpenGLBuffer::VertexBuffer)
    {
    }

    QOpenGLVertexArrayObject vertexArray; // 当前分片顶点属性状态。
    QOpenGLBuffer vertexBuffer; // 当前分片展开后的三角形顶点数据。
    int vertexCount = 0; // 当前分片需要绘制的OpenGL顶点数量。
};

struct VoxelOpenGLWidget::MeshObject
{
    typedef std::map<MeshPartIndex, MyVoxel::Geometry::Mesh> CpuPartMap;
    typedef std::map<MeshPartIndex, std::uint64_t> PartVersionMap;
    typedef std::map<MeshPartIndex, std::size_t> PartTriangleCountMap;
    typedef std::map<MeshPartIndex, std::unique_ptr<RenderMesh>> GpuPartMap;

    MeshObject(MeshObjectKind kindValue, const QMatrix4x4& modelMatrixValue)
        : kind(kindValue)
        , modelMatrix(modelMatrixValue)
    {
    }

    MeshObjectKind kind; // 普通网格对象或体素根缓存对象。
    QMatrix4x4 modelMatrix; // 当前对象局部空间到显示世界空间的模型矩阵。
    bool visible = true; // 当前对象是否参与绘制和包围范围计算。
    MyVoxel::Bounds3 localBounds; // 当前对象全部CPU分片形成的局部轴对齐包围盒。
    CpuPartMap cpuParts; // 普通对象持久保存CPU Mesh；体素对象仅在OpenGL初始化前暂存待上传分片。
    PartVersionMap partVersions; // Root方向分片最后同步的缓存版本。
    PartTriangleCountMap partTriangleCounts; // 当前非空分片的三角形数量，避免依赖CPU Mesh副本统计。
    GpuPartMap gpuParts; // 当前对象的GPU网格分片。
};

VoxelOpenGLWidget::VoxelOpenGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    // OpenGL 3.3 Core提供VAO、现代Shader和稳定的跨平台顶点接口。
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    setFormat(format);

    m_defaultModelMatrix.setToIdentity();

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(false);
    setMinimumSize(480, 320);
}

VoxelOpenGLWidget::~VoxelOpenGLWidget()
{
    if (context())
    {
        makeCurrent();
        clearAllRenderMeshes();

        if (m_backgroundVao.isCreated())
        {
            m_backgroundVao.destroy();
        }

        doneCurrent();
    }
}

/// 通用网格对象

MeshObjectId VoxelOpenGLWidget::invalidMeshObjectId()
{
    return static_cast<MeshObjectId>(0);
}

MeshObjectId VoxelOpenGLWidget::addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix)
{
    assert(mesh.isValid());

    const MeshObjectId objectId = createMeshObject(MeshObjectKind::Mesh, modelMatrix);
    MeshObject* object = findMeshObject(objectId);

    assert(object);
    replaceObjectMesh(*object, mesh);

    updateBounds();
    update();
    return objectId;
}

bool VoxelOpenGLWidget::setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh)
{
    assert(mesh.isValid());

    MeshObject* object = findMeshObject(objectId);

    if (!object || object->kind != MeshObjectKind::Mesh)
    {
        return false;
    }

    replaceObjectMesh(*object, mesh);
    updateBounds();
    update();
    return true;
}

MeshObjectId VoxelOpenGLWidget::addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix)
{
    const MeshObjectId objectId = createMeshObject(MeshObjectKind::MeshCache, modelMatrix);
    MeshObject* object = findMeshObject(objectId);

    assert(object);
    replaceObjectMeshCache(*object, cache);

    updateBounds();
    update();
    return objectId;
}

bool VoxelOpenGLWidget::setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache)
{
    MeshObject* object = findMeshObject(objectId);

    if (!object || object->kind != MeshObjectKind::MeshCache)
    {
        return false;
    }

    replaceObjectMeshCache(*object, cache);
    updateBounds();
    update();
    return true;
}

bool VoxelOpenGLWidget::updateRootMeshes(MeshObjectId objectId,
                                         const MyVoxel::VoxelSurfaceCache& cache,
                                         const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    m_lastUpdatedMeshPartCount = 0;
    m_lastMeshUpdateStatistics.clear();

    const UpdateClock::time_point updateStart = UpdateClock::now();
    MeshObject* object = findMeshObject(objectId);

    if (!object || object->kind != MeshObjectKind::MeshCache)
    {
        m_lastMeshUpdateStatistics.totalMilliseconds =
            elapsedMilliseconds(updateStart, UpdateClock::now());
        m_lastMeshUpdateStatistics.cacheAndVersionMilliseconds =
            m_lastMeshUpdateStatistics.totalMilliseconds;
        return false;
    }

    m_lastUpdatedMeshPartCount =
        updateObjectRootMeshes(
            *object,
            cache,
            changedRootIndices);

    update();

    m_lastMeshUpdateStatistics.totalMilliseconds =
        elapsedMilliseconds(updateStart, UpdateClock::now());

    const double measuredStageMilliseconds =
        m_lastMeshUpdateStatistics.cpuStagingCopyMilliseconds +
        m_lastMeshUpdateStatistics.contextAcquireMilliseconds +
        m_lastMeshUpdateStatistics.vertexExpansionMilliseconds +
        m_lastMeshUpdateStatistics.gpuUploadMilliseconds +
        m_lastMeshUpdateStatistics.gpuRemovalMilliseconds +
        m_lastMeshUpdateStatistics.contextReleaseMilliseconds;

    m_lastMeshUpdateStatistics.cacheAndVersionMilliseconds =
        (std::max)(
            0.0,
            m_lastMeshUpdateStatistics.totalMilliseconds -
                measuredStageMilliseconds);

    return true;
}

bool VoxelOpenGLWidget::setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix)
{
    MeshObject* object = findMeshObject(objectId);

    if (!object)
    {
        return false;
    }

    object->modelMatrix = matrix;
    update();
    return true;
}

const QMatrix4x4* VoxelOpenGLWidget::meshObjectMatrix(MeshObjectId objectId) const
{
    const MeshObject* object = findMeshObject(objectId);
    return object ? &object->modelMatrix : nullptr;
}

bool VoxelOpenGLWidget::setMeshObjectVisible(MeshObjectId objectId, bool visible)
{
    MeshObject* object = findMeshObject(objectId);

    if (!object)
    {
        return false;
    }

    if (object->visible == visible)
    {
        return true;
    }

    object->visible = visible;
    updateBounds();
    update();
    return true;
}

bool VoxelOpenGLWidget::isMeshObjectVisible(MeshObjectId objectId) const
{
    const MeshObject* object = findMeshObject(objectId);
    return object && object->visible;
}

bool VoxelOpenGLWidget::removeMeshObject(MeshObjectId objectId)
{
    MeshObjectMap::iterator iterator = m_meshObjects.find(objectId);

    if (iterator == m_meshObjects.end())
    {
        return false;
    }

    if (m_initialized)
    {
        makeCurrent();
        clearObjectRenderMeshes(*iterator->second);
        doneCurrent();
    }

    m_meshObjects.erase(iterator);

    if (m_defaultVoxelObjectId == objectId)
    {
        m_defaultVoxelObjectId = invalidMeshObjectId();
    }

    updateBounds();
    update();
    return true;
}

void VoxelOpenGLWidget::clearMeshes()
{
    if (m_initialized)
    {
        makeCurrent();
        clearAllRenderMeshes();
        doneCurrent();
    }

    m_meshObjects.clear();
    m_defaultVoxelObjectId = invalidMeshObjectId();

    updateBounds();
    fitAll();
}

bool VoxelOpenGLWidget::containsMeshObject(MeshObjectId objectId) const
{
    return findMeshObject(objectId) != nullptr;
}

std::size_t VoxelOpenGLWidget::meshObjectCount() const
{
    return m_meshObjects.size();
}

std::size_t VoxelOpenGLWidget::meshObjectPartCount(MeshObjectId objectId) const
{
    const MeshObject* object = findMeshObject(objectId);
    return object ? object->partTriangleCounts.size() : 0;
}

std::size_t VoxelOpenGLWidget::meshObjectTriangleCount(MeshObjectId objectId) const
{
    const MeshObject* object = findMeshObject(objectId);

    if (!object)
    {
        return 0;
    }

    std::size_t triangleCount = 0;

    for (MeshObject::PartTriangleCountMap::const_iterator iterator =
             object->partTriangleCounts.begin();
         iterator != object->partTriangleCounts.end();
         ++iterator)
    {
        triangleCount += iterator->second;
    }

    return triangleCount;
}

std::size_t VoxelOpenGLWidget::lastUpdatedMeshPartCount() const
{
    return m_lastUpdatedMeshPartCount;
}

const MeshUpdateStatistics& VoxelOpenGLWidget::lastMeshUpdateStatistics() const
{
    return m_lastMeshUpdateStatistics;
}

/// 单体素对象兼容入口

void VoxelOpenGLWidget::setMeshCache(const MyVoxel::VoxelSurfaceCache& cache)
{
    if (m_defaultVoxelObjectId == invalidMeshObjectId())
    {
        m_defaultVoxelObjectId = addMeshCache(cache, m_defaultModelMatrix);
    }
    else
    {
        const bool replaced = setMeshCache(m_defaultVoxelObjectId, cache);
        assert(replaced);
    }

    fitAll();
}

void VoxelOpenGLWidget::updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache,
                                         const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    if (m_defaultVoxelObjectId == invalidMeshObjectId())
    {
        setMeshCache(cache);
        return;
    }

    const bool updated = updateRootMeshes(m_defaultVoxelObjectId, cache, changedRootIndices);
    assert(updated);
}

std::size_t VoxelOpenGLWidget::rootMeshCount() const
{
    const MeshObject* object =
        findMeshObject(m_defaultVoxelObjectId);

    if (!object ||
        object->kind != MeshObjectKind::MeshCache)
    {
        return 0;
    }

    std::size_t rootCount = 0;
    bool hasPreviousRoot = false;
    MyVoxel::VoxelCellIndex previousRoot;

    for (MeshObject::PartVersionMap::const_iterator iterator =
             object->partVersions.begin();
         iterator != object->partVersions.end();
         ++iterator)
    {
        if (iterator->first.kind !=
            MeshPartIndex::RootDirection)
        {
            continue;
        }

        if (!hasPreviousRoot ||
            iterator->first.rootIndex != previousRoot)
        {
            previousRoot =
                iterator->first.rootIndex;
            hasPreviousRoot = true;
            ++rootCount;
        }
    }

    return rootCount;
}

void VoxelOpenGLWidget::setModelMatrix(const QMatrix4x4& matrix)
{
    m_defaultModelMatrix = matrix;

    if (m_defaultVoxelObjectId != invalidMeshObjectId())
    {
        const bool updated = setMeshObjectMatrix(m_defaultVoxelObjectId, matrix);
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
    update();
}

bool VoxelOpenGLWidget::isWireframe() const
{
    return m_wireframe;
}

/// 相机控制

void VoxelOpenGLWidget::fitAll()
{
    updateBounds();
    m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f);
    m_cameraScale = 2.8f;
    update();
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

/// 对象管理

MeshObjectId VoxelOpenGLWidget::createMeshObject(MeshObjectKind kind, const QMatrix4x4& modelMatrix)
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

    std::unique_ptr<MeshObject> object(new MeshObject(kind, modelMatrix));
    const std::pair<MeshObjectMap::iterator, bool> inserted = m_meshObjects.insert(std::make_pair(objectId, std::move(object)));

    assert(inserted.second);
    return objectId;
}

VoxelOpenGLWidget::MeshObject* VoxelOpenGLWidget::findMeshObject(MeshObjectId objectId)
{
    MeshObjectMap::iterator iterator = m_meshObjects.find(objectId);
    return iterator == m_meshObjects.end() ? nullptr : iterator->second.get();
}

const VoxelOpenGLWidget::MeshObject* VoxelOpenGLWidget::findMeshObject(MeshObjectId objectId) const
{
    MeshObjectMap::const_iterator iterator = m_meshObjects.find(objectId);
    return iterator == m_meshObjects.end() ? nullptr : iterator->second.get();
}

void VoxelOpenGLWidget::replaceObjectMesh(MeshObject& object, const MyVoxel::Geometry::Mesh& mesh)
{
    assert(mesh.isValid());

    if (m_initialized)
    {
        makeCurrent();
        clearObjectRenderMeshes(object);
    }

    object.cpuParts.clear();
    object.partVersions.clear();
    object.partTriangleCounts.clear();
    object.localBounds = mesh.localBounds();

    if (!mesh.isEmpty())
    {
        const MeshPartIndex partIndex = MeshPartIndex::single();

        object.cpuParts.insert(std::make_pair(partIndex, mesh));
        object.partTriangleCounts[partIndex] = mesh.triangleCount();

        if (m_initialized)
        {
            uploadMeshPart(object, partIndex, mesh);
        }
    }

    if (m_initialized)
    {
        doneCurrent();
    }
}

void VoxelOpenGLWidget::replaceObjectMeshCache(
    MeshObject& object,
    const MyVoxel::VoxelSurfaceCache& cache)
{
    if (m_initialized)
    {
        makeCurrent();
        clearObjectRenderMeshes(object);
    }

    object.cpuParts.clear();
    object.partVersions.clear();
    object.partTriangleCounts.clear();
    object.localBounds = cache.localBounds();

    for (MyVoxel::VoxelSurfaceCache::RootEntryMap::const_iterator
             rootIterator = cache.rootEntries().begin();
         rootIterator != cache.rootEntries().end();
         ++rootIterator)
    {
        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            const MyVoxel::VoxelFaceDirection direction =
                static_cast<MyVoxel::VoxelFaceDirection>(
                    directionValue);
            const MeshPartIndex partIndex =
                MeshPartIndex::rootDirection(
                    rootIterator->first,
                    direction);
            const MyVoxel::Geometry::Mesh& directionMesh =
                rootIterator->second.directionMeshes[
                    directionValue];

            object.partVersions[partIndex] =
                rootIterator->second.directionMeshVersions[
                    directionValue];

            if (directionMesh.isEmpty())
            {
                continue;
            }

            object.partTriangleCounts[partIndex] =
                directionMesh.triangleCount();

            if (m_initialized)
            {
                uploadMeshPart(
                    object,
                    partIndex,
                    directionMesh);
            }
            else
            {
                object.cpuParts.insert(
                    std::make_pair(
                        partIndex,
                        directionMesh));
            }
        }
    }

    if (m_initialized)
    {
        doneCurrent();
    }
}

std::size_t VoxelOpenGLWidget::updateObjectRootMeshes(
    MeshObject& object,
    const MyVoxel::VoxelSurfaceCache& cache,
    const MyVoxel::VoxelSurfaceCache::RootIndexSet&
        changedRootIndices)
{
    if (m_initialized)
    {
        const UpdateClock::time_point acquireStart =
            UpdateClock::now();

        makeCurrent();

        m_lastMeshUpdateStatistics.contextAcquireMilliseconds +=
            elapsedMilliseconds(
                acquireStart,
                UpdateClock::now());
    }

    std::size_t updatedPartCount = 0;

    for (MyVoxel::VoxelSurfaceCache::RootIndexSet::const_iterator
             rootIterator = changedRootIndices.begin();
         rootIterator != changedRootIndices.end();
         ++rootIterator)
    {
        const MyVoxel::VoxelSurfaceCache::RootEntry* rootEntry =
            cache.rootEntry(*rootIterator);

        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            const MyVoxel::VoxelFaceDirection direction =
                static_cast<MyVoxel::VoxelFaceDirection>(
                    directionValue);
            const MeshPartIndex partIndex =
                MeshPartIndex::rootDirection(
                    *rootIterator,
                    direction);

            if (!rootEntry)
            {
                const bool hadPart =
                    object.partTriangleCounts.erase(
                        partIndex) > 0;

                object.cpuParts.erase(partIndex);
                object.partVersions.erase(partIndex);

                if (m_initialized)
                {
                    removeRenderMesh(
                        object,
                        partIndex,
                        &m_lastMeshUpdateStatistics);
                }

                if (hadPart)
                {
                    ++updatedPartCount;
                    ++m_lastMeshUpdateStatistics.removedPartCount;
                }

                continue;
            }

            const std::uint64_t currentVersion =
                rootEntry->directionMeshVersions[
                    directionValue];
            const MeshObject::PartVersionMap::const_iterator
                versionIterator =
                    object.partVersions.find(partIndex);

            if (versionIterator !=
                    object.partVersions.end() &&
                versionIterator->second == currentVersion)
            {
                continue;
            }

            const MyVoxel::Geometry::Mesh& directionMesh =
                rootEntry->directionMeshes[directionValue];

            object.partVersions[partIndex] =
                currentVersion;

            if (!directionMesh.isEmpty())
            {
                object.partTriangleCounts[partIndex] =
                    directionMesh.triangleCount();

                if (m_initialized)
                {
                    uploadMeshPart(
                        object,
                        partIndex,
                        directionMesh,
                        &m_lastMeshUpdateStatistics);
                }
                else
                {
                    const UpdateClock::time_point copyStart =
                        UpdateClock::now();

                    object.cpuParts[partIndex] =
                        directionMesh;

                    m_lastMeshUpdateStatistics.cpuStagingCopyMilliseconds +=
                        elapsedMilliseconds(
                            copyStart,
                            UpdateClock::now());

                    ++m_lastMeshUpdateStatistics.stagedCpuPartCount;
                }

                ++updatedPartCount;

                if (m_initialized)
                {
                    ++m_lastMeshUpdateStatistics.uploadedPartCount;
                }
            }
            else
            {
                const bool hadPart =
                    object.partTriangleCounts.erase(
                        partIndex) > 0;

                object.cpuParts.erase(partIndex);

                if (m_initialized)
                {
                    removeRenderMesh(
                        object,
                        partIndex,
                        &m_lastMeshUpdateStatistics);
                }

                if (hadPart)
                {
                    ++updatedPartCount;
                    ++m_lastMeshUpdateStatistics.removedPartCount;
                }
            }
        }
    }

    object.localBounds = cache.localBounds();

    if (m_initialized)
    {
        const UpdateClock::time_point releaseStart =
            UpdateClock::now();

        doneCurrent();

        m_lastMeshUpdateStatistics.contextReleaseMilliseconds +=
            elapsedMilliseconds(
                releaseStart,
                UpdateClock::now());
    }

    return updatedPartCount;
}

/// OpenGL事件

void VoxelOpenGLWidget::initializeGL()
{
    m_functions = context()->versionFunctions<QOpenGLFunctions_3_3_Core>();

    assert(m_functions);
    m_functions->initializeOpenGLFunctions();

    m_functions->glEnable(GL_DEPTH_TEST);
    m_functions->glDepthFunc(GL_LEQUAL);
    m_functions->glEnable(GL_MULTISAMPLE);
    m_functions->glEnable(GL_BLEND);
    m_functions->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    m_functions->glDisable(GL_CULL_FACE);
    m_functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

    createShaderPrograms();
    createBackgroundResources();

    for (MeshObjectMap::iterator objectIterator =
             m_meshObjects.begin();
         objectIterator != m_meshObjects.end();
         ++objectIterator)
    {
        MeshObject& object = *objectIterator->second;

        for (MeshObject::CpuPartMap::const_iterator partIterator =
                 object.cpuParts.begin();
             partIterator != object.cpuParts.end();
             ++partIterator)
        {
            uploadMeshPart(
                object,
                partIterator->first,
                partIterator->second);
        }

        if (object.kind == MeshObjectKind::MeshCache)
        {
            object.cpuParts.clear();
        }
    }

    m_initialized = true;
}

void VoxelOpenGLWidget::resizeGL(int width, int height)
{
    m_functions->glViewport(0, 0, width, height);
}

void VoxelOpenGLWidget::paintGL()
{
    m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawBackground();

    if (!hasVisibleRenderMeshes())
    {
        return;
    }

    const QVector3D viewCenter = m_center + m_viewOffset;
    const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
    const QVector3D eye = viewCenter + cameraDirection() * cameraDistance;

    drawMeshObjects(eye, viewCenter);
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
        update();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
        const float moveScale = cameraDistance * 0.0015f;

        m_viewOffset -= cameraRight() * static_cast<float>(delta.x()) * moveScale;
        m_viewOffset += cameraUp() * static_cast<float>(delta.y()) * moveScale;
        update();
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
    update();
    event->accept();
}

/// OpenGL资源

void VoxelOpenGLWidget::createShaderPrograms()
{
    const char* backgroundVertexShader =
        "#version 330 core\n"
        "out vec2 v_uv;\n"
        "void main()\n"
        "{\n"
        "    vec2 positions[3] = vec2[3](\n"
        "        vec2(-1.0, -1.0),\n"
        "        vec2( 3.0, -1.0),\n"
        "        vec2(-1.0,  3.0));\n"
        "    vec2 position = positions[gl_VertexID];\n"
        "    gl_Position = vec4(position, 0.0, 1.0);\n"
        "    v_uv = position * 0.5 + 0.5;\n"
        "}\n";

    const char* backgroundFragmentShader =
        "#version 330 core\n"
        "in vec2 v_uv;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    vec3 bottomColor = vec3(0.040, 0.050, 0.070);\n"
        "    vec3 topColor = vec3(0.205, 0.235, 0.285);\n"
        "    float vertical = smoothstep(0.0, 1.0, clamp(v_uv.y, 0.0, 1.0));\n"
        "    vec3 color = mix(bottomColor, topColor, vertical);\n"
        "    vec2 centered = v_uv - vec2(0.50, 0.46);\n"
        "    float radial = length(centered * vec2(1.0, 0.82));\n"
        "    color += vec3(0.030, 0.034, 0.042) * (1.0 - smoothstep(0.05, 0.82, radial));\n"
        "    color *= 1.0 - smoothstep(0.35, 0.95, radial) * 0.24;\n"
        "    fragColor = vec4(color, 1.0);\n"
        "}\n";

    const char* meshVertexShader =
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_position;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec4 a_color;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_mvp;\n"
        "uniform mat3 u_normalMatrix;\n"
        "out vec3 v_worldPosition;\n"
        "out vec3 v_normal;\n"
        "out vec4 v_color;\n"
        "void main()\n"
        "{\n"
        "    vec4 worldPosition = u_model * vec4(a_position, 1.0);\n"
        "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
        "    v_worldPosition = worldPosition.xyz;\n"
        "    v_normal = normalize(u_normalMatrix * a_normal);\n"
        "    v_color = a_color;\n"
        "}\n";

    const char* meshFragmentShader =
        "#version 330 core\n"
        "in vec3 v_worldPosition;\n"
        "in vec3 v_normal;\n"
        "in vec4 v_color;\n"
        "uniform vec3 u_cameraPosition;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    vec3 normal = normalize(v_normal);\n"
        "    if (!gl_FrontFacing)\n"
        "    {\n"
        "        normal = -normal;\n"
        "    }\n"
        "    vec3 viewDirection = normalize(u_cameraPosition - v_worldPosition);\n"
        "    vec3 keyDirection = normalize(vec3(0.45, -0.35, 0.82));\n"
        "    vec3 fillDirection = normalize(vec3(-0.65, 0.55, 0.35));\n"
        "    float keyDiffuse = max(dot(normal, keyDirection), 0.0);\n"
        "    float fillDiffuse = max(dot(normal, fillDirection), 0.0);\n"
        "    vec3 halfDirection = normalize(keyDirection + viewDirection);\n"
        "    float specular = pow(max(dot(normal, halfDirection), 0.0), 48.0);\n"
        "    float rim = pow(1.0 - max(dot(normal, viewDirection), 0.0), 2.0);\n"
        "    vec3 baseColor = pow(max(v_color.rgb, vec3(0.0)), vec3(2.2));\n"
        "    vec3 linearColor = baseColor * 0.24;\n"
        "    linearColor += baseColor * keyDiffuse * 0.78;\n"
        "    linearColor += baseColor * fillDiffuse * 0.20;\n"
        "    linearColor += vec3(1.0, 0.97, 0.90) * specular * 0.28;\n"
        "    linearColor += mix(baseColor, vec3(0.70, 0.82, 1.0), 0.45) * rim * 0.12;\n"
        "    linearColor = linearColor / (linearColor + vec3(0.35));\n"
        "    vec3 displayColor = pow(max(linearColor, vec3(0.0)), vec3(1.0 / 2.2));\n"
        "    fragColor = vec4(displayColor, v_color.a);\n"
        "}\n";

    const bool backgroundVertexOk = m_backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, backgroundVertexShader);
    const bool backgroundFragmentOk = m_backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, backgroundFragmentShader);
    const bool backgroundLinkOk = m_backgroundProgram.link();
    const bool meshVertexOk = m_meshProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, meshVertexShader);
    const bool meshFragmentOk = m_meshProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, meshFragmentShader);
    const bool meshLinkOk = m_meshProgram.link();

    assert(backgroundVertexOk);
    assert(backgroundFragmentOk);
    assert(backgroundLinkOk);
    assert(meshVertexOk);
    assert(meshFragmentOk);
    assert(meshLinkOk);
}

void VoxelOpenGLWidget::createBackgroundResources()
{
    const bool created = m_backgroundVao.create();
    assert(created);
}

void VoxelOpenGLWidget::uploadMeshPart(
    MeshObject& object,
    const MeshPartIndex& partIndex,
    const MyVoxel::Geometry::Mesh& mesh,
    MeshUpdateStatistics* statistics)
{
    assert(mesh.isValid());
    assert(!mesh.isEmpty());

    const UpdateClock::time_point expansionStart =
        UpdateClock::now();
    buildOpenGLVertexData(
        mesh,
        m_uploadVertexData);

    if (statistics)
    {
        statistics->vertexExpansionMilliseconds +=
            elapsedMilliseconds(
                expansionStart,
                UpdateClock::now());
    }

    const UpdateClock::time_point uploadStart =
        UpdateClock::now();
    MeshObject::GpuPartMap::iterator iterator =
        object.gpuParts.find(partIndex);
    std::unique_ptr<RenderMesh> newRenderMesh;
    RenderMesh* renderMesh = nullptr;
    bool created = false;

    if (iterator == object.gpuParts.end())
    {
        newRenderMesh.reset(new RenderMesh());
        renderMesh = newRenderMesh.get();
        created = true;

        const bool vaoCreated =
            renderMesh->vertexArray.create();
        const bool bufferCreated =
            renderMesh->vertexBuffer.create();

        assert(vaoCreated);
        assert(bufferCreated);
    }
    else
    {
        renderMesh = iterator->second.get();
    }

    renderMesh->vertexArray.bind();
    renderMesh->vertexBuffer.bind();

    if (created)
    {
        renderMesh->vertexBuffer.setUsagePattern(
            object.kind == MeshObjectKind::MeshCache
                ? QOpenGLBuffer::DynamicDraw
                : QOpenGLBuffer::StaticDraw);
    }

    renderMesh->vertexBuffer.allocate(
        &m_uploadVertexData[0],
        checkedBufferByteSize(
            m_uploadVertexData.size() *
            sizeof(float)));

    if (created)
    {
        m_meshProgram.bind();

        m_meshProgram.enableAttributeArray(0);
        m_meshProgram.setAttributeBuffer(
            0,
            GL_FLOAT,
            OpenGLPositionFloatOffset *
                static_cast<int>(sizeof(float)),
            3,
            OpenGLVertexFloatCount *
                static_cast<int>(sizeof(float)));

        m_meshProgram.enableAttributeArray(1);
        m_meshProgram.setAttributeBuffer(
            1,
            GL_FLOAT,
            OpenGLNormalFloatOffset *
                static_cast<int>(sizeof(float)),
            3,
            OpenGLVertexFloatCount *
                static_cast<int>(sizeof(float)));

        m_meshProgram.enableAttributeArray(2);
        m_meshProgram.setAttributeBuffer(
            2,
            GL_FLOAT,
            OpenGLColorFloatOffset *
                static_cast<int>(sizeof(float)),
            4,
            OpenGLVertexFloatCount *
                static_cast<int>(sizeof(float)));

        m_meshProgram.release();
    }

    renderMesh->vertexBuffer.release();
    renderMesh->vertexArray.release();

    assert(
        mesh.indexCount() <=
        static_cast<std::size_t>(
            (std::numeric_limits<int>::max)()));

    renderMesh->vertexCount =
        static_cast<int>(mesh.indexCount());

    if (created)
    {
        object.gpuParts.insert(
            std::make_pair(
                partIndex,
                std::move(newRenderMesh)));
    }

    if (statistics)
    {
        statistics->gpuUploadMilliseconds +=
            elapsedMilliseconds(
                uploadStart,
                UpdateClock::now());

        if (created)
        {
            ++statistics->createdGpuPartCount;
        }
        else
        {
            ++statistics->reusedGpuPartCount;
        }
    }
}

void VoxelOpenGLWidget::removeRenderMesh(
    MeshObject& object,
    const MeshPartIndex& partIndex,
    MeshUpdateStatistics* statistics)
{
    MeshObject::GpuPartMap::iterator iterator =
        object.gpuParts.find(partIndex);

    if (iterator == object.gpuParts.end())
    {
        return;
    }

    const UpdateClock::time_point removalStart =
        UpdateClock::now();

    if (iterator->second->vertexBuffer.isCreated())
    {
        iterator->second->vertexBuffer.destroy();
    }

    if (iterator->second->vertexArray.isCreated())
    {
        iterator->second->vertexArray.destroy();
    }

    object.gpuParts.erase(iterator);

    if (statistics)
    {
        statistics->gpuRemovalMilliseconds +=
            elapsedMilliseconds(
                removalStart,
                UpdateClock::now());
    }
}

void VoxelOpenGLWidget::clearObjectRenderMeshes(MeshObject& object)
{
    while (!object.gpuParts.empty())
    {
        removeRenderMesh(object, object.gpuParts.begin()->first);
    }
}

void VoxelOpenGLWidget::clearAllRenderMeshes()
{
    for (MeshObjectMap::iterator iterator = m_meshObjects.begin(); iterator != m_meshObjects.end(); ++iterator)
    {
        clearObjectRenderMeshes(*iterator->second);
    }
}

/// 绘制

bool VoxelOpenGLWidget::hasVisibleRenderMeshes() const
{
    for (MeshObjectMap::const_iterator iterator = m_meshObjects.begin(); iterator != m_meshObjects.end(); ++iterator)
    {
        if (iterator->second->visible && !iterator->second->gpuParts.empty())
        {
            return true;
        }
    }

    return false;
}

void VoxelOpenGLWidget::drawBackground()
{
    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glDepthMask(GL_FALSE);
    m_functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    m_backgroundProgram.bind();
    m_backgroundVao.bind();
    m_functions->glDrawArrays(GL_TRIANGLES, 0, 3);
    m_backgroundVao.release();
    m_backgroundProgram.release();

    m_functions->glDepthMask(GL_TRUE);
    m_functions->glEnable(GL_DEPTH_TEST);
}

void VoxelOpenGLWidget::drawMeshObjects(const QVector3D& eye, const QVector3D& viewCenter)
{
    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    const float cameraDistance = (eye - viewCenter).length();
    const float nearPlane = (std::max)(0.001f, cameraDistance - m_radius * 1.75f);
    const float farPlane = (std::max)(nearPlane + 1.0f, cameraDistance + m_radius * 3.50f);

    QMatrix4x4 projection;
    projection.perspective(45.0f, aspect, nearPlane, farPlane);

    QMatrix4x4 view;
    view.lookAt(eye, viewCenter, QVector3D(0.0f, 0.0f, 1.0f));

    m_meshProgram.bind();
    m_meshProgram.setUniformValue("u_cameraPosition", eye);

    m_functions->glPolygonMode(GL_FRONT_AND_BACK, m_wireframe ? GL_LINE : GL_FILL);

    for (MeshObjectMap::const_iterator objectIterator = m_meshObjects.begin();
         objectIterator != m_meshObjects.end();
         ++objectIterator)
    {
        const MeshObject& object = *objectIterator->second;

        if (!object.visible || object.gpuParts.empty())
        {
            continue;
        }

        const QMatrix4x4 mvp = projection * view * object.modelMatrix;

        m_meshProgram.setUniformValue("u_model", object.modelMatrix);
        m_meshProgram.setUniformValue("u_mvp", mvp);
        m_meshProgram.setUniformValue("u_normalMatrix", object.modelMatrix.normalMatrix());

        for (MeshObject::GpuPartMap::const_iterator partIterator = object.gpuParts.begin();
             partIterator != object.gpuParts.end();
             ++partIterator)
        {
            partIterator->second->vertexArray.bind();
            m_functions->glDrawArrays(GL_TRIANGLES, 0, partIterator->second->vertexCount);
            partIterator->second->vertexArray.release();
        }
    }

    m_functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    m_meshProgram.release();
}

/// 相机

void VoxelOpenGLWidget::updateBounds()
{
    bool hasPoint = false;
    QVector3D minimum;
    QVector3D maximum;

    for (MeshObjectMap::const_iterator objectIterator = m_meshObjects.begin(); objectIterator != m_meshObjects.end(); ++objectIterator)
    {
        const MeshObject& object = *objectIterator->second;

        if (!object.visible)
        {
            continue;
        }

        expandWorldBounds(
            object.localBounds,
            object.modelMatrix,
            hasPoint,
            minimum,
            maximum);
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

QVector3D VoxelOpenGLWidget::cameraDirection() const
{
    const float yaw = degreeToRadian(m_yaw);
    const float pitch = degreeToRadian(m_pitch);

    return QVector3D(
        std::cos(yaw) * std::cos(pitch),
        std::sin(yaw) * std::cos(pitch),
        std::sin(pitch)).normalized();
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