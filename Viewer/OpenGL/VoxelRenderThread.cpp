#include "VoxelRenderThread.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <QCoreApplication>
#include <QMutexLocker>
#include <QOffscreenSurface>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QString>

#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"

namespace
{

const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const float MinimumCameraDistanceRatio = 1.0e-6f; // 相机距离最低保留场景参考半径的一百万分之一，仅用于避免eye与viewCenter重合。
const float MinimumCameraDistanceAbsolute = 1.0e-6f; // 极小场景仍保留1e-6世界单位观察距离，属于数值安全下限而非用户缩放范围。
const std::size_t MeshVertexFloatCount = 10; // Mesh顶点包含位置3、法线3和颜色4个float。
const int MeshPositionFloatOffset = 0; // Mesh顶点位置属性float偏移。
const int MeshNormalFloatOffset = 3; // Mesh顶点法线属性float偏移。
const int MeshColorFloatOffset = 6; // Mesh顶点颜色属性float偏移。
const std::size_t LineVertexFloatCount = 7; // Line顶点包含位置3和颜色4个float。
const int LinePositionFloatOffset = 0; // Line顶点位置属性float偏移。
const int LineColorFloatOffset = 3; // Line顶点颜色属性float偏移。

static_assert(sizeof(MyVoxel::Display_MeshVertex) == MeshVertexFloatCount * sizeof(float),
              "Display_MeshVertex must contain exactly ten consecutive float values.");
static_assert(sizeof(MyVoxel::Display_LineVertex) == LineVertexFloatCount * sizeof(float),
              "Display_LineVertex must contain exactly seven consecutive float values.");

typedef std::chrono::steady_clock RenderClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const RenderClock::time_point& begin, const RenderClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 将字节数量转换为QOpenGLBuffer使用的有符号整数。
int checkedBufferByteSize(std::size_t byteSize)
{
    assert(byteSize <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
    return static_cast<int>(byteSize);
}

// 返回当前相机参考半径对应的最小有效观察距离，仅防止视图矩阵数值退化。
float minimumCameraDistance(float radius)
{
    return (std::max)(radius * MinimumCameraDistanceRatio, MinimumCameraDistanceAbsolute);
}

// 返回观察中心指向相机位置的世界单位方向。
QVector3D cameraDirection(const MyVoxelViewer::RenderCameraState& camera)
{
    return camera.rotation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
}

// 返回当前相机局部上方向对应的世界单位方向。
QVector3D cameraUp(const MyVoxelViewer::RenderCameraState& camera)
{
    return camera.rotation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
}

}

namespace MyVoxelViewer
{

RenderFrameReadyEvent::RenderFrameReadyEvent(std::uint64_t frameId, unsigned int textureId, std::uint64_t frameVersion,
                                             const QSize& textureSize, const MeshUpdateStatistics& statistics)
    : QEvent(eventType())
    , m_frameId(frameId)
    , m_textureId(textureId)
    , m_frameVersion(frameVersion)
    , m_textureSize(textureSize)
    , m_statistics(statistics)
{
}

QEvent::Type RenderFrameReadyEvent::eventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

std::uint64_t RenderFrameReadyEvent::frameId() const { return m_frameId; }
unsigned int RenderFrameReadyEvent::textureId() const { return m_textureId; }
std::uint64_t RenderFrameReadyEvent::frameVersion() const { return m_frameVersion; }
const QSize& RenderFrameReadyEvent::textureSize() const { return m_textureSize; }
const MeshUpdateStatistics& RenderFrameReadyEvent::statistics() const { return m_statistics; }

RenderFailureEvent::RenderFailureEvent(const QString& message)
    : QEvent(eventType())
    , m_message(message)
{
}

QEvent::Type RenderFailureEvent::eventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

const QString& RenderFailureEvent::message() const { return m_message; }

VoxelRenderThread::PendingObjectChange::PendingObjectChange()
    : action(PendingNone)
    , resourceKind(MyVoxel::Display_ResourceKind::Mesh)
    , hasResourceKind(false)
    , stateVersion(0)
    , hasStateVersion(false)
    , dynamicUsage(false)
    , hasUsage(false)
    , hasModelMatrix(false)
    , visible(true)
    , hasVisible(false)
    , lineWidth(1.0f)
    , hasLineWidth(false)
{
    modelMatrix.setToIdentity();
}

VoxelRenderThread::PendingBatch::PendingBatch()
    : clearObjects(false)
    , hasCamera(false)
    , hasWireframe(false)
    , wireframe(false)
    , requestFrame(false)
    , cacheMilliseconds(0.0)
    , cpuCopyMilliseconds(0.0)
    , stagedCpuPartCount(0)
{
}

bool VoxelRenderThread::PendingBatch::isEmpty() const
{
    return !clearObjects && objects.empty() && !hasCamera && !hasWireframe && !requestFrame &&
           cacheMilliseconds == 0.0 && cpuCopyMilliseconds == 0.0 && stagedCpuPartCount == 0;
}

void VoxelRenderThread::PendingBatch::clear()
{
    clearObjects = false;
    objects.clear();
    hasCamera = false;
    hasWireframe = false;
    requestFrame = false;
    cacheMilliseconds = 0.0;
    cpuCopyMilliseconds = 0.0;
    stagedCpuPartCount = 0;
}

struct VoxelRenderThread::RenderState
{
    struct RenderPart
    {
        RenderPart()
            : vertexBuffer(QOpenGLBuffer::VertexBuffer)
            , vertexCount(0)
            , triangleCount(0)
            , segmentCount(0)
        {
        }

        QOpenGLVertexArrayObject vertexArray; // 当前分片顶点属性状态。
        QOpenGLBuffer vertexBuffer; // 当前分片连续顶点数据。
        int vertexCount; // 当前分片需要绘制的OpenGL顶点数量。
        std::size_t triangleCount; // Mesh分片包含的三角形数量。
        std::size_t segmentCount; // Line分片包含的独立线段数量。
    };

    struct RenderObject
    {
        typedef std::map<MyVoxel::Display_ObjectPartId, RenderPart*> PartMap;
        typedef std::map<MyVoxel::Display_ObjectPartId, std::uint64_t> PartVersionMap;

        RenderObject()
            : resourceKind(MyVoxel::Display_ResourceKind::Mesh)
            , dynamicUsage(false)
            , stateVersion(0)
            , visible(true)
            , lineWidth(1.0f)
        {
            modelMatrix.setToIdentity();
        }

        MyVoxel::Display_ResourceKind resourceKind; // 当前对象统一资源类型。
        bool dynamicUsage; // 当前对象GPU缓冲是否使用DynamicDraw。
        std::uint64_t stateVersion; // 后台最近应用的对象状态版本。
        QMatrix4x4 modelMatrix; // 对象局部空间到显示世界空间的模型矩阵。
        bool visible; // 对象是否参与后台绘制。
        float lineWidth; // Line对象期望OpenGL线宽。
        PartMap parts; // 当前对象全部活动GPU分片。
        PartVersionMap partVersions; // 当前全部已知partId版本，删除后继续保存版本墓碑。
    };

    struct FrameResource
    {
        FrameResource()
            : id(0)
            , framebuffer(0)
            , colorTexture(0)
            , depthStencilBuffer(0)
            , ready(false)
            , displayed(false)
            , retired(false)
        {
        }

        std::uint64_t id;
        QSize size;
        unsigned int framebuffer;
        unsigned int colorTexture;
        unsigned int depthStencilBuffer;
        bool ready;
        bool displayed;
        bool retired;
    };

    typedef std::map<MyVoxel::Display_ObjectId, RenderObject*> ObjectMap;
    typedef std::map<std::uint64_t, FrameResource> FrameMap;

    RenderState()
        : functions(nullptr)
        , initialized(false)
        , wireframe(false)
        , nextFrameId(1)
        , nextFrameVersion(1)
        , readyFrameId(0)
        , displayedFrameId(0)
    {
    }

    bool initialize(QOpenGLContext* context, QString& errorMessage)
    {
        functions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();

        if (!functions)
        {
            errorMessage = QStringLiteral("OpenGL 3.3 Core function table is unavailable.");
            return false;
        }

        functions->initializeOpenGLFunctions();
        initialized = true;
        functions->glEnable(GL_DEPTH_TEST);
        functions->glDepthFunc(GL_LEQUAL);
        functions->glEnable(GL_BLEND);
        functions->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        functions->glDisable(GL_CULL_FACE);
        functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

        if (!createShaderPrograms(errorMessage))
        {
            return false;
        }

        if (!backgroundVao.create())
        {
            errorMessage = QStringLiteral("Failed to create the background OpenGL vertex array object.");
            return false;
        }

        return true;
    }

    void cleanup()
    {
        if (!initialized)
        {
            return;
        }

        clearObjects(nullptr);

        for (FrameMap::iterator iterator = frames.begin(); iterator != frames.end(); ++iterator)
        {
            destroyFrame(iterator->second);
        }

        frames.clear();
        currentFrameIds.clear();
        readyFrameId = 0;
        displayedFrameId = 0;

        if (backgroundVao.isCreated())
        {
            backgroundVao.destroy();
        }

        backgroundProgram.removeAllShaders();
        meshProgram.removeAllShaders();
        lineProgram.removeAllShaders();
        initialized = false;
    }

    bool createShaderPrograms(QString& errorMessage)
    {
        const char* backgroundVertexShader =
            "#version 330 core\n"
            "out vec2 v_uv;\n"
            "void main()\n"
            "{\n"
            "    vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));\n"
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
            "    if (!gl_FrontFacing) normal = -normal;\n"
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

        const char* lineVertexShader =
            "#version 330 core\n"
            "layout(location = 0) in vec3 a_position;\n"
            "layout(location = 1) in vec4 a_color;\n"
            "uniform mat4 u_mvp;\n"
            "out vec4 v_color;\n"
            "void main()\n"
            "{\n"
            "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
            "    v_color = a_color;\n"
            "}\n";

        const char* lineFragmentShader =
            "#version 330 core\n"
            "in vec4 v_color;\n"
            "out vec4 fragColor;\n"
            "void main()\n"
            "{\n"
            "    fragColor = v_color;\n"
            "}\n";

        if (!backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, backgroundVertexShader) ||
            !backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, backgroundFragmentShader) ||
            !backgroundProgram.link())
        {
            errorMessage = QStringLiteral("Background shader initialization failed: ") + backgroundProgram.log();
            return false;
        }

        if (!meshProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, meshVertexShader) ||
            !meshProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, meshFragmentShader) ||
            !meshProgram.link())
        {
            errorMessage = QStringLiteral("Mesh shader initialization failed: ") + meshProgram.log();
            return false;
        }

        if (!lineProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, lineVertexShader) ||
            !lineProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, lineFragmentShader) ||
            !lineProgram.link())
        {
            errorMessage = QStringLiteral("Line shader initialization failed: ") + lineProgram.log();
            return false;
        }

        return true;
    }

    void applyBatch(const PendingBatch& batch, MeshUpdateStatistics& statistics)
    {
        statistics.cacheAndVersionMilliseconds += batch.cacheMilliseconds;
        statistics.cpuStagingCopyMilliseconds += batch.cpuCopyMilliseconds;
        statistics.stagedCpuPartCount += batch.stagedCpuPartCount;

        if (batch.clearObjects)
        {
            clearObjects(&statistics);
        }

        for (std::map<MyVoxel::Display_ObjectId, PendingObjectChange>::const_iterator iterator = batch.objects.begin();
             iterator != batch.objects.end(); ++iterator)
        {
            applyObjectChange(iterator->first, iterator->second, statistics);
        }

        if (batch.hasCamera)
        {
            camera = batch.camera;
            ensureFramePool(camera.viewportSize);
        }

        if (batch.hasWireframe)
        {
            wireframe = batch.wireframe;
        }
    }

    void applyObjectChange(MyVoxel::Display_ObjectId objectId, const PendingObjectChange& change, MeshUpdateStatistics& statistics)
    {
        ObjectMap::iterator objectIterator = objects.find(objectId);

        if (change.action == PendingRemove)
        {
            if (objectIterator != objects.end())
            {
                clearObject(*objectIterator->second, &statistics);
                delete objectIterator->second;
                objects.erase(objectIterator);
            }

            return;
        }

        if (change.action == PendingReplace)
        {
            if (objectIterator != objects.end())
            {
                clearObject(*objectIterator->second, &statistics);
                delete objectIterator->second;
                objects.erase(objectIterator);
            }

            RenderObject* object = new RenderObject();
            object->resourceKind = change.resourceKind;
            object->dynamicUsage = change.dynamicUsage;
            object->stateVersion = change.stateVersion;
            object->modelMatrix = change.modelMatrix;
            object->visible = change.visible;
            object->lineWidth = change.lineWidth;
            const std::pair<ObjectMap::iterator, bool> inserted = objects.insert(std::make_pair(objectId, object));

            if (!inserted.second)
            {
                delete object;
                return;
            }

            objectIterator = inserted.first;
        }

        if (objectIterator == objects.end())
        {
            return;
        }

        RenderObject& object = *objectIterator->second;

        if (change.action != PendingReplace && change.hasStateVersion && change.stateVersion > object.stateVersion)
        {
            if (change.hasUsage)
            {
                object.dynamicUsage = change.dynamicUsage;
            }

            if (change.hasModelMatrix)
            {
                object.modelMatrix = change.modelMatrix;
            }

            if (change.hasVisible)
            {
                object.visible = change.visible;
            }

            if (change.hasLineWidth && object.resourceKind == MyVoxel::Display_ResourceKind::Line)
            {
                object.lineWidth = change.lineWidth;
            }

            object.stateVersion = change.stateVersion;
        }

        if (change.hasResourceKind && change.resourceKind != object.resourceKind)
        {
            return;
        }

        for (std::map<MyVoxel::Display_ObjectPartId, MyVoxel::Display_ObjectPartUpdate>::const_iterator iterator =
                 change.parts.begin(); iterator != change.parts.end(); ++iterator)
        {
            applyPartUpdate(object, iterator->second, statistics);
        }
    }

    void applyPartUpdate(RenderObject& object, const MyVoxel::Display_ObjectPartUpdate& update, MeshUpdateStatistics& statistics)
    {
        RenderObject::PartVersionMap::const_iterator versionIterator = object.partVersions.find(update.partId);

        if (versionIterator != object.partVersions.end() && update.version <= versionIterator->second)
        {
            return;
        }

        if (update.operation == MyVoxel::Display_ObjectPartOperation::Remove)
        {
            removePart(object, update.partId, &statistics);
            object.partVersions[update.partId] = update.version;
            return;
        }

        if (!update.resource || update.resource->kind() != object.resourceKind)
        {
            return;
        }

        bool uploaded = false;

        if (object.resourceKind == MyVoxel::Display_ResourceKind::Mesh)
        {
            uploaded = uploadMeshPart(object, update.partId,
                                      static_cast<const MyVoxel::Display_MeshResource&>(*update.resource), statistics);
        }
        else
        {
            uploaded = uploadLinePart(object, update.partId,
                                      static_cast<const MyVoxel::Display_LineResource&>(*update.resource), statistics);
        }

        if (uploaded)
        {
            object.partVersions[update.partId] = update.version;
        }
    }

    RenderPart* createOrFindPart(RenderObject& object, MyVoxel::Display_ObjectPartId partId, bool& created)
    {
        RenderObject::PartMap::iterator iterator = object.parts.find(partId);

        if (iterator != object.parts.end())
        {
            created = false;
            return iterator->second;
        }

        RenderPart* part = new RenderPart();
        created = true;

        if (!part->vertexArray.create() || !part->vertexBuffer.create())
        {
            if (part->vertexBuffer.isCreated()) part->vertexBuffer.destroy();
            if (part->vertexArray.isCreated()) part->vertexArray.destroy();
            delete part;
            return nullptr;
        }

        return part;
    }

    bool commitCreatedPart(RenderObject& object, MyVoxel::Display_ObjectPartId partId, RenderPart* part, bool created,
                           MeshUpdateStatistics& statistics)
    {
        if (!created)
        {
            ++statistics.reusedGpuPartCount;
            return true;
        }

        const std::pair<RenderObject::PartMap::iterator, bool> inserted = object.parts.insert(std::make_pair(partId, part));

        if (!inserted.second)
        {
            if (part->vertexBuffer.isCreated()) part->vertexBuffer.destroy();
            if (part->vertexArray.isCreated()) part->vertexArray.destroy();
            delete part;
            return false;
        }

        ++statistics.createdGpuPartCount;
        return true;
    }

    bool uploadMeshPart(RenderObject& object, MyVoxel::Display_ObjectPartId partId,
                        const MyVoxel::Display_MeshResource& resource, MeshUpdateStatistics& statistics)
    {
        assert(resource.isValid() && resource.vertexCount() > 0);
        const RenderClock::time_point uploadStart = RenderClock::now();
        bool created = false;
        RenderPart* part = createOrFindPart(object, partId, created);

        if (!part)
        {
            return false;
        }

        const std::vector<MyVoxel::Display_MeshVertex>& vertices = resource.vertices();
        part->vertexArray.bind();
        part->vertexBuffer.bind();
        part->vertexBuffer.setUsagePattern(object.dynamicUsage ? QOpenGLBuffer::DynamicDraw : QOpenGLBuffer::StaticDraw);
        part->vertexBuffer.allocate(&vertices[0], checkedBufferByteSize(resource.memoryByteSize()));

        if (created)
        {
            meshProgram.bind();
            meshProgram.enableAttributeArray(0);
            meshProgram.setAttributeBuffer(0, GL_FLOAT, MeshPositionFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.enableAttributeArray(1);
            meshProgram.setAttributeBuffer(1, GL_FLOAT, MeshNormalFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.enableAttributeArray(2);
            meshProgram.setAttributeBuffer(2, GL_FLOAT, MeshColorFloatOffset * static_cast<int>(sizeof(float)), 4,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.release();
        }

        part->vertexBuffer.release();
        part->vertexArray.release();
        assert(resource.vertexCount() <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
        part->vertexCount = static_cast<int>(resource.vertexCount());
        part->triangleCount = resource.triangleCount();
        part->segmentCount = 0;

        if (!commitCreatedPart(object, partId, part, created, statistics))
        {
            return false;
        }

        statistics.gpuUploadMilliseconds += elapsedMilliseconds(uploadStart, RenderClock::now());
        ++statistics.uploadedPartCount;
        return true;
    }

    bool uploadLinePart(RenderObject& object, MyVoxel::Display_ObjectPartId partId,
                        const MyVoxel::Display_LineResource& resource, MeshUpdateStatistics& statistics)
    {
        assert(resource.isValid() && resource.vertexCount() > 0);
        const RenderClock::time_point uploadStart = RenderClock::now();
        bool created = false;
        RenderPart* part = createOrFindPart(object, partId, created);

        if (!part)
        {
            return false;
        }

        const std::vector<MyVoxel::Display_LineVertex>& vertices = resource.vertices();
        part->vertexArray.bind();
        part->vertexBuffer.bind();
        part->vertexBuffer.setUsagePattern(object.dynamicUsage ? QOpenGLBuffer::DynamicDraw : QOpenGLBuffer::StaticDraw);
        part->vertexBuffer.allocate(&vertices[0], checkedBufferByteSize(resource.memoryByteSize()));

        if (created)
        {
            lineProgram.bind();
            lineProgram.enableAttributeArray(0);
            lineProgram.setAttributeBuffer(0, GL_FLOAT, LinePositionFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           LineVertexFloatCount * static_cast<int>(sizeof(float)));
            lineProgram.enableAttributeArray(1);
            lineProgram.setAttributeBuffer(1, GL_FLOAT, LineColorFloatOffset * static_cast<int>(sizeof(float)), 4,
                                           LineVertexFloatCount * static_cast<int>(sizeof(float)));
            lineProgram.release();
        }

        part->vertexBuffer.release();
        part->vertexArray.release();
        assert(resource.vertexCount() <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
        part->vertexCount = static_cast<int>(resource.vertexCount());
        part->triangleCount = 0;
        part->segmentCount = resource.segmentCount();

        if (!commitCreatedPart(object, partId, part, created, statistics))
        {
            return false;
        }

        statistics.gpuUploadMilliseconds += elapsedMilliseconds(uploadStart, RenderClock::now());
        ++statistics.uploadedPartCount;
        return true;
    }

    void removePart(RenderObject& object, MyVoxel::Display_ObjectPartId partId, MeshUpdateStatistics* statistics)
    {
        RenderObject::PartMap::iterator iterator = object.parts.find(partId);

        if (iterator == object.parts.end())
        {
            return;
        }

        const RenderClock::time_point removalStart = RenderClock::now();
        if (iterator->second->vertexBuffer.isCreated()) iterator->second->vertexBuffer.destroy();
        if (iterator->second->vertexArray.isCreated()) iterator->second->vertexArray.destroy();
        delete iterator->second;
        object.parts.erase(iterator);

        if (statistics)
        {
            statistics->gpuRemovalMilliseconds += elapsedMilliseconds(removalStart, RenderClock::now());
            ++statistics->removedPartCount;
        }
    }

    void clearObject(RenderObject& object, MeshUpdateStatistics* statistics)
    {
        while (!object.parts.empty())
        {
            removePart(object, object.parts.begin()->first, statistics);
        }

        object.partVersions.clear();
    }

    void clearObjects(MeshUpdateStatistics* statistics)
    {
        for (ObjectMap::iterator iterator = objects.begin(); iterator != objects.end(); ++iterator)
        {
            clearObject(*iterator->second, statistics);
            delete iterator->second;
        }

        objects.clear();
    }

    void ensureFramePool(const QSize& requestedSize)
    {
        const QSize size((std::max)(requestedSize.width(), 1), (std::max)(requestedSize.height(), 1));

        if (currentFrameSize == size && currentFrameIds.size() == 2)
        {
            return;
        }

        currentFrameSize = size;

        for (FrameMap::iterator iterator = frames.begin(); iterator != frames.end(); ++iterator)
        {
            iterator->second.retired = true;
        }

        currentFrameIds.clear();

        for (int index = 0; index < 2; ++index)
        {
            const std::uint64_t frameId = createFrame(size);

            if (frameId != 0)
            {
                currentFrameIds.push_back(frameId);
            }
        }

        cleanupRetiredFrames();
    }

    std::uint64_t createFrame(const QSize& size)
    {
        FrameResource frame;
        frame.id = nextFrameId++;
        frame.size = size;
        functions->glGenTextures(1, &frame.colorTexture);
        functions->glBindTexture(GL_TEXTURE_2D, frame.colorTexture);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        functions->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size.width(), size.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        functions->glBindTexture(GL_TEXTURE_2D, 0);

        functions->glGenRenderbuffers(1, &frame.depthStencilBuffer);
        functions->glBindRenderbuffer(GL_RENDERBUFFER, frame.depthStencilBuffer);
        functions->glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, size.width(), size.height());
        functions->glBindRenderbuffer(GL_RENDERBUFFER, 0);

        functions->glGenFramebuffers(1, &frame.framebuffer);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, frame.framebuffer);
        const unsigned int drawBuffer = GL_COLOR_ATTACHMENT0;
        functions->glDrawBuffers(1, &drawBuffer);
        functions->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, frame.colorTexture, 0);
        functions->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, frame.depthStencilBuffer);
        const unsigned int status = functions->glCheckFramebufferStatus(GL_FRAMEBUFFER);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            destroyFrame(frame);
            return 0;
        }

        frames.insert(std::make_pair(frame.id, frame));
        return frame.id;
    }

    void destroyFrame(FrameResource& frame)
    {
        if (frame.framebuffer != 0)
        {
            functions->glDeleteFramebuffers(1, &frame.framebuffer);
            frame.framebuffer = 0;
        }

        if (frame.depthStencilBuffer != 0)
        {
            functions->glDeleteRenderbuffers(1, &frame.depthStencilBuffer);
            frame.depthStencilBuffer = 0;
        }

        if (frame.colorTexture != 0)
        {
            functions->glDeleteTextures(1, &frame.colorTexture);
            frame.colorTexture = 0;
        }
    }

    void cleanupRetiredFrames()
    {
        FrameMap::iterator iterator = frames.begin();

        while (iterator != frames.end())
        {
            if (iterator->second.retired && !iterator->second.ready && !iterator->second.displayed)
            {
                destroyFrame(iterator->second);
                frames.erase(iterator++);
            }
            else
            {
                ++iterator;
            }
        }
    }

    FrameResource* freeCurrentFrame()
    {
        for (std::size_t index = 0; index < currentFrameIds.size(); ++index)
        {
            FrameMap::iterator iterator = frames.find(currentFrameIds[index]);

            if (iterator != frames.end() && !iterator->second.ready && !iterator->second.displayed && !iterator->second.retired)
            {
                return &iterator->second;
            }
        }

        return nullptr;
    }

    bool renderFrame(MeshUpdateStatistics& statistics, std::uint64_t& frameId, unsigned int& textureId,
                     std::uint64_t& frameVersion, QSize& textureSize)
    {
        ensureFramePool(camera.viewportSize);
        FrameResource* frame = freeCurrentFrame();

        if (!frame)
        {
            return false;
        }

        const RenderClock::time_point renderStart = RenderClock::now();
        functions->glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
        functions->glViewport(0, 0, frame->size.width(), frame->size.height());
        functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        drawBackground();
        drawObjects(statistics);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        functions->glFinish(); // GPU等待仅发生在后台线程，确保GUI收到事件后可直接无阻塞采样共享纹理。
        statistics.renderMilliseconds += elapsedMilliseconds(renderStart, RenderClock::now());
        statistics.totalMilliseconds = statistics.cacheAndVersionMilliseconds + statistics.cpuStagingCopyMilliseconds +
            statistics.contextAcquireMilliseconds + statistics.vertexExpansionMilliseconds + statistics.gpuUploadMilliseconds +
            statistics.gpuRemovalMilliseconds + statistics.contextReleaseMilliseconds + statistics.renderMilliseconds;
        statistics.frameVersion = nextFrameVersion++;

        frame->ready = true;
        readyFrameId = frame->id;
        frameId = frame->id;
        textureId = frame->colorTexture;
        frameVersion = statistics.frameVersion;
        textureSize = frame->size;
        return true;
    }

    void acceptPresentedFrame(std::uint64_t frameId)
    {
        FrameMap::iterator newIterator = frames.find(frameId);

        if (newIterator == frames.end())
        {
            return;
        }

        if (displayedFrameId != 0 && displayedFrameId != frameId)
        {
            FrameMap::iterator oldIterator = frames.find(displayedFrameId);

            if (oldIterator != frames.end())
            {
                oldIterator->second.displayed = false;
            }
        }

        newIterator->second.ready = false;
        newIterator->second.displayed = true;
        displayedFrameId = frameId;

        if (readyFrameId == frameId)
        {
            readyFrameId = 0;
        }

        cleanupRetiredFrames();
    }

    void drawBackground()
    {
        functions->glDisable(GL_DEPTH_TEST);
        functions->glDepthMask(GL_FALSE);
        functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        backgroundProgram.bind();
        backgroundVao.bind();
        functions->glDrawArrays(GL_TRIANGLES, 0, 3);
        backgroundVao.release();
        backgroundProgram.release();
        functions->glDepthMask(GL_TRUE);
        functions->glEnable(GL_DEPTH_TEST);
    }

    void drawObjects(MeshUpdateStatistics& statistics)
    {
        const QVector3D viewCenter = camera.center + camera.viewOffset;
        const float radius = (std::max)(camera.radius, MinimumRadius);
        const float cameraDistance = (std::max)(radius * camera.cameraScale, minimumCameraDistance(radius));
        const QVector3D eye = viewCenter + cameraDirection(camera) * cameraDistance;
        const float aspect = camera.viewportSize.height() > 0
            ? static_cast<float>(camera.viewportSize.width()) / static_cast<float>(camera.viewportSize.height()) : 1.0f;
        const float nearPlane = (std::max)(0.001f, cameraDistance - radius * 1.75f);
        const float farPlane = (std::max)(nearPlane + 1.0f, cameraDistance + radius * 3.50f);

        QMatrix4x4 projection;
        projection.perspective(45.0f, aspect, nearPlane, farPlane);
        QMatrix4x4 view;
        view.lookAt(eye, viewCenter, cameraUp(camera));

        meshProgram.bind();
        meshProgram.setUniformValue("u_cameraPosition", eye);
        functions->glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);

        for (ObjectMap::const_iterator objectIterator = objects.begin(); objectIterator != objects.end(); ++objectIterator)
        {
            const RenderObject& object = *objectIterator->second;

            if (object.resourceKind != MyVoxel::Display_ResourceKind::Mesh || !object.visible || object.parts.empty())
            {
                continue;
            }

            const QMatrix4x4 mvp = projection * view * object.modelMatrix;
            meshProgram.setUniformValue("u_model", object.modelMatrix);
            meshProgram.setUniformValue("u_mvp", mvp);
            meshProgram.setUniformValue("u_normalMatrix", object.modelMatrix.normalMatrix());

            for (RenderObject::PartMap::const_iterator partIterator = object.parts.begin(); partIterator != object.parts.end(); ++partIterator)
            {
                partIterator->second->vertexArray.bind();
                functions->glDrawArrays(GL_TRIANGLES, 0, partIterator->second->vertexCount);
                partIterator->second->vertexArray.release();
                ++statistics.drawCallCount;
                statistics.triangleCount += partIterator->second->triangleCount;
            }
        }

        functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        meshProgram.release();

        lineProgram.bind();

        for (ObjectMap::const_iterator objectIterator = objects.begin(); objectIterator != objects.end(); ++objectIterator)
        {
            const RenderObject& object = *objectIterator->second;

            if (object.resourceKind != MyVoxel::Display_ResourceKind::Line || !object.visible || object.parts.empty())
            {
                continue;
            }

            lineProgram.setUniformValue("u_mvp", projection * view * object.modelMatrix);
            functions->glLineWidth(object.lineWidth);

            for (RenderObject::PartMap::const_iterator partIterator = object.parts.begin(); partIterator != object.parts.end(); ++partIterator)
            {
                partIterator->second->vertexArray.bind();
                functions->glDrawArrays(GL_LINES, 0, partIterator->second->vertexCount);
                partIterator->second->vertexArray.release();
                ++statistics.drawCallCount;
                ++statistics.lineDrawCallCount;
                statistics.lineSegmentCount += partIterator->second->segmentCount;
            }
        }

        functions->glLineWidth(1.0f);
        lineProgram.release();
    }

    QOpenGLFunctions_3_3_Core* functions;
    bool initialized;
    bool wireframe;
    RenderCameraState camera;
    QOpenGLShaderProgram backgroundProgram;
    QOpenGLShaderProgram meshProgram;
    QOpenGLShaderProgram lineProgram;
    QOpenGLVertexArrayObject backgroundVao;
    ObjectMap objects;
    FrameMap frames;
    std::vector<std::uint64_t> currentFrameIds;
    QSize currentFrameSize;
    std::uint64_t nextFrameId;
    std::uint64_t nextFrameVersion;
    std::uint64_t readyFrameId;
    std::uint64_t displayedFrameId;
};

VoxelRenderThread::VoxelRenderThread(QObject* parent)
    : QThread(parent)
    , m_stopRequested(false)
    , m_started(false)
    , m_readyFrameOutstanding(false)
    , m_frameAcceptancePending(false)
    , m_readyFrameId(0)
    , m_readyFrameVersion(0)
    , m_acceptedFrameId(0)
    , m_acceptedFrameVersion(0)
    , m_context(nullptr)
    , m_surface(nullptr)
    , m_frameReceiver(nullptr)
    , m_guiThread(nullptr)
{
}

VoxelRenderThread::~VoxelRenderThread()
{
    stopRendering();
}

bool VoxelRenderThread::startRendering(QOpenGLContext* context, QOffscreenSurface* surface,
                                       QObject* frameReceiver, QThread* guiThread)
{
    QMutexLocker locker(&m_mutex);

    if (m_started || !context || !surface || !surface->isValid() || !frameReceiver || !guiThread)
    {
        return false;
    }

    context->moveToThread(this);
    m_context = context;
    m_surface = surface;
    m_frameReceiver = frameReceiver;
    m_guiThread = guiThread;
    m_stopRequested = false;
    m_readyFrameOutstanding = false;
    m_frameAcceptancePending = false;
    m_readyFrameId = 0;
    m_readyFrameVersion = 0;
    m_acceptedFrameId = 0;
    m_acceptedFrameVersion = 0;
    m_started = true;
    start();
    return true;
}

void VoxelRenderThread::stopRendering()
{
    const bool running = isRunning();

    {
        QMutexLocker locker(&m_mutex);

        if (m_started)
        {
            m_stopRequested = true;
            m_waitCondition.wakeAll();
        }
    }

    if (running && QThread::currentThread() != this)
    {
        wait();
    }
}

bool VoxelRenderThread::isRendering() const
{
    QMutexLocker locker(&m_mutex);
    return m_started && !m_stopRequested;
}

void VoxelRenderThread::enqueueReplaceObject(const MyVoxel::Display_ObjectSnapshot& snapshot,
                                             double cacheMilliseconds, double cpuCopyMilliseconds)
{
    assert(snapshot.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[snapshot.objectId];
    change = PendingObjectChange();
    change.action = PendingReplace;
    change.resourceKind = snapshot.resourceKind;
    change.hasResourceKind = true;
    change.stateVersion = snapshot.stateVersion;
    change.hasStateVersion = true;
    change.dynamicUsage = snapshot.usage == MyVoxel::Display_ObjectUsage::Dynamic;
    change.hasUsage = true;
    change.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
    change.hasModelMatrix = true;
    change.visible = snapshot.visible;
    change.hasVisible = true;
    change.lineWidth = snapshot.lineWidth;
    change.hasLineWidth = true;

    for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
    {
        const MyVoxel::Display_ObjectPartSnapshot& part = snapshot.parts[index];
        change.parts[part.partId] =
            MyVoxel::Display_ObjectPartUpdate::replacement(part.partId, part.version, part.resourceId, part.resource);
        ++m_pending.stagedCpuPartCount;
    }

    m_pending.cacheMilliseconds += cacheMilliseconds;
    m_pending.cpuCopyMilliseconds += cpuCopyMilliseconds;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueUpdateParts(const MyVoxel::Display_ObjectPartsUpdate& update,
                                           double cacheMilliseconds, double cpuCopyMilliseconds)
{
    assert(update.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[update.objectId];

    if (change.action == PendingRemove)
    {
        return;
    }

    if (change.hasResourceKind && change.resourceKind != update.resourceKind)
    {
        return;
    }

    change.resourceKind = update.resourceKind;
    change.hasResourceKind = true;

    for (std::size_t index = 0; index < update.parts.size(); ++index)
    {
        const MyVoxel::Display_ObjectPartUpdate& source = update.parts[index];
        std::map<MyVoxel::Display_ObjectPartId, MyVoxel::Display_ObjectPartUpdate>::iterator iterator =
            change.parts.find(source.partId);

        if (iterator != change.parts.end() && source.version <= iterator->second.version)
        {
            continue;
        }

        change.parts[source.partId] = source;

        if (source.operation == MyVoxel::Display_ObjectPartOperation::Replace)
        {
            ++m_pending.stagedCpuPartCount;
        }
    }

    m_pending.cacheMilliseconds += cacheMilliseconds;
    m_pending.cpuCopyMilliseconds += cpuCopyMilliseconds;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update)
{
    assert(update.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[update.objectId];

    if (change.action == PendingRemove)
    {
        return;
    }

    if (change.hasStateVersion && update.stateVersion <= change.stateVersion)
    {
        return;
    }

    change.stateVersion = update.stateVersion;
    change.hasStateVersion = true;

    if (update.hasUsage)
    {
        change.dynamicUsage = update.usage == MyVoxel::Display_ObjectUsage::Dynamic;
        change.hasUsage = true;
    }

    if (update.hasLocalToWorld)
    {
        change.modelMatrix = toQMatrix4x4(update.localToWorld);
        change.hasModelMatrix = true;
    }

    if (update.hasVisible)
    {
        change.visible = update.visible;
        change.hasVisible = true;
    }

    if (update.hasLineWidth)
    {
        change.lineWidth = update.lineWidth;
        change.hasLineWidth = true;
    }

    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueRemoveObject(MyVoxel::Display_ObjectId objectId)
{
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[objectId];
    change = PendingObjectChange();
    change.action = PendingRemove;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueClearObjects()
{
    QMutexLocker locker(&m_mutex);
    m_pending.clearObjects = true;
    m_pending.objects.clear();
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueCameraState(const RenderCameraState& state)
{
    QMutexLocker locker(&m_mutex);
    m_pending.camera = state;
    m_pending.hasCamera = true;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueWireframe(bool enabled)
{
    QMutexLocker locker(&m_mutex);
    m_pending.wireframe = enabled;
    m_pending.hasWireframe = true;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::requestFrame()
{
    QMutexLocker locker(&m_mutex);
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::acceptPresentedFrame(std::uint64_t frameId, std::uint64_t frameVersion)
{
    QMutexLocker locker(&m_mutex);

    if (!m_readyFrameOutstanding || frameId != m_readyFrameId || frameVersion != m_readyFrameVersion)
    {
        return;
    }

    m_readyFrameOutstanding = false;
    m_frameAcceptancePending = true;
    m_acceptedFrameId = frameId;
    m_acceptedFrameVersion = frameVersion;
    m_waitCondition.wakeOne();
}

VoxelRenderThread::PendingBatch VoxelRenderThread::takePendingBatch()
{
    PendingBatch batch;
    batch.clearObjects = m_pending.clearObjects;
    batch.objects.swap(m_pending.objects);
    batch.hasCamera = m_pending.hasCamera;
    batch.camera = m_pending.camera;
    batch.hasWireframe = m_pending.hasWireframe;
    batch.wireframe = m_pending.wireframe;
    batch.requestFrame = m_pending.requestFrame;
    batch.cacheMilliseconds = m_pending.cacheMilliseconds;
    batch.cpuCopyMilliseconds = m_pending.cpuCopyMilliseconds;
    batch.stagedCpuPartCount = m_pending.stagedCpuPartCount;
    m_pending.clear();
    return batch;
}

bool VoxelRenderThread::hasRunnableWorkLocked() const
{
    return m_stopRequested || !m_pending.isEmpty() || m_frameAcceptancePending;
}

void VoxelRenderThread::postFailure(const QString& message) const
{
    if (m_frameReceiver)
    {
        QCoreApplication::postEvent(m_frameReceiver, new RenderFailureEvent(message));
    }
}

void VoxelRenderThread::run()
{
    const RenderClock::time_point acquireStart = RenderClock::now();

    if (!m_context || !m_surface || !m_context->makeCurrent(m_surface))
    {
        postFailure(QStringLiteral("Failed to make the background OpenGL context current."));

        if (m_context && m_guiThread)
        {
            m_context->moveToThread(m_guiThread);
        }

        QMutexLocker locker(&m_mutex);
        m_started = false;
        return;
    }

    MeshUpdateStatistics accumulatedStatistics;
    accumulatedStatistics.contextAcquireMilliseconds = elapsedMilliseconds(acquireStart, RenderClock::now());
    RenderState state;
    QString errorMessage;

    if (!state.initialize(m_context, errorMessage))
    {
        postFailure(errorMessage);
        state.cleanup();
        m_context->doneCurrent();

        if (m_guiThread)
        {
            m_context->moveToThread(m_guiThread);
        }

        QMutexLocker locker(&m_mutex);
        m_started = false;
        return;
    }

    bool renderRequested = true;

    while (true)
    {
        PendingBatch batch;
        bool stopRequested = false;
        bool acceptFrame = false;
        std::uint64_t acceptedFrameId = 0;

        {
            QMutexLocker locker(&m_mutex);

            while (!m_stopRequested && m_pending.isEmpty() && !m_frameAcceptancePending &&
                   !(renderRequested && !m_readyFrameOutstanding))
            {
                m_waitCondition.wait(&m_mutex);
            }

            stopRequested = m_stopRequested;

            if (!stopRequested)
            {
                batch = takePendingBatch();
                acceptFrame = m_frameAcceptancePending;
                acceptedFrameId = m_acceptedFrameId;
                m_frameAcceptancePending = false;
            }
        }

        if (stopRequested)
        {
            break;
        }

        if (acceptFrame)
        {
            state.acceptPresentedFrame(acceptedFrameId);
        }

        if (!batch.isEmpty())
        {
            state.applyBatch(batch, accumulatedStatistics);
            renderRequested = renderRequested || batch.requestFrame;
        }

        bool canPublish = false;

        {
            QMutexLocker locker(&m_mutex);
            canPublish = !m_readyFrameOutstanding;
        }

        if (renderRequested && canPublish)
        {
            std::uint64_t frameId = 0;
            unsigned int textureId = 0;
            std::uint64_t frameVersion = 0;
            QSize textureSize;

            if (state.renderFrame(accumulatedStatistics, frameId, textureId, frameVersion, textureSize))
            {
                {
                    QMutexLocker locker(&m_mutex);
                    m_readyFrameOutstanding = true;
                    m_readyFrameId = frameId;
                    m_readyFrameVersion = frameVersion;
                }

                if (m_frameReceiver)
                {
                    QCoreApplication::postEvent(m_frameReceiver,
                        new RenderFrameReadyEvent(frameId, textureId, frameVersion, textureSize, accumulatedStatistics));
                }

                accumulatedStatistics.clear();
                renderRequested = false;
            }
        }
    }

    state.cleanup();
    const RenderClock::time_point releaseStart = RenderClock::now();
    m_context->doneCurrent();
    accumulatedStatistics.contextReleaseMilliseconds += elapsedMilliseconds(releaseStart, RenderClock::now());

    if (m_guiThread)
    {
        m_context->moveToThread(m_guiThread);
    }

    QMutexLocker locker(&m_mutex);
    m_started = false;
    m_stopRequested = false;
    m_context = nullptr;
    m_surface = nullptr;
    m_frameReceiver = nullptr;
    m_guiThread = nullptr;
}

}