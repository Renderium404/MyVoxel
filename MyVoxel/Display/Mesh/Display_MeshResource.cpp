#include "Display_MeshResource.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace
{

const double Pi = 3.14159265358979323846; // 显示锐边角度从度转换为弧度使用的圆周率。

// 返回MeshVertex局部空间位置。
MyMath::Vector3 vertexPosition(const MyVoxel::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 返回三角形未单位化面积法线；其长度等于三角形二倍面积。
MyMath::Vector3 triangleAreaNormal(const std::vector<MyVoxel::MeshVertex>& vertices,
                                   const std::vector<std::uint32_t>& indices, std::size_t triangleIndex)
{
    const std::size_t offset = triangleIndex * 3;
    const MyMath::Vector3 point0 = vertexPosition(vertices[indices[offset]]);
    const MyMath::Vector3 point1 = vertexPosition(vertices[indices[offset + 1]]);
    const MyMath::Vector3 point2 = vertexPosition(vertices[indices[offset + 2]]);
    return MyMath::Vector3::cross(point1 - point0, point2 - point0);
}

// 返回三角形单位几何法线；退化三角形使用第一个顶点已有单位显示法线作为稳定后备。
MyMath::Vector3 triangleUnitNormal(const std::vector<MyVoxel::MeshVertex>& vertices,
                                   const std::vector<std::uint32_t>& indices, std::size_t triangleIndex,
                                   const MyMath::Vector3& areaNormal)
{
    MyMath::Vector3 normal = areaNormal;

    if (normal.normalize())
    {
        return normal;
    }

    const std::size_t offset = triangleIndex * 3;
    normal = vertices[indices[offset]].normal.vector();
    MYVOXEL_ASSERT_MESSAGE(normal.normalize(), "Renderable Mesh triangle must provide a usable fallback display normal.");
    return normal;
}

// 判断两个三角形是否通过包含指定顶点的一条完整网格边相邻。
bool shareEdgeAtVertex(const std::vector<std::uint32_t>& indices, std::size_t firstTriangle,
                       std::size_t secondTriangle, std::uint32_t sharedVertex)
{
    const std::size_t firstOffset = firstTriangle * 3;
    const std::size_t secondOffset = secondTriangle * 3;

    for (unsigned int firstCorner = 0; firstCorner < 3; ++firstCorner)
    {
        const std::uint32_t firstVertex = indices[firstOffset + firstCorner];

        if (firstVertex == sharedVertex)
        {
            continue;
        }

        for (unsigned int secondCorner = 0; secondCorner < 3; ++secondCorner)
        {
            const std::uint32_t secondVertex = indices[secondOffset + secondCorner];

            if (secondVertex != sharedVertex && firstVertex == secondVertex)
            {
                return true;
            }
        }
    }

    return false;
}

// 返回当前三角形角点所属平滑组的显示法线；未形成锐边分组时严格保留源Mesh法线。
MyVoxel::Display_Normal creaseCornerNormal(
    const std::vector<MyVoxel::MeshVertex>& vertices,
    const std::vector<std::uint32_t>& indices,
    const std::vector<std::vector<std::size_t> >& incidentFaces,
    const std::vector<MyMath::Vector3>& areaNormals,
    const std::vector<MyMath::Vector3>& unitNormals,
    std::size_t triangleIndex,
    std::uint32_t vertexIndex,
    double minimumSmoothDot)
{
    const std::vector<std::size_t>& faces = incidentFaces[vertexIndex];

    if (faces.size() <= 1)
    {
        return vertices[vertexIndex].normal;
    }

    std::size_t seedPosition = faces.size();

    for (std::size_t position = 0; position < faces.size(); ++position)
    {
        if (faces[position] == triangleIndex)
        {
            seedPosition = position;
            break;
        }
    }

    MYVOXEL_ASSERT_MESSAGE(seedPosition < faces.size(), "Triangle corner must belong to its source vertex incident-face list.");

    if (seedPosition >= faces.size())
    {
        return vertices[vertexIndex].normal;
    }

    std::vector<unsigned char> visited(faces.size(), 0);
    std::vector<std::size_t> queue;
    queue.reserve(faces.size());
    visited[seedPosition] = 1;
    queue.push_back(seedPosition);

    for (std::size_t queueIndex = 0; queueIndex < queue.size(); ++queueIndex)
    {
        const std::size_t currentPosition = queue[queueIndex];
        const std::size_t currentFace = faces[currentPosition];

        for (std::size_t candidatePosition = 0; candidatePosition < faces.size(); ++candidatePosition)
        {
            if (visited[candidatePosition])
            {
                continue;
            }

            const std::size_t candidateFace = faces[candidatePosition];

            if (!shareEdgeAtVertex(indices, currentFace, candidateFace, vertexIndex))
            {
                continue;
            }

            if (MyMath::Vector3::dot(unitNormals[currentFace], unitNormals[candidateFace]) < minimumSmoothDot)
            {
                continue;
            }

            visited[candidatePosition] = 1;
            queue.push_back(candidatePosition);
        }
    }

    // 所有关联面仍属于同一个连续平滑组时保留原TSDF/几何提供的高质量顶点法线。
    if (queue.size() == faces.size())
    {
        return vertices[vertexIndex].normal;
    }

    // 发生锐边分组时仅平均当前连通平滑组的面积法线，得到锐边一侧独立法线。
    MyMath::Vector3 normal = MyMath::Vector3::zero();

    for (std::size_t groupIndex = 0; groupIndex < queue.size(); ++groupIndex)
    {
        normal += areaNormals[faces[queue[groupIndex]]];
    }

    if (!normal.normalize())
    {
        normal = unitNormals[triangleIndex];
    }

    return MyVoxel::Display_Normal(normal);
}

// 建立每个源Mesh顶点关联的全部三角形列表。
void buildIncidentFaces(const MyVoxel::Mesh& mesh, std::vector<std::vector<std::size_t> >& incidentFaces)
{
    incidentFaces.clear();
    incidentFaces.resize(mesh.vertexCount());
    const std::vector<std::uint32_t>& indices = mesh.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        const std::size_t offset = triangleIndex * 3;

        for (unsigned int cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
        {
            incidentFaces[indices[offset + cornerIndex]].push_back(triangleIndex);
        }
    }
}

// 预计算全部三角形面积法线和单位几何法线。
void buildFaceNormals(const MyVoxel::Mesh& mesh, std::vector<MyMath::Vector3>& areaNormals,
                      std::vector<MyMath::Vector3>& unitNormals)
{
    areaNormals.resize(mesh.triangleCount());
    unitNormals.resize(mesh.triangleCount());
    const std::vector<MyVoxel::MeshVertex>& vertices = mesh.vertices();
    const std::vector<std::uint32_t>& indices = mesh.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        areaNormals[triangleIndex] = triangleAreaNormal(vertices, indices, triangleIndex);
        unitNormals[triangleIndex] = triangleUnitNormal(vertices, indices, triangleIndex, areaNormals[triangleIndex]);
    }
}

}

namespace MyVoxel
{

Display_MeshResourceOptions::Display_MeshResourceOptions()
    : normalMode(Display_MeshNormalMode::SourceNormals)
    , creaseAngleDegrees(30.0) // 30度作为CAD/机械显示硬边的默认起始阈值，后续可根据视觉效果调整。
{
}

Display_MeshResource::Display_MeshResource(const Mesh& mesh)
    : m_triangleCount(0)
    , m_valid(false)
{
    build(mesh, Display_MeshResourceOptions());
}

Display_MeshResource::Display_MeshResource(const Mesh& mesh, const Display_MeshResourceOptions& options)
    : m_triangleCount(0)
    , m_valid(false)
{
    build(mesh, options);
}

void Display_MeshResource::build(const Mesh& mesh, const Display_MeshResourceOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable(), "Display mesh resource requires a renderable Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!mesh.isEmpty(), "Display mesh resource requires a non-empty Mesh.");
    MYVOXEL_ASSERT_MESSAGE(options.normalMode == Display_MeshNormalMode::SourceNormals ||
                           options.normalMode == Display_MeshNormalMode::CreaseAware,
                           "Display mesh resource encountered an invalid normal mode.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.creaseAngleDegrees) &&
                           options.creaseAngleDegrees > 0.0 && options.creaseAngleDegrees < 180.0,
                           "Display mesh crease angle must lie strictly inside (0,180) degrees.");

    if (!mesh.isRenderable() || mesh.isEmpty())
    {
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(mesh.indexCount() <= m_vertices.max_size(),
                           "Display mesh vertex count exceeds vector capacity.");

    m_vertices.clear();
    m_vertices.reserve(mesh.indexCount());
    const std::vector<MeshVertex>& sourceVertices = mesh.vertices();
    const std::vector<std::uint32_t>& sourceIndices = mesh.indices();
    std::vector<std::vector<std::size_t> > incidentFaces;
    std::vector<MyMath::Vector3> areaNormals;
    std::vector<MyMath::Vector3> unitNormals;
    const bool creaseAware = options.normalMode == Display_MeshNormalMode::CreaseAware;
    const double minimumSmoothDot = std::cos(options.creaseAngleDegrees * Pi / 180.0);

    if (creaseAware)
    {
        buildIncidentFaces(mesh, incidentFaces);
        buildFaceNormals(mesh, areaNormals, unitNormals);
    }

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        const Display_Color& color = mesh.triangleColor(triangleIndex);
        const std::size_t firstIndexPosition = triangleIndex * 3;

        for (unsigned int cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
        {
            const std::uint32_t sourceVertexIndex = sourceIndices[firstIndexPosition + cornerIndex];
            const MeshVertex& sourceVertex = sourceVertices[sourceVertexIndex];
            const Display_Normal normal = creaseAware
                ? creaseCornerNormal(sourceVertices, sourceIndices, incidentFaces, areaNormals, unitNormals,
                                     triangleIndex, sourceVertexIndex, minimumSmoothDot)
                : sourceVertex.normal;

            m_vertices.push_back(Display_MeshVertex(sourceVertex.x, sourceVertex.y, sourceVertex.z, normal, color));
        }
    }

    m_localBounds = mesh.localBounds();
    m_triangleCount = mesh.triangleCount();
    m_valid = !m_vertices.empty() && m_vertices.size() == m_triangleCount * 3 && m_localBounds.isValid();
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Display mesh resource construction produced inconsistent data.");
}

/// 资源属性

Display_ResourceKind Display_MeshResource::kind() const
{
    return Display_ResourceKind::Mesh;
}

bool Display_MeshResource::isValid() const
{
    return m_valid;
}

const Bounds3& Display_MeshResource::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the bounds of an invalid display mesh resource.");
    return m_localBounds;
}

std::size_t Display_MeshResource::memoryByteSize() const
{
    return m_vertices.size() * sizeof(Display_MeshVertex);
}

/// 网格数据

const std::vector<Display_MeshVertex>& Display_MeshResource::vertices() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid display mesh resource.");
    return m_vertices;
}

std::size_t Display_MeshResource::vertexCount() const
{
    return m_vertices.size();
}

std::size_t Display_MeshResource::triangleCount() const
{
    return m_triangleCount;
}

}