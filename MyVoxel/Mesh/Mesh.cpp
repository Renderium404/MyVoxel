#include "Mesh.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double ValidNormalTolerance = 1.0e-12; // 显示法线能够作为有效方向参与计算的最小长度阈值。
const double UnitNormalTolerance = 1.0e-5; // Mesh能够直接转换为显示资源时使用的单位法线误差。

// 判断float存储值是否为有限值。
bool isFiniteFloat(float value)
{
    return std::isfinite(static_cast<double>(value));
}

// 判断顶点位置是否为有限值。
bool isFiniteVertexPosition(const MyVoxel::MeshVertex& vertex)
{
    return isFiniteFloat(vertex.x) && isFiniteFloat(vertex.y) && isFiniteFloat(vertex.z);
}

// 判断顶点位置和可选显示法线是否为有限值。
bool isFiniteVertex(const MyVoxel::MeshVertex& vertex)
{
    return isFiniteVertexPosition(vertex) && vertex.normal.isFinite();
}

// 返回网格顶点的位置。
MyMath::Vector3 vertexPosition(const MyVoxel::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 创建经过点变换和可选法线逆转置变换后的顶点。
MyVoxel::MeshVertex transformedVertex(const MyVoxel::MeshVertex& vertex, const MyMath::Matrix4& pointTransform,
                                      const MyMath::Matrix4& normalTransform)
{
    const MyMath::Vector3 point = pointTransform.transformPoint(vertexPosition(vertex));
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Mesh point transform produced a non-finite position.");

    if (vertex.normal.isZero())
    {
        return MyVoxel::MeshVertex(point.x(), point.y(), point.z());
    }

    const MyMath::Vector3 normal = normalTransform.transformVector(vertex.normal.vector()).normalized(ValidNormalTolerance);
    MYVOXEL_ASSERT_MESSAGE(normal.isVector(ValidNormalTolerance), "Mesh normal transform produced an invalid direction.");
    return MyVoxel::MeshVertex(point.x(), point.y(), point.z(), MyVoxel::Display_Normal(normal));
}

}

namespace MyVoxel
{

MeshVertex::MeshVertex()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
{
}

MeshVertex::MeshVertex(double xValue, double yValue, double zValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(*this), "Mesh vertex position must be finite and representable by float.");
}

MeshVertex::MeshVertex(double xValue, double yValue, double zValue, const Display_Normal& normalValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , normal(normalValue)
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(*this), "Mesh vertex position and display normal must be finite and representable by float.");
}

MeshVertex::MeshVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , normal(normalXValue, normalYValue, normalZValue)
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(*this), "Mesh vertex position and display normal must be finite and representable by float.");
}

/// 网格状态

void Mesh::clear()
{
    m_vertices.clear();
    m_indices.clear();
    m_triangleColors.clear();
}

void Mesh::reserve(std::size_t vertexCountValue, std::size_t indexCountValue)
{
    MYVOXEL_ASSERT_MESSAGE(indexCountValue % 3 == 0, "Reserved mesh index count must be divisible by three.");
    m_vertices.reserve(vertexCountValue);
    m_indices.reserve(indexCountValue);
    m_triangleColors.reserve(indexCountValue / 3);
}

void Mesh::resizeVertices(std::size_t vertexCountValue)
{
    const std::size_t maximumVertexCount = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());
    MYVOXEL_ASSERT_MESSAGE(m_indices.empty() && m_triangleColors.empty(),
                           "Mesh vertex array can only be resized before triangle construction.");
    MYVOXEL_ASSERT_MESSAGE(vertexCountValue <= maximumVertexCount, "Mesh vertex count exceeds the 32-bit index range.");
    m_vertices.resize(vertexCountValue);
}

bool Mesh::isEmpty() const
{
    return m_indices.empty();
}

bool Mesh::isGeometryValid() const
{
    if (m_indices.size() % 3 != 0)
    {
        return false;
    }

    if (m_indices.empty())
    {
        return m_vertices.empty();
    }

    if (m_vertices.empty())
    {
        return false;
    }

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        if (!isFiniteVertexPosition(m_vertices[vertexIndex]))
        {
            return false;
        }
    }

    for (std::size_t indexPosition = 0; indexPosition < m_indices.size(); ++indexPosition)
    {
        if (static_cast<std::size_t>(m_indices[indexPosition]) >= m_vertices.size())
        {
            return false;
        }
    }

    return true;
}

bool Mesh::isValid() const
{
    if (!isGeometryValid())
    {
        return false;
    }

    if (isEmpty())
    {
        return m_triangleColors.empty();
    }

    bool hasNoNormals = true;
    bool hasCompleteNormals = true;

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        const Display_Normal& normal = m_vertices[vertexIndex].normal;
        hasNoNormals = hasNoNormals && normal.isZero();
        hasCompleteNormals = hasCompleteNormals && normal.isDirection(ValidNormalTolerance);
    }

    return (hasNoNormals || hasCompleteNormals) && (m_triangleColors.empty() || hasTriangleColors());
}

bool Mesh::isRenderable() const
{
    return isValid() && !isEmpty() && hasUnitNormals(UnitNormalTolerance) && hasTriangleColors();
}

bool Mesh::hasValidNormals(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(epsilon) && epsilon >= 0.0,
                           "Mesh normal validation epsilon must be finite and non-negative.");

    if (!isGeometryValid())
    {
        return false;
    }

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        if (!m_vertices[vertexIndex].normal.isDirection(epsilon))
        {
            return false;
        }
    }

    return true;
}

bool Mesh::hasUnitNormals(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(epsilon) && epsilon >= 0.0,
                           "Mesh normal comparison epsilon must be finite and non-negative.");

    if (!hasValidNormals(ValidNormalTolerance))
    {
        return false;
    }

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        if (!m_vertices[vertexIndex].normal.isUnit(epsilon))
        {
            return false;
        }
    }

    return true;
}

bool Mesh::hasTriangleColors() const
{
    if (!isGeometryValid() || isEmpty() || m_triangleColors.size() != triangleCount())
    {
        return false;
    }

    for (std::size_t triangleIndex = 0; triangleIndex < m_triangleColors.size(); ++triangleIndex)
    {
        if (!m_triangleColors[triangleIndex].isValid())
        {
            return false;
        }
    }

    return true;
}

bool Mesh::hasDegenerateTriangles(double doubledAreaTolerance) const
{
    return degenerateTriangleCount(doubledAreaTolerance) != 0;
}

std::size_t Mesh::degenerateTriangleCount(double doubledAreaTolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isGeometryValid(), "Cannot inspect geometric degeneracy of an invalid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(doubledAreaTolerance) && doubledAreaTolerance >= 0.0,
                           "Mesh doubled-area tolerance must be finite and non-negative.");

    const double squaredThreshold = doubledAreaTolerance * doubledAreaTolerance;
    std::size_t count = 0;

    for (std::size_t triangleIndex = 0; triangleIndex < triangleCount(); ++triangleIndex)
    {
        const std::size_t indexOffset = triangleIndex * 3;
        const MyMath::Vector3 point0 = vertexPosition(m_vertices[m_indices[indexOffset]]);
        const MyMath::Vector3 point1 = vertexPosition(m_vertices[m_indices[indexOffset + 1]]);
        const MyMath::Vector3 point2 = vertexPosition(m_vertices[m_indices[indexOffset + 2]]);
        const MyMath::Vector3 doubledAreaVector = MyMath::Vector3::cross(point1 - point0, point2 - point0);

        if (!doubledAreaVector.isFinite() || doubledAreaVector.lengthSquared() <= squaredThreshold)
        {
            ++count;
        }
    }

    return count;
}

/// 网格属性

std::size_t Mesh::vertexCount() const
{
    return m_vertices.size();
}

std::size_t Mesh::indexCount() const
{
    return m_indices.size();
}

std::size_t Mesh::triangleCount() const
{
    MYVOXEL_ASSERT_MESSAGE(m_indices.size() % 3 == 0, "Mesh index count must be divisible by three.");
    return m_indices.size() / 3;
}

std::size_t Mesh::memoryByteSize() const
{
    return m_vertices.size() * sizeof(MeshVertex) + m_indices.size() * sizeof(std::uint32_t) +
           m_triangleColors.size() * sizeof(Display_Color);
}

Bounds3 Mesh::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isGeometryValid(), "Cannot calculate local bounds of a geometrically invalid Mesh.");

    if (isEmpty())
    {
        return Bounds3();
    }

    const MeshVertex& firstVertex = m_vertices[m_indices[0]];
    double minimumX = firstVertex.x;
    double minimumY = firstVertex.y;
    double minimumZ = firstVertex.z;
    double maximumX = firstVertex.x;
    double maximumY = firstVertex.y;
    double maximumZ = firstVertex.z;

    for (std::size_t indexPosition = 1; indexPosition < m_indices.size(); ++indexPosition)
    {
        const MeshVertex& vertex = m_vertices[m_indices[indexPosition]];
        minimumX = (std::min)(minimumX, static_cast<double>(vertex.x));
        minimumY = (std::min)(minimumY, static_cast<double>(vertex.y));
        minimumZ = (std::min)(minimumZ, static_cast<double>(vertex.z));
        maximumX = (std::max)(maximumX, static_cast<double>(vertex.x));
        maximumY = (std::max)(maximumY, static_cast<double>(vertex.y));
        maximumZ = (std::max)(maximumZ, static_cast<double>(vertex.z));
    }

    return Bounds3(MyMath::Vector3(minimumX, minimumY, minimumZ), MyMath::Vector3(maximumX, maximumY, maximumZ));
}

/// 网格数据

const std::vector<MeshVertex>& Mesh::vertices() const
{
    return m_vertices;
}

const std::vector<std::uint32_t>& Mesh::indices() const
{
    return m_indices;
}

const std::vector<Display_Color>& Mesh::triangleColors() const
{
    return m_triangleColors;
}

const Display_Color& Mesh::triangleColor(std::size_t triangleIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(hasTriangleColors(), "Cannot access triangle color from a Mesh without complete colors.");
    MYVOXEL_ASSERT_MESSAGE(triangleIndex < m_triangleColors.size(), "Mesh triangle color index exceeds the triangle array.");
    return m_triangleColors[triangleIndex];
}

/// 网格变换

Mesh Mesh::transformed(const MyMath::Matrix4& transform) const
{
    Mesh result = *this;
    result.transformInPlace(transform);
    return result;
}

void Mesh::transformInPlace(const MyMath::Matrix4& transform)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot transform an invalid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(transform.isAffine(), "Mesh transform must be affine.");

    if (isEmpty())
    {
        return;
    }

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(m_vertices[vertexIndex]),
                               "Mesh transform requires finite optional display attributes.");
        MYVOXEL_ASSERT_MESSAGE(m_vertices[vertexIndex].normal.isZero() ||
                                   m_vertices[vertexIndex].normal.isDirection(ValidNormalTolerance),
                               "Mesh transform requires every normal to be zero or a valid direction.");
    }

    MyMath::Matrix4 inverseTransform;
    const bool inverted = transform.inverted(inverseTransform);
    MYVOXEL_ASSERT_MESSAGE(inverted, "Mesh transform must be invertible.");

    if (!transform.isAffine() || !inverted)
    {
        return;
    }

    const MyMath::Matrix4 normalTransform = inverseTransform.transposed();

    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        m_vertices[vertexIndex] = transformedVertex(m_vertices[vertexIndex], transform, normalTransform);
    }

    if (transform.determinant() < 0.0)
    {
        for (std::size_t triangleIndex = 0; triangleIndex < triangleCount(); ++triangleIndex)
        {
            const std::size_t indexOffset = triangleIndex * 3;
            std::swap(m_indices[indexOffset + 1], m_indices[indexOffset + 2]);
        }
    }

    MYVOXEL_ASSERT_MESSAGE(isValid(), "Mesh transform produced inconsistent mesh data.");
}

/// 网格构建

void Mesh::append(const Mesh& mesh)
{
    MYVOXEL_ASSERT_MESSAGE(this != &mesh, "Mesh cannot append itself.");
    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "Cannot append an invalid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot append to an invalid Mesh.");

    if (mesh.isEmpty())
    {
        return;
    }

    if (!isEmpty())
    {
        MYVOXEL_ASSERT_MESSAGE(hasValidNormals(ValidNormalTolerance) == mesh.hasValidNormals(ValidNormalTolerance),
                               "Meshes with triangles must use the same display-normal layout before append.");
        MYVOXEL_ASSERT_MESSAGE(m_triangleColors.empty() == mesh.m_triangleColors.empty(),
                               "Meshes with triangles must use the same triangle-color layout before append.");
    }

    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());
    const std::size_t vertexOffset = m_vertices.size();
    MYVOXEL_ASSERT_MESSAGE(vertexOffset <= maximumIndex, "Mesh vertex offset exceeds the 32-bit index range.");
    MYVOXEL_ASSERT_MESSAGE(mesh.vertexCount() <= maximumIndex - vertexOffset,
                           "Combined mesh vertex count exceeds the 32-bit index range.");

    m_vertices.reserve(m_vertices.size() + mesh.vertexCount());
    m_indices.reserve(m_indices.size() + mesh.indexCount());

    if (!mesh.m_triangleColors.empty())
    {
        m_triangleColors.reserve(m_triangleColors.size() + mesh.triangleCount());
    }

    m_vertices.insert(m_vertices.end(), mesh.m_vertices.begin(), mesh.m_vertices.end());

    for (std::size_t indexPosition = 0; indexPosition < mesh.m_indices.size(); ++indexPosition)
    {
        const std::size_t targetIndex = vertexOffset + static_cast<std::size_t>(mesh.m_indices[indexPosition]);
        MYVOXEL_ASSERT_MESSAGE(targetIndex <= maximumIndex, "Combined mesh index exceeds the 32-bit index range.");
        m_indices.push_back(static_cast<std::uint32_t>(targetIndex));
    }

    if (!mesh.m_triangleColors.empty())
    {
        m_triangleColors.insert(m_triangleColors.end(), mesh.m_triangleColors.begin(), mesh.m_triangleColors.end());
    }

    MYVOXEL_ASSERT_MESSAGE(isValid(), "Mesh append produced inconsistent mesh data.");
}

void Mesh::appendTransformed(const Mesh& mesh, const MyMath::Matrix4& transform)
{
    MYVOXEL_ASSERT_MESSAGE(this != &mesh, "Mesh cannot append a transformed copy of itself directly.");
    append(mesh.transformed(transform));
}

std::uint32_t Mesh::appendVertex(const MeshVertex& vertex)
{
    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());
    MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(vertex), "Cannot append a MeshVertex containing non-finite data.");
    MYVOXEL_ASSERT_MESSAGE(m_vertices.size() < maximumIndex, "Mesh vertex count exceeds the 32-bit index range.");
    const std::uint32_t index = static_cast<std::uint32_t>(m_vertices.size());
    m_vertices.push_back(vertex);
    return index;
}

void Mesh::setVertex(std::uint32_t vertexIndex, const MeshVertex& vertex)
{
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(vertexIndex) < m_vertices.size(),
                           "Mesh target vertex index exceeds the vertex array.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteVertex(vertex), "Cannot set a MeshVertex containing non-finite data.");
    m_vertices[vertexIndex] = vertex;
}

void Mesh::appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
{
    MYVOXEL_ASSERT_MESSAGE(m_triangleColors.empty(), "Cannot append a colorless triangle after colored triangles.");
    appendTriangleIndices(index0, index1, index2);
}

void Mesh::appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2, const Display_Color& color)
{
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Cannot append a Mesh triangle with an invalid display color.");
    MYVOXEL_ASSERT_MESSAGE(m_indices.empty() || hasTriangleColors(),
                           "Cannot append a colored triangle after colorless triangles.");
    appendTriangleIndices(index0, index1, index2);
    m_triangleColors.push_back(color);
}

void Mesh::appendTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);
    appendTriangle(index0, index1, index2);
}

void Mesh::appendTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2,
                          const Display_Color& color)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);
    appendTriangle(index0, index1, index2, color);
}

void Mesh::appendQuad(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2,
                      const MeshVertex& vertex3)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);
    const std::uint32_t index3 = appendVertex(vertex3);
    appendTriangle(index0, index1, index2);
    appendTriangle(index0, index2, index3);
}

void Mesh::appendQuad(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2,
                      const MeshVertex& vertex3, const Display_Color& color)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);
    const std::uint32_t index3 = appendVertex(vertex3);
    appendTriangle(index0, index1, index2, color);
    appendTriangle(index0, index2, index3, color);
}

/// 内部辅助

void Mesh::appendTriangleIndices(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
{
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index0) < m_vertices.size(),
                           "Mesh triangle index0 exceeds the vertex array.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index1) < m_vertices.size(),
                           "Mesh triangle index1 exceeds the vertex array.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index2) < m_vertices.size(),
                           "Mesh triangle index2 exceeds the vertex array.");
    m_indices.push_back(index0);
    m_indices.push_back(index1);
    m_indices.push_back(index2);
}

}
