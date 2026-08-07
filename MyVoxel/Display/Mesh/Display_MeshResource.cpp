#include "Display_MeshResource.h"

#include <cstdint>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace MyVoxel
{

Display_MeshResource::Display_MeshResource(const Mesh& mesh)
    : m_triangleCount(0)
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable(), "Display mesh resource requires a renderable Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!mesh.isEmpty(), "Display mesh resource requires a non-empty Mesh.");

    if (!mesh.isRenderable() || mesh.isEmpty())
    {
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(mesh.indexCount() <= m_vertices.max_size(),
                           "Display mesh vertex count exceeds vector capacity.");

    m_vertices.reserve(mesh.indexCount());
    const std::vector<MeshVertex>& sourceVertices = mesh.vertices();
    const std::vector<std::uint32_t>& sourceIndices = mesh.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        const Display_Color& color = mesh.triangleColor(triangleIndex);
        const std::size_t firstIndexPosition = triangleIndex * 3;

        for (unsigned int cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
        {
            const MeshVertex& sourceVertex = sourceVertices[sourceIndices[firstIndexPosition + cornerIndex]];
            m_vertices.push_back(Display_MeshVertex(sourceVertex.x, sourceVertex.y, sourceVertex.z,
                                                    sourceVertex.normal, color));
        }
    }

    m_localBounds = mesh.localBounds();
    m_triangleCount = mesh.triangleCount();
    m_valid = !m_vertices.empty() && m_vertices.size() == m_triangleCount * 3 && m_localBounds.isValid();
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Display mesh resource construction produced inconsistent data.");
}

/// 状态判断

bool Display_MeshResource::isValid() const
{
    return m_valid;
}

/// 资源数据

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

const Bounds3& Display_MeshResource::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the bounds of an invalid display mesh resource.");
    return m_localBounds;
}

std::size_t Display_MeshResource::memoryByteSize() const
{
    return m_vertices.size() * sizeof(Display_MeshVertex);
}

}
