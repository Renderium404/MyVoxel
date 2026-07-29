#include "VoxelSurfaceMesh.h"

#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelSurfaceColor::VoxelSurfaceColor()
    : red(1.0f)
    , green(1.0f)
    , blue(1.0f)
    , alpha(1.0f)
{
}

VoxelSurfaceColor::VoxelSurfaceColor(double redValue, double greenValue, double blueValue, double alphaValue)
    : red(static_cast<float>(redValue))
    , green(static_cast<float>(greenValue))
    , blue(static_cast<float>(blueValue))
    , alpha(static_cast<float>(alphaValue))
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(redValue), "Voxel surface red component must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(greenValue), "Voxel surface green component must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(blueValue), "Voxel surface blue component must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(alphaValue), "Voxel surface alpha component must be finite.");
}

VoxelSurfaceVertex::VoxelSurfaceVertex()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , normalX(0.0f)
    , normalY(0.0f)
    , normalZ(1.0f)
{
}

VoxelSurfaceVertex::VoxelSurfaceVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue, const VoxelSurfaceColor& colorValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , normalX(static_cast<float>(normalXValue))
    , normalY(static_cast<float>(normalYValue))
    , normalZ(static_cast<float>(normalZValue))
    , color(colorValue)
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(xValue), "Voxel surface vertex X coordinate must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(yValue), "Voxel surface vertex Y coordinate must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(zValue), "Voxel surface vertex Z coordinate must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(normalXValue), "Voxel surface normal X component must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(normalYValue), "Voxel surface normal Y component must be finite.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(normalZValue), "Voxel surface normal Z component must be finite.");
}

void VoxelSurfaceMesh::clear()
{
    m_vertices.clear();
    m_indices.clear();
}

void VoxelSurfaceMesh::reserve(std::size_t vertexCountValue, std::size_t indexCountValue)
{
    m_vertices.reserve(vertexCountValue);
    m_indices.reserve(indexCountValue);
}

bool VoxelSurfaceMesh::isEmpty() const
{
    return m_vertices.empty() || m_indices.empty();
}

std::size_t VoxelSurfaceMesh::vertexCount() const
{
    return m_vertices.size();
}

std::size_t VoxelSurfaceMesh::indexCount() const
{
    return m_indices.size();
}

std::size_t VoxelSurfaceMesh::triangleCount() const
{
    return m_indices.size() / 3;
}

const std::vector<VoxelSurfaceVertex>& VoxelSurfaceMesh::vertices() const
{
    return m_vertices;
}

const std::vector<std::uint32_t>& VoxelSurfaceMesh::indices() const
{
    return m_indices;
}

void VoxelSurfaceMesh::append(const VoxelSurfaceMesh& mesh)
{
    MYVOXEL_ASSERT_MESSAGE(this != &mesh, "VoxelSurfaceMesh cannot append itself.");

    if (mesh.isEmpty())
    {
        return;
    }

    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());
    const std::size_t vertexOffset = m_vertices.size();

    MYVOXEL_ASSERT_MESSAGE(vertexOffset <= maximumIndex, "Voxel surface vertex offset exceeds the 32-bit index range.");
    MYVOXEL_ASSERT_MESSAGE(mesh.vertexCount() <= maximumIndex - vertexOffset, "Combined voxel surface vertex count exceeds the 32-bit index range.");

    m_vertices.reserve(m_vertices.size() + mesh.vertexCount());
    m_indices.reserve(m_indices.size() + mesh.indexCount());
    m_vertices.insert(m_vertices.end(), mesh.m_vertices.begin(), mesh.m_vertices.end());

    for (std::size_t indexPosition = 0; indexPosition < mesh.m_indices.size(); ++indexPosition)
    {
        const std::uint32_t sourceIndex = mesh.m_indices[indexPosition];

        MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(sourceIndex) < mesh.m_vertices.size(), "Source voxel surface index exceeds its vertex array.");

        const std::size_t targetIndex = vertexOffset + static_cast<std::size_t>(sourceIndex);

        MYVOXEL_ASSERT_MESSAGE(targetIndex <= maximumIndex, "Combined voxel surface index exceeds the 32-bit index range.");
        m_indices.push_back(static_cast<std::uint32_t>(targetIndex));
    }
}

std::uint32_t VoxelSurfaceMesh::appendVertex(const VoxelSurfaceVertex& vertex)
{
    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());

    MYVOXEL_ASSERT_MESSAGE(m_vertices.size() < maximumIndex, "Voxel surface vertex count exceeds the 32-bit index range.");

    const std::uint32_t index = static_cast<std::uint32_t>(m_vertices.size());

    m_vertices.push_back(vertex);
    return index;
}

void VoxelSurfaceMesh::appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
{
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index0) < m_vertices.size(), "Voxel surface triangle index0 exceeds the vertex array.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index1) < m_vertices.size(), "Voxel surface triangle index1 exceeds the vertex array.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(index2) < m_vertices.size(), "Voxel surface triangle index2 exceeds the vertex array.");

    m_indices.push_back(index0);
    m_indices.push_back(index1);
    m_indices.push_back(index2);
}

void VoxelSurfaceMesh::appendTriangle(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);

    appendTriangle(index0, index1, index2);
}

void VoxelSurfaceMesh::appendQuad(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2, const VoxelSurfaceVertex& vertex3)
{
    const std::uint32_t index0 = appendVertex(vertex0);
    const std::uint32_t index1 = appendVertex(vertex1);
    const std::uint32_t index2 = appendVertex(vertex2);
    const std::uint32_t index3 = appendVertex(vertex3);

    appendTriangle(index0, index1, index2);
    appendTriangle(index0, index2, index3);
}

}