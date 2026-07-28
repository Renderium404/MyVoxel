#include "VoxelSurfaceMesh.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace MyVoxel
{

VoxelSurfaceColor::VoxelSurfaceColor()
{
}

VoxelSurfaceColor::VoxelSurfaceColor(double redValue, double greenValue, double blueValue, double alphaValue)
    : red(static_cast<float>(redValue))
    , green(static_cast<float>(greenValue))
    , blue(static_cast<float>(blueValue))
    , alpha(static_cast<float>(alphaValue))
{
    assert(std::isfinite(redValue));
    assert(std::isfinite(greenValue));
    assert(std::isfinite(blueValue));
    assert(std::isfinite(alphaValue));
}

VoxelSurfaceVertex::VoxelSurfaceVertex()
{
}

VoxelSurfaceVertex::VoxelSurfaceVertex(double xValue, double yValue, double zValue, double nxValue, double nyValue, double nzValue, const VoxelSurfaceColor& colorValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , nx(static_cast<float>(nxValue))
    , ny(static_cast<float>(nyValue))
    , nz(static_cast<float>(nzValue))
    , color(colorValue)
{
    assert(std::isfinite(xValue));
    assert(std::isfinite(yValue));
    assert(std::isfinite(zValue));
    assert(std::isfinite(nxValue));
    assert(std::isfinite(nyValue));
    assert(std::isfinite(nzValue));
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
    assert(this != &mesh);

    if (mesh.isEmpty())
    {
        return;
    }

    const std::size_t maximumVertexCount = static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());
    const std::size_t vertexOffset = m_vertices.size();

    assert(vertexOffset <= maximumVertexCount);
    assert(mesh.vertexCount() <= maximumVertexCount - vertexOffset);

    m_vertices.reserve(m_vertices.size() + mesh.vertexCount());
    m_indices.reserve(m_indices.size() + mesh.indexCount());
    m_vertices.insert(m_vertices.end(), mesh.m_vertices.begin(), mesh.m_vertices.end());

    for (std::size_t i = 0; i < mesh.m_indices.size(); ++i)
    {
        assert(static_cast<std::size_t>(mesh.m_indices[i]) < mesh.m_vertices.size());

        const std::size_t targetIndex = vertexOffset + static_cast<std::size_t>(mesh.m_indices[i]);

        assert(targetIndex < maximumVertexCount);
        m_indices.push_back(static_cast<std::uint32_t>(targetIndex));
    }
}

std::uint32_t VoxelSurfaceMesh::appendVertex(const VoxelSurfaceVertex& vertex)
{
    assert(m_vertices.size() < static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)()));

    const std::uint32_t index = static_cast<std::uint32_t>(m_vertices.size());
    m_vertices.push_back(vertex);
    return index;
}

void VoxelSurfaceMesh::appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
{
    assert(static_cast<std::size_t>(index0) < m_vertices.size());
    assert(static_cast<std::size_t>(index1) < m_vertices.size());
    assert(static_cast<std::size_t>(index2) < m_vertices.size());

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