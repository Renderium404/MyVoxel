#include "DisplayMesh.h"

#include <cassert>
#include <cmath>

namespace MyVoxelViewer
{

DisplayVertex::DisplayVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue, const DisplayColor& colorValue)
    : x(static_cast<float>(xValue))
    , y(static_cast<float>(yValue))
    , z(static_cast<float>(zValue))
    , normalX(static_cast<float>(normalXValue))
    , normalY(static_cast<float>(normalYValue))
    , normalZ(static_cast<float>(normalZValue))
    , color(colorValue)
{
    assert(std::isfinite(xValue));
    assert(std::isfinite(yValue));
    assert(std::isfinite(zValue));
    assert(std::isfinite(normalXValue));
    assert(std::isfinite(normalYValue));
    assert(std::isfinite(normalZValue));
}

void DisplayMesh::clear()
{
    m_vertices.clear();
    m_indices.clear();
}

void DisplayMesh::reserve(std::size_t vertexCount, std::size_t indexCount)
{
    m_vertices.reserve(vertexCount);
    m_indices.reserve(indexCount);
}

bool DisplayMesh::isEmpty() const
{
    return m_vertices.empty() || m_indices.empty();
}

std::size_t DisplayMesh::vertexCount() const
{
    return m_vertices.size();
}

std::size_t DisplayMesh::indexCount() const
{
    return m_indices.size();
}

std::vector<DisplayVertex>& DisplayMesh::vertices()
{
    return m_vertices;
}

const std::vector<DisplayVertex>& DisplayMesh::vertices() const
{
    return m_vertices;
}

std::vector<std::uint32_t>& DisplayMesh::indices()
{
    return m_indices;
}

const std::vector<std::uint32_t>& DisplayMesh::indices() const
{
    return m_indices;
}

}