#include "MeshValidation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 保存一个按顶点索引排序的无向边。
struct EdgeKey
{
    EdgeKey()
        : first(0)
        , second(0)
    {
    }

    EdgeKey(std::uint32_t firstValue, std::uint32_t secondValue)
        : first(firstValue)
        , second(secondValue)
    {
    }

    bool operator<(const EdgeKey& other) const
    {
        return first < other.first ||
               (first == other.first && second < other.second);
    }

    std::uint32_t first; // 较小顶点索引。
    std::uint32_t second; // 较大顶点索引。
};

// 保存一个按顶点索引排序的无向三角形。
struct TriangleKey
{
    TriangleKey()
        : first(0)
        , second(0)
        , third(0)
    {
    }

    TriangleKey(
        std::uint32_t firstValue,
        std::uint32_t secondValue,
        std::uint32_t thirdValue)
        : first(firstValue)
        , second(secondValue)
        , third(thirdValue)
    {
    }

    bool operator<(const TriangleKey& other) const
    {
        if (first != other.first)
        {
            return first < other.first;
        }

        if (second != other.second)
        {
            return second < other.second;
        }

        return third < other.third;
    }

    std::uint32_t first; // 最小顶点索引。
    std::uint32_t second; // 中间顶点索引。
    std::uint32_t third; // 最大顶点索引。
};

// 保存一个三角形的三个原始顶点索引。
struct TriangleIndices
{
    std::uint32_t first; // 第一个顶点索引。
    std::uint32_t second; // 第二个顶点索引。
    std::uint32_t third; // 第三个顶点索引。
};

// 保存一个无向边关联的全部三角形。
struct EdgeRecord
{
    std::vector<std::size_t> triangleIndices; // 使用当前边的三角形索引。
};

// 使用并查集统计通过共享边连接的三角形分量。
class TriangleDisjointSet
{
public:
    explicit TriangleDisjointSet(std::size_t count)
        : m_parent(count)
        , m_rank(count, 0)
    {
        for (std::size_t index = 0; index < count; ++index)
        {
            m_parent[index] = index;
        }
    }

    std::size_t find(std::size_t index)
    {
        MYVOXEL_ASSERT_MESSAGE(
            index < m_parent.size(),
            "Mesh validation disjoint-set index exceeds its range.");

        if (m_parent[index] != index)
        {
            m_parent[index] = find(m_parent[index]);
        }

        return m_parent[index];
    }

    void unite(std::size_t first, std::size_t second)
    {
        std::size_t firstRoot = find(first);
        std::size_t secondRoot = find(second);

        if (firstRoot == secondRoot)
        {
            return;
        }

        if (m_rank[firstRoot] < m_rank[secondRoot])
        {
            std::swap(firstRoot, secondRoot);
        }

        m_parent[secondRoot] = firstRoot;

        if (m_rank[firstRoot] == m_rank[secondRoot])
        {
            ++m_rank[firstRoot];
        }
    }

private:
    std::vector<std::size_t> m_parent; // 每个三角形当前所属集合的父节点。
    std::vector<unsigned char> m_rank; // 并查集合并使用的近似树高。
};

using EdgeMap = std::map<EdgeKey, EdgeRecord>;
using TriangleMap = std::map<TriangleKey, std::size_t>;

// 返回两个顶点索引组成的规范无向边。
EdgeKey makeEdgeKey(std::uint32_t first, std::uint32_t second)
{
    return first <= second
               ? EdgeKey(first, second)
               : EdgeKey(second, first);
}

// 返回三个顶点索引组成的规范无向三角形。
TriangleKey makeTriangleKey(
    std::uint32_t first,
    std::uint32_t second,
    std::uint32_t third)
{
    std::uint32_t values[3] = { first, second, third };

    if (values[0] > values[1])
    {
        std::swap(values[0], values[1]);
    }

    if (values[1] > values[2])
    {
        std::swap(values[1], values[2]);
    }

    if (values[0] > values[1])
    {
        std::swap(values[0], values[1]);
    }

    return TriangleKey(values[0], values[1], values[2]);
}

// 将指定三角形的三条无向边加入边表。
void addTriangleEdges(
    const TriangleIndices& triangle,
    std::size_t triangleIndex,
    EdgeMap& edges)
{
    edges[makeEdgeKey(triangle.first, triangle.second)]
        .triangleIndices.push_back(triangleIndex);

    edges[makeEdgeKey(triangle.second, triangle.third)]
        .triangleIndices.push_back(triangleIndex);

    edges[makeEdgeKey(triangle.third, triangle.first)]
        .triangleIndices.push_back(triangleIndex);
}

// 判断指定三角形是否包含目标顶点，并返回与该顶点相连的两条边。
void appendTriangleVertexEdges(
    const TriangleIndices& triangle,
    std::uint32_t vertexIndex,
    std::vector<EdgeKey>& edges)
{
    edges.clear();

    if (triangle.first == vertexIndex)
    {
        edges.push_back(
            makeEdgeKey(triangle.first, triangle.second));

        edges.push_back(
            makeEdgeKey(triangle.third, triangle.first));

        return;
    }

    if (triangle.second == vertexIndex)
    {
        edges.push_back(
            makeEdgeKey(triangle.first, triangle.second));

        edges.push_back(
            makeEdgeKey(triangle.second, triangle.third));

        return;
    }

    MYVOXEL_ASSERT_MESSAGE(
        triangle.third == vertexIndex,
        "Mesh validation triangle does not contain the expected vertex.");

    edges.push_back(
        makeEdgeKey(triangle.second, triangle.third));

    edges.push_back(
        makeEdgeKey(triangle.third, triangle.first));
}

// 判断指定顶点的全部入射三角形是否通过包含该顶点的边形成一个连通扇区。
bool hasSingleIncidentTriangleFan(
    std::uint32_t vertexIndex,
    const std::vector<std::size_t>& incidentTriangles,
    const std::vector<TriangleIndices>& triangles,
    const EdgeMap& edges)
{
    if (incidentTriangles.size() <= 1)
    {
        return true;
    }

    std::vector<unsigned char> visited(
        incidentTriangles.size(),
        static_cast<unsigned char>(0));

    std::vector<std::size_t> stack;
    stack.reserve(incidentTriangles.size());
    stack.push_back(0);
    visited[0] = 1;

    std::size_t visitedCount = 0;
    std::vector<EdgeKey> triangleEdges;
    triangleEdges.reserve(2);

    while (!stack.empty())
    {
        const std::size_t localTrianglePosition =
            stack.back();

        stack.pop_back();
        ++visitedCount;

        const std::size_t triangleIndex =
            incidentTriangles[localTrianglePosition];

        appendTriangleVertexEdges(
            triangles[triangleIndex],
            vertexIndex,
            triangleEdges);

        for (std::size_t edgePosition = 0;
             edgePosition < triangleEdges.size();
             ++edgePosition)
        {
            const EdgeMap::const_iterator edgeIterator =
                edges.find(triangleEdges[edgePosition]);

            MYVOXEL_ASSERT_MESSAGE(
                edgeIterator != edges.end(),
                "Mesh validation could not find an incident edge.");

            const std::vector<std::size_t>& adjacentTriangles =
                edgeIterator->second.triangleIndices;

            for (std::size_t adjacentPosition = 0;
                 adjacentPosition < adjacentTriangles.size();
                 ++adjacentPosition)
            {
                const std::size_t adjacentTriangleIndex =
                    adjacentTriangles[adjacentPosition];

                const std::vector<std::size_t>::const_iterator localIterator =
                    std::lower_bound(
                        incidentTriangles.begin(),
                        incidentTriangles.end(),
                        adjacentTriangleIndex);

                MYVOXEL_ASSERT_MESSAGE(
                    localIterator != incidentTriangles.end() &&
                        *localIterator == adjacentTriangleIndex,
                    "Mesh validation vertex incident-triangle list is inconsistent.");

                const std::size_t adjacentLocalPosition =
                    static_cast<std::size_t>(
                        localIterator - incidentTriangles.begin());

                if (visited[adjacentLocalPosition] == 0)
                {
                    visited[adjacentLocalPosition] = 1;
                    stack.push_back(adjacentLocalPosition);
                }
            }
        }
    }

    return visitedCount == incidentTriangles.size();
}

}

namespace MyVoxel
{
namespace Geometry
{

MeshValidationReport::MeshValidationReport()
    : geometryValid(false)
    , vertexCount(0)
    , usedVertexCount(0)
    , unusedVertexCount(0)
    , triangleCount(0)
    , degenerateTriangleCount(0)
    , duplicateTriangleCount(0)
    , uniqueEdgeCount(0)
    , boundaryEdgeCount(0)
    , manifoldEdgeCount(0)
    , nonManifoldEdgeCount(0)
    , nonManifoldVertexCount(0)
    , connectedComponentCount(0)
{
}

/// 综合判断

bool MeshValidationReport::hasValidGeometry() const
{
    return geometryValid;
}

bool MeshValidationReport::isClosedManifold() const
{
    return geometryValid &&
           triangleCount != 0 &&
           degenerateTriangleCount == 0 &&
           duplicateTriangleCount == 0 &&
           boundaryEdgeCount == 0 &&
           nonManifoldEdgeCount == 0 &&
           nonManifoldVertexCount == 0;
}

bool MeshValidationReport::isSingleComponentClosedManifold() const
{
    return isClosedManifold() &&
           connectedComponentCount == 1;
}

/// 网格验证

MeshValidationReport validateMesh(
    const Mesh& mesh,
    double doubledAreaTolerance)
{
    MYVOXEL_ASSERT_MESSAGE(
        std::isfinite(doubledAreaTolerance) &&
            doubledAreaTolerance >= 0.0,
        "Mesh validation doubled-area tolerance must be finite and non-negative.");

    MeshValidationReport report;
    report.vertexCount = mesh.vertexCount();
    report.geometryValid = mesh.isGeometryValid();

    if (!report.geometryValid)
    {
        return report;
    }

    report.triangleCount = mesh.triangleCount();
    report.unusedVertexCount = report.vertexCount;

    if (mesh.isEmpty())
    {
        return report;
    }

    report.degenerateTriangleCount =
        mesh.degenerateTriangleCount(doubledAreaTolerance);

    const std::vector<std::uint32_t>& indices =
        mesh.indices();

    std::vector<TriangleIndices> triangles(
        report.triangleCount);

    std::vector<unsigned char> usedVertices(
        report.vertexCount,
        static_cast<unsigned char>(0));

    std::vector<std::vector<std::size_t> > vertexIncidentTriangles(
        report.vertexCount);

    EdgeMap edges;
    TriangleMap uniqueTriangles;

    for (std::size_t triangleIndex = 0;
         triangleIndex < report.triangleCount;
         ++triangleIndex)
    {
        const std::size_t indexOffset =
            triangleIndex * 3;

        TriangleIndices triangle;
        triangle.first = indices[indexOffset];
        triangle.second = indices[indexOffset + 1];
        triangle.third = indices[indexOffset + 2];

        triangles[triangleIndex] = triangle;

        const std::uint32_t triangleVertices[3] =
        {
            triangle.first,
            triangle.second,
            triangle.third
        };

        for (std::size_t vertexPosition = 0;
             vertexPosition < 3;
             ++vertexPosition)
        {
            const std::uint32_t vertexIndex =
                triangleVertices[vertexPosition];

            usedVertices[vertexIndex] = 1;
            vertexIncidentTriangles[vertexIndex]
                .push_back(triangleIndex);
        }

        const TriangleKey triangleKey =
            makeTriangleKey(
                triangle.first,
                triangle.second,
                triangle.third);

        const std::pair<TriangleMap::iterator, bool> inserted =
            uniqueTriangles.insert(
                std::make_pair(
                    triangleKey,
                    triangleIndex));

        if (!inserted.second)
        {
            ++report.duplicateTriangleCount;
        }

        addTriangleEdges(
            triangle,
            triangleIndex,
            edges);
    }

    for (std::size_t vertexIndex = 0;
         vertexIndex < usedVertices.size();
         ++vertexIndex)
    {
        if (usedVertices[vertexIndex] != 0)
        {
            ++report.usedVertexCount;
        }
    }

    report.unusedVertexCount =
        report.vertexCount - report.usedVertexCount;

    report.uniqueEdgeCount = edges.size();

    TriangleDisjointSet components(
        report.triangleCount);

    for (EdgeMap::const_iterator edgeIterator =
             edges.begin();
         edgeIterator != edges.end();
         ++edgeIterator)
    {
        const std::vector<std::size_t>& edgeTriangles =
            edgeIterator->second.triangleIndices;

        if (edgeTriangles.size() == 1)
        {
            ++report.boundaryEdgeCount;
        }
        else if (edgeTriangles.size() == 2)
        {
            ++report.manifoldEdgeCount;
        }
        else
        {
            ++report.nonManifoldEdgeCount;
        }

        for (std::size_t trianglePosition = 1;
             trianglePosition < edgeTriangles.size();
             ++trianglePosition)
        {
            components.unite(
                edgeTriangles[0],
                edgeTriangles[trianglePosition]);
        }
    }

    std::set<std::size_t> componentRoots;

    for (std::size_t triangleIndex = 0;
         triangleIndex < report.triangleCount;
         ++triangleIndex)
    {
        componentRoots.insert(
            components.find(triangleIndex));
    }

    report.connectedComponentCount =
        componentRoots.size();

    for (std::size_t vertexIndex = 0;
         vertexIndex < vertexIncidentTriangles.size();
         ++vertexIndex)
    {
        const std::vector<std::size_t>& incidentTriangles =
            vertexIncidentTriangles[vertexIndex];

        if (incidentTriangles.empty())
        {
            continue;
        }

        if (!hasSingleIncidentTriangleFan(
                static_cast<std::uint32_t>(vertexIndex),
                incidentTriangles,
                triangles,
                edges))
        {
            ++report.nonManifoldVertexCount;
        }
    }

    return report;
}

}
}