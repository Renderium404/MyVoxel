#include "VoxelSmoothSurfaceExtractor.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"

namespace
{

const double MinimumNormalLengthSquared = 1.0e-20; // 小于该平方长度的法线视为无有效方向。
const double MinimumTriangleAreaSquared = 1.0e-24; // 小于该平方面积的三角形视为退化三角形。
const double MinimumInterpolationDifference = 1.0e-12; // 小于该差值时使用边中点作为等值面交点。

// 表示最高层级体素中心采样网格中的整数点。
struct GridPoint
{
    GridPoint()
        : x(0)
        , y(0)
        , z(0)
    {
    }

    GridPoint(std::int64_t xValue, std::int64_t yValue, std::int64_t zValue)
        : x(xValue)
        , y(yValue)
        , z(zValue)
    {
    }

    bool operator==(const GridPoint& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator<(const GridPoint& other) const
    {
        if (x != other.x)
        {
            return x < other.x;
        }

        if (y != other.y)
        {
            return y < other.y;
        }

        return z < other.z;
    }

    std::int64_t x; // 最高层级网格X坐标。
    std::int64_t y; // 最高层级网格Y坐标。
    std::int64_t z; // 最高层级网格Z坐标。
};

// 表示两个采样点之间的无方向网格边。
struct GridEdgeKey
{
    GridEdgeKey()
    {
    }

    GridEdgeKey(const GridPoint& point0, const GridPoint& point1)
    {
        if (point1 < point0)
        {
            first = point1;
            second = point0;
        }
        else
        {
            first = point0;
            second = point1;
        }
    }

    bool operator==(const GridEdgeKey& other) const
    {
        return first == other.first && second == other.second;
    }

    GridPoint first; // 字典序较小的端点。
    GridPoint second; // 字典序较大的端点。
};

// 连续曲面提取使用的双精度三维向量。
struct Vector3d
{
    Vector3d()
        : x(0.0)
        , y(0.0)
        , z(0.0)
    {
    }

    Vector3d(double xValue, double yValue, double zValue)
        : x(xValue)
        , y(yValue)
        , z(zValue)
    {
    }

    Vector3d operator+(const Vector3d& other) const
    {
        return Vector3d(x + other.x, y + other.y, z + other.z);
    }

    Vector3d operator-(const Vector3d& other) const
    {
        return Vector3d(x - other.x, y - other.y, z - other.z);
    }

    Vector3d operator*(double scale) const
    {
        return Vector3d(x * scale, y * scale, z * scale);
    }

    double x; // X分量。
    double y; // Y分量。
    double z; // Z分量。
};

// 计算三维整数点哈希值。
struct GridPointHash
{
    std::size_t operator()(const GridPoint& point) const
    {
        std::size_t result = std::hash<std::int64_t>()(point.x);

        result ^= std::hash<std::int64_t>()(point.y) + static_cast<std::size_t>(0x9e3779b9U) + (result << 6) + (result >> 2);
        result ^= std::hash<std::int64_t>()(point.z) + static_cast<std::size_t>(0x9e3779b9U) + (result << 6) + (result >> 2);
        return result;
    }
};

// 计算无方向网格边哈希值。
struct GridEdgeKeyHash
{
    std::size_t operator()(const GridEdgeKey& edge) const
    {
        GridPointHash hash;
        std::size_t result = hash(edge.first);

        result ^= hash(edge.second) + static_cast<std::size_t>(0x9e3779b9U) + (result << 6) + (result >> 2);
        return result;
    }
};

// 返回向量点积。
double dot(const Vector3d& first, const Vector3d& second)
{
    return first.x * second.x + first.y * second.y + first.z * second.z;
}

// 返回向量叉积。
Vector3d cross(const Vector3d& first, const Vector3d& second)
{
    return Vector3d(first.y * second.z - first.z * second.y, first.z * second.x - first.x * second.z, first.x * second.y - first.y * second.x);
}

// 返回向量平方长度。
double lengthSquared(const Vector3d& vector)
{
    return dot(vector, vector);
}

// 返回单位向量，无有效方向时返回零向量。
Vector3d normalized(const Vector3d& vector)
{
    const double squaredLength = lengthSquared(vector);

    if (squaredLength <= MinimumNormalLengthSquared)
    {
        return Vector3d();
    }

    return vector * (1.0 / std::sqrt(squaredLength));
}

// 返回两个向量的线性插值结果。
Vector3d interpolate(const Vector3d& first, const Vector3d& second, double ratio)
{
    return first + (second - first) * ratio;
}

// 将64位网格坐标转换为体素索引。
MyVoxel::VoxelIndex toVoxelIndex(std::int64_t value)
{
    assert(value >= static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)()));
    assert(value <= static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)()));
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回材料节点展开到最高层级后的单轴跨度。
std::int64_t levelScale(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    assert(level <= maximumLevel);

    const unsigned int difference = static_cast<unsigned int>(maximumLevel - level);

    assert(difference < 63);
    return std::int64_t(1) << difference;
}

// 使用最高层级网格坐标创建体素地址。
MyVoxel::VoxelCellAddress maximumLevelAddress(const MyVoxel::VoxelShape& shape, const GridPoint& point)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(toVoxelIndex(point.x), toVoxelIndex(point.y), toVoxelIndex(point.z)), shape.maximumLevel());
}

// 返回指定最高层级体素是否包含材料。
bool isMaximumLevelCellMaterial(const MyVoxel::VoxelShape& shape, const GridPoint& point)
{
    return shape.forest().state(maximumLevelAddress(shape, point)) == MyVoxel::VoxelState::Material;
}

// 向候选集合追加一个立方体原点及其扩展邻域。
void appendCandidateCell(std::vector<GridPoint>& candidates, const GridPoint& origin, int padding)
{
    for (int dz = -padding; dz <= padding; ++dz)
    {
        for (int dy = -padding; dy <= padding; ++dy)
        {
            for (int dx = -padding; dx <= padding; ++dx)
            {
                candidates.push_back(GridPoint(origin.x + dx, origin.y + dy, origin.z + dz));
            }
        }
    }
}

// 追加包含一条X方向材料边界采样边的四个候选立方体。
void appendXBoundaryCandidates(std::vector<GridPoint>& candidates, const GridPoint& point0, const GridPoint& point1, int padding)
{
    assert(std::abs(point0.x - point1.x) == 1);
    assert(point0.y == point1.y);
    assert(point0.z == point1.z);

    const std::int64_t minimumX = std::min(point0.x, point1.x);

    for (int yOffset = -1; yOffset <= 0; ++yOffset)
    {
        for (int zOffset = -1; zOffset <= 0; ++zOffset)
        {
            appendCandidateCell(candidates, GridPoint(minimumX, point0.y + yOffset, point0.z + zOffset), padding);
        }
    }
}

// 追加包含一条Y方向材料边界采样边的四个候选立方体。
void appendYBoundaryCandidates(std::vector<GridPoint>& candidates, const GridPoint& point0, const GridPoint& point1, int padding)
{
    assert(point0.x == point1.x);
    assert(std::abs(point0.y - point1.y) == 1);
    assert(point0.z == point1.z);

    const std::int64_t minimumY = std::min(point0.y, point1.y);

    for (int xOffset = -1; xOffset <= 0; ++xOffset)
    {
        for (int zOffset = -1; zOffset <= 0; ++zOffset)
        {
            appendCandidateCell(candidates, GridPoint(point0.x + xOffset, minimumY, point0.z + zOffset), padding);
        }
    }
}

// 追加包含一条Z方向材料边界采样边的四个候选立方体。
void appendZBoundaryCandidates(std::vector<GridPoint>& candidates, const GridPoint& point0, const GridPoint& point1, int padding)
{
    assert(point0.x == point1.x);
    assert(point0.y == point1.y);
    assert(std::abs(point0.z - point1.z) == 1);

    const std::int64_t minimumZ = std::min(point0.z, point1.z);

    for (int xOffset = -1; xOffset <= 0; ++xOffset)
    {
        for (int yOffset = -1; yOffset <= 0; ++yOffset)
        {
            appendCandidateCell(candidates, GridPoint(point0.x + xOffset, point0.y + yOffset, minimumZ), padding);
        }
    }
}

// 收集一个材料节点边界附近可能跨越等值面的最高层级立方体。
void collectMaterialCellCandidates(std::vector<GridPoint>& candidates, const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellAddress& address, int padding)
{
    assert(address.level <= shape.maximumLevel());

    const std::int64_t scale = levelScale(address.level, shape.maximumLevel());
    const std::int64_t startX = static_cast<std::int64_t>(address.index.x) * scale;
    const std::int64_t startY = static_cast<std::int64_t>(address.index.y) * scale;
    const std::int64_t startZ = static_cast<std::int64_t>(address.index.z) * scale;
    const std::int64_t endX = startX + scale - 1;
    const std::int64_t endY = startY + scale - 1;
    const std::int64_t endZ = startZ + scale - 1;

    for (std::int64_t y = startY; y <= endY; ++y)
    {
        for (std::int64_t z = startZ; z <= endZ; ++z)
        {
            const GridPoint negativeMaterial(startX, y, z);
            const GridPoint negativeNeighbor(startX - 1, y, z);

            if (!isMaximumLevelCellMaterial(shape, negativeNeighbor))
            {
                appendXBoundaryCandidates(candidates, negativeMaterial, negativeNeighbor, padding);
            }

            const GridPoint positiveMaterial(endX, y, z);
            const GridPoint positiveNeighbor(endX + 1, y, z);

            if (!isMaximumLevelCellMaterial(shape, positiveNeighbor))
            {
                appendXBoundaryCandidates(candidates, positiveMaterial, positiveNeighbor, padding);
            }
        }
    }

    for (std::int64_t x = startX; x <= endX; ++x)
    {
        for (std::int64_t z = startZ; z <= endZ; ++z)
        {
            const GridPoint negativeMaterial(x, startY, z);
            const GridPoint negativeNeighbor(x, startY - 1, z);

            if (!isMaximumLevelCellMaterial(shape, negativeNeighbor))
            {
                appendYBoundaryCandidates(candidates, negativeMaterial, negativeNeighbor, padding);
            }

            const GridPoint positiveMaterial(x, endY, z);
            const GridPoint positiveNeighbor(x, endY + 1, z);

            if (!isMaximumLevelCellMaterial(shape, positiveNeighbor))
            {
                appendYBoundaryCandidates(candidates, positiveMaterial, positiveNeighbor, padding);
            }
        }
    }

    for (std::int64_t x = startX; x <= endX; ++x)
    {
        for (std::int64_t y = startY; y <= endY; ++y)
        {
            const GridPoint negativeMaterial(x, y, startZ);
            const GridPoint negativeNeighbor(x, y, startZ - 1);

            if (!isMaximumLevelCellMaterial(shape, negativeNeighbor))
            {
                appendZBoundaryCandidates(candidates, negativeMaterial, negativeNeighbor, padding);
            }

            const GridPoint positiveMaterial(x, y, endZ);
            const GridPoint positiveNeighbor(x, y, endZ + 1);

            if (!isMaximumLevelCellMaterial(shape, positiveNeighbor))
            {
                appendZBoundaryCandidates(candidates, positiveMaterial, positiveNeighbor, padding);
            }
        }
    }
}

// 收集并去重全部连续曲面候选立方体。
std::vector<GridPoint> collectCandidateCells(const MyVoxel::VoxelShape& shape, int padding)
{
    std::vector<GridPoint> candidates;

    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            collectMaterialCellCandidates(candidates, shape, address, padding);
        });

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    return candidates;
}

// 保存连续等值面生成过程中的采样、法线和顶点缓存。
class SmoothExtractionContext
{
public:
    SmoothExtractionContext(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelSurfaceColor& color, double isoValue)
        : m_shape(shape)
        , m_color(color)
        , m_isoValue(isoValue)
        , m_edgeLength(shape.voxelEdgeLength(shape.maximumLevel()))
        , m_pointTransform(shape.transform())
    {
        assert(m_pointTransform.isAffine());

        MyMath::Matrix4 inverseTransform;
        const bool inverted = m_pointTransform.inverted(inverseTransform);

        assert(inverted);
        m_normalTransform = inverseTransform.transposed();
    }

    // 移出最终生成的表面网格。
    MyVoxel::VoxelSurfaceMesh takeMesh()
    {
        return std::move(m_mesh);
    }

    // 返回指定最高层级体素中心的二值材料值。
    double sampleValue(const GridPoint& point)
    {
        const std::unordered_map<GridPoint, double, GridPointHash>::const_iterator iterator = m_sampleValues.find(point);

        if (iterator != m_sampleValues.end())
        {
            return iterator->second;
        }

        const double value = isMaximumLevelCellMaterial(m_shape, point) ? 1.0 : 0.0;

        m_sampleValues.insert(std::make_pair(point, value));
        return value;
    }

    // 根据相邻材料值计算由材料内部指向外部的局部平滑法线。
    Vector3d sampleNormal(const GridPoint& point)
    {
        const std::unordered_map<GridPoint, Vector3d, GridPointHash>::const_iterator iterator = m_sampleNormals.find(point);

        if (iterator != m_sampleNormals.end())
        {
            return iterator->second;
        }

        const double gradientX = sampleValue(GridPoint(point.x + 1, point.y, point.z)) - sampleValue(GridPoint(point.x - 1, point.y, point.z));
        const double gradientY = sampleValue(GridPoint(point.x, point.y + 1, point.z)) - sampleValue(GridPoint(point.x, point.y - 1, point.z));
        const double gradientZ = sampleValue(GridPoint(point.x, point.y, point.z + 1)) - sampleValue(GridPoint(point.x, point.y, point.z - 1));
        const Vector3d normal = normalized(Vector3d(-gradientX, -gradientY, -gradientZ));

        m_sampleNormals.insert(std::make_pair(point, normal));
        return normal;
    }

    // 返回或创建指定等值面交叉边上的共享顶点索引。
    std::uint32_t edgeVertex(const GridPoint& point0, const GridPoint& point1, double value0, double value1)
    {
        const GridEdgeKey edge(point0, point1);
        const std::unordered_map<GridEdgeKey, std::uint32_t, GridEdgeKeyHash>::const_iterator iterator = m_edgeVertices.find(edge);

        if (iterator != m_edgeVertices.end())
        {
            return iterator->second;
        }

        const double difference = value1 - value0;
        double ratio = std::abs(difference) > MinimumInterpolationDifference ? (m_isoValue - value0) / difference : 0.5;

        ratio = std::max(0.0, std::min(ratio, 1.0));

        const Vector3d localPoint0((static_cast<double>(point0.x) + 0.5) * m_edgeLength, (static_cast<double>(point0.y) + 0.5) * m_edgeLength, (static_cast<double>(point0.z) + 0.5) * m_edgeLength);
        const Vector3d localPoint1((static_cast<double>(point1.x) + 0.5) * m_edgeLength, (static_cast<double>(point1.y) + 0.5) * m_edgeLength, (static_cast<double>(point1.z) + 0.5) * m_edgeLength);
        const Vector3d localPoint = interpolate(localPoint0, localPoint1, ratio);
        Vector3d localNormal = normalized(interpolate(sampleNormal(point0), sampleNormal(point1), ratio));

        // 中心差分退化时，使用材料采样点指向空采样点的边方向作为外法线。
        if (lengthSquared(localNormal) <= MinimumNormalLengthSquared)
        {
            localNormal = value0 > value1 ? normalized(localPoint1 - localPoint0) : normalized(localPoint0 - localPoint1);
        }

        const MyMath::Vector3 worldPoint = m_pointTransform.transformPoint(MyMath::Vector3(localPoint.x, localPoint.y, localPoint.z));
        const MyMath::Vector3 worldNormal = m_normalTransform.transformVector(MyMath::Vector3(localNormal.x, localNormal.y, localNormal.z)).normalized();

        assert(worldPoint.isFinite());
        assert(worldNormal.isVector());

        const MyVoxel::VoxelSurfaceVertex vertex(worldPoint.x(), worldPoint.y(), worldPoint.z(), worldNormal.x(), worldNormal.y(), worldNormal.z(), m_color);
        const std::uint32_t vertexIndex = m_mesh.appendVertex(vertex);

        m_edgeVertices.insert(std::make_pair(edge, vertexIndex));
        return vertexIndex;
    }

    // 追加与顶点平滑法线方向一致的非退化三角形。
    void appendOrientedTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
    {
        if (index0 == index1 || index1 == index2 || index2 == index0)
        {
            return;
        }

        const std::vector<MyVoxel::VoxelSurfaceVertex>& vertices = m_mesh.vertices();
        const MyVoxel::VoxelSurfaceVertex& vertex0 = vertices[static_cast<std::size_t>(index0)];
        const MyVoxel::VoxelSurfaceVertex& vertex1 = vertices[static_cast<std::size_t>(index1)];
        const MyVoxel::VoxelSurfaceVertex& vertex2 = vertices[static_cast<std::size_t>(index2)];
        const Vector3d point0(vertex0.x, vertex0.y, vertex0.z);
        const Vector3d point1(vertex1.x, vertex1.y, vertex1.z);
        const Vector3d point2(vertex2.x, vertex2.y, vertex2.z);
        const Vector3d faceNormal = cross(point1 - point0, point2 - point0);

        if (lengthSquared(faceNormal) <= MinimumTriangleAreaSquared)
        {
            return;
        }

        const Vector3d averageNormal(vertex0.nx + vertex1.nx + vertex2.nx, vertex0.ny + vertex1.ny + vertex2.ny, vertex0.nz + vertex1.nz + vertex2.nz);

        if (dot(faceNormal, averageNormal) < 0.0)
        {
            std::swap(index1, index2);
        }

        m_mesh.appendTriangle(index0, index1, index2);
    }

private:
    const MyVoxel::VoxelShape& m_shape; // 被提取的只读体素形体。
    const MyVoxel::VoxelSurfaceColor& m_color; // 生成顶点使用的显示颜色。
    double m_isoValue; // 当前等值面阈值。
    double m_edgeLength; // 最高层级体素边长。
    const MyMath::Matrix4& m_pointTransform; // 体素局部坐标到世界坐标的点变换。
    MyMath::Matrix4 m_normalTransform; // 局部法线到世界法线的逆转置变换。
    MyVoxel::VoxelSurfaceMesh m_mesh; // 当前生成网格。
    std::unordered_map<GridPoint, double, GridPointHash> m_sampleValues; // 二值材料采样缓存。
    std::unordered_map<GridPoint, Vector3d, GridPointHash> m_sampleNormals; // 平滑法线缓存。
    std::unordered_map<GridEdgeKey, std::uint32_t, GridEdgeKeyHash> m_edgeVertices; // 等值面边顶点缓存。
};

// 对一个四面体执行等值面三角化。
void polygonizeTetrahedron(SmoothExtractionContext& context, const GridPoint cubePoints[8], const double cubeValues[8], const int tetrahedron[4], double isoValue)
{
    int inside[4];
    int outside[4];
    int insideCount = 0;
    int outsideCount = 0;

    for (int i = 0; i < 4; ++i)
    {
        const int cubeIndex = tetrahedron[i];

        if (cubeValues[cubeIndex] >= isoValue)
        {
            inside[insideCount++] = cubeIndex;
        }
        else
        {
            outside[outsideCount++] = cubeIndex;
        }
    }

    if (insideCount == 0 || insideCount == 4)
    {
        return;
    }

    if (insideCount == 1)
    {
        const int materialIndex = inside[0];
        const std::uint32_t index0 = context.edgeVertex(cubePoints[materialIndex], cubePoints[outside[0]], cubeValues[materialIndex], cubeValues[outside[0]]);
        const std::uint32_t index1 = context.edgeVertex(cubePoints[materialIndex], cubePoints[outside[1]], cubeValues[materialIndex], cubeValues[outside[1]]);
        const std::uint32_t index2 = context.edgeVertex(cubePoints[materialIndex], cubePoints[outside[2]], cubeValues[materialIndex], cubeValues[outside[2]]);

        context.appendOrientedTriangle(index0, index1, index2);
        return;
    }

    if (insideCount == 3)
    {
        const int emptyIndex = outside[0];
        const std::uint32_t index0 = context.edgeVertex(cubePoints[emptyIndex], cubePoints[inside[0]], cubeValues[emptyIndex], cubeValues[inside[0]]);
        const std::uint32_t index1 = context.edgeVertex(cubePoints[emptyIndex], cubePoints[inside[1]], cubeValues[emptyIndex], cubeValues[inside[1]]);
        const std::uint32_t index2 = context.edgeVertex(cubePoints[emptyIndex], cubePoints[inside[2]], cubeValues[emptyIndex], cubeValues[inside[2]]);

        context.appendOrientedTriangle(index0, index1, index2);
        return;
    }

    assert(insideCount == 2);
    assert(outsideCount == 2);

    const std::uint32_t index0 = context.edgeVertex(cubePoints[inside[0]], cubePoints[outside[0]], cubeValues[inside[0]], cubeValues[outside[0]]);
    const std::uint32_t index1 = context.edgeVertex(cubePoints[inside[0]], cubePoints[outside[1]], cubeValues[inside[0]], cubeValues[outside[1]]);
    const std::uint32_t index2 = context.edgeVertex(cubePoints[inside[1]], cubePoints[outside[0]], cubeValues[inside[1]], cubeValues[outside[0]]);
    const std::uint32_t index3 = context.edgeVertex(cubePoints[inside[1]], cubePoints[outside[1]], cubeValues[inside[1]], cubeValues[outside[1]]);

    context.appendOrientedTriangle(index0, index1, index3);
    context.appendOrientedTriangle(index0, index3, index2);
}

// 对一个最高层级采样立方体执行六四面体等值面提取。
void polygonizeCube(SmoothExtractionContext& context, const GridPoint& cell, double isoValue)
{
    const GridPoint points[8] =
    {
        GridPoint(cell.x, cell.y, cell.z),
        GridPoint(cell.x + 1, cell.y, cell.z),
        GridPoint(cell.x + 1, cell.y + 1, cell.z),
        GridPoint(cell.x, cell.y + 1, cell.z),
        GridPoint(cell.x, cell.y, cell.z + 1),
        GridPoint(cell.x + 1, cell.y, cell.z + 1),
        GridPoint(cell.x + 1, cell.y + 1, cell.z + 1),
        GridPoint(cell.x, cell.y + 1, cell.z + 1)
    };

    double values[8];
    int materialCount = 0;

    for (int i = 0; i < 8; ++i)
    {
        values[i] = context.sampleValue(points[i]);

        if (values[i] >= isoValue)
        {
            ++materialCount;
        }
    }

    if (materialCount == 0 || materialCount == 8)
    {
        return;
    }

    // 六个四面体统一共享0到6主对角线，保证相邻立方体使用一致拆分规则。
    const int tetrahedra[6][4] =
    {
        {0, 5, 1, 6},
        {0, 1, 2, 6},
        {0, 2, 3, 6},
        {0, 3, 7, 6},
        {0, 7, 4, 6},
        {0, 4, 5, 6}
    };

    for (int i = 0; i < 6; ++i)
    {
        polygonizeTetrahedron(context, points, values, tetrahedra[i], isoValue);
    }
}

}

namespace MyVoxel
{

VoxelSurfaceMesh VoxelSmoothSurfaceExtractor::extract(const VoxelShape& shape, const VoxelSurfaceColor& color, const VoxelSurfaceGenerationOptions& options)
{
    assert(options.mode == VoxelSurfaceGenerationMode::SmoothMarchingTetrahedra);
    assert(std::isfinite(options.isoValue));
    assert(options.isoValue > 0.0 && options.isoValue < 1.0);
    assert(options.boundaryPadding >= 0);
    assert(shape.transform().isAffine());

    const std::vector<GridPoint> candidateCells = collectCandidateCells(shape, options.boundaryPadding);
    SmoothExtractionContext context(shape, color, options.isoValue);

    for (std::size_t i = 0; i < candidateCells.size(); ++i)
    {
        polygonizeCube(context, candidateCells[i], options.isoValue);
    }

    return context.takeMesh();
}

}