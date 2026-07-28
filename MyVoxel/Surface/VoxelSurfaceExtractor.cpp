#include "VoxelSurfaceExtractor.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "VoxelSmoothSurfaceExtractor.h"

namespace
{

// 表面方向。
enum class SurfaceFaceDirection
{
    NegativeX,
    PositiveX,
    NegativeY,
    PositiveY,
    NegativeZ,
    PositiveZ
};

// 同一方向、同一固定坐标的表面平面。
struct SurfacePlaneKey
{
    SurfacePlaneKey()
        : direction(SurfaceFaceDirection::PositiveZ)
        , fixedCoordinate(0)
    {
    }

    SurfacePlaneKey(SurfaceFaceDirection directionValue, std::int64_t fixedCoordinateValue)
        : direction(directionValue)
        , fixedCoordinate(fixedCoordinateValue)
    {
    }

    bool operator<(const SurfacePlaneKey& other) const
    {
        if (direction != other.direction)
        {
            return static_cast<int>(direction) < static_cast<int>(other.direction);
        }

        return fixedCoordinate < other.fixedCoordinate;
    }

    SurfaceFaceDirection direction; // 表面方向。
    std::int64_t fixedCoordinate; // 固定轴上的最高层级网格坐标。
};

// 平面内的一个最高层级表面单元。
struct SurfaceCell
{
    SurfaceCell()
        : u(0)
        , v(0)
    {
    }

    SurfaceCell(std::int64_t uValue, std::int64_t vValue)
        : u(uValue)
        , v(vValue)
    {
    }

    bool operator<(const SurfaceCell& other) const
    {
        if (u != other.u)
        {
            return u < other.u;
        }

        return v < other.v;
    }

    std::int64_t u; // 平面内第一坐标。
    std::int64_t v; // 平面内第二坐标。
};

using SurfaceCellSet = std::set<SurfaceCell>;
using SurfacePlaneMap = std::map<SurfacePlaneKey, SurfaceCellSet>;

// 返回指定材料节点展开到最高层级后的单轴体素跨度。
std::int64_t levelScale(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    assert(level <= maximumLevel);

    const unsigned int difference = static_cast<unsigned int>(maximumLevel - level);

    assert(difference < 63);
    return std::int64_t(1) << difference;
}

// 将64位网格坐标转换为体素索引。
MyVoxel::VoxelIndex toVoxelIndex(std::int64_t value)
{
    assert(value >= static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)()));
    assert(value <= static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)()));
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 使用最高层级网格坐标创建体素地址。
MyVoxel::VoxelCellAddress maximumLevelAddress(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(toVoxelIndex(x), toVoxelIndex(y), toVoxelIndex(z)), shape.maximumLevel());
}

// 返回最高层级地址是否为材料。
bool isMaximumLevelCellMaterial(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    return shape.forest().state(maximumLevelAddress(shape, x, y, z)) == MyVoxel::VoxelState::Material;
}

// 使用最高层级网格坐标创建世界点。
MyMath::Vector3 gridPoint(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    const double edgeLength = shape.voxelEdgeLength(shape.maximumLevel());
    const MyMath::Vector3 localPoint(static_cast<double>(x) * edgeLength, static_cast<double>(y) * edgeLength, static_cast<double>(z) * edgeLength);
    return shape.transform().transformPoint(localPoint);
}

// 创建法线变换矩阵。
MyMath::Matrix4 createNormalTransform(const MyVoxel::VoxelShape& shape)
{
    assert(shape.transform().isAffine());

    MyMath::Matrix4 inverseTransform;
    const bool inverted = shape.transform().inverted(inverseTransform);

    assert(inverted);
    return inverseTransform.transposed();
}

// 将局部法线转换为世界法线。
MyMath::Vector3 transformNormal(const MyMath::Matrix4& normalTransform, double x, double y, double z)
{
    const MyMath::Vector3 result = normalTransform.transformVector(MyMath::Vector3(x, y, z)).normalized();

    assert(result.isVector());
    return result;
}

// 创建表面顶点。
MyVoxel::VoxelSurfaceVertex makeVertex(const MyVoxel::VoxelShape& shape, const MyMath::Matrix4& normalTransform, std::int64_t x, std::int64_t y, std::int64_t z, double normalX, double normalY, double normalZ, const MyVoxel::VoxelSurfaceColor& color)
{
    const MyMath::Vector3 point = gridPoint(shape, x, y, z);
    const MyMath::Vector3 normal = transformNormal(normalTransform, normalX, normalY, normalZ);
    return MyVoxel::VoxelSurfaceVertex(point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z(), color);
}

// 追加一个暴露表面单元到待合并集合。
void addSurfaceCell(SurfacePlaneMap& planes, SurfaceFaceDirection direction, std::int64_t fixedCoordinate, std::int64_t u, std::int64_t v)
{
    planes[SurfacePlaneKey(direction, fixedCoordinate)].insert(SurfaceCell(u, v));
}

// 返回集合中是否存在指定表面单元。
bool containsSurfaceCell(const SurfaceCellSet& cells, std::int64_t u, std::int64_t v)
{
    return cells.find(SurfaceCell(u, v)) != cells.end();
}

// 追加合并后的矩形表面。
void appendMergedFace(MyVoxel::VoxelSurfaceMesh& mesh, const MyVoxel::VoxelShape& shape, const MyMath::Matrix4& normalTransform, const SurfacePlaneKey& key, std::int64_t u0, std::int64_t v0, std::int64_t u1, std::int64_t v1, const MyVoxel::VoxelSurfaceColor& color)
{
    if (key.direction == SurfaceFaceDirection::NegativeX)
    {
        const std::int64_t x = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, normalTransform, x, u0, v0, -1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u0, v1, -1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u1, v1, -1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u1, v0, -1.0, 0.0, 0.0, color));

        return;
    }

    if (key.direction == SurfaceFaceDirection::PositiveX)
    {
        const std::int64_t x = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, normalTransform, x, u0, v0, 1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u1, v0, 1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u1, v1, 1.0, 0.0, 0.0, color),
            makeVertex(shape, normalTransform, x, u0, v1, 1.0, 0.0, 0.0, color));

        return;
    }

    if (key.direction == SurfaceFaceDirection::NegativeY)
    {
        const std::int64_t y = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, normalTransform, u0, y, v0, 0.0, -1.0, 0.0, color),
            makeVertex(shape, normalTransform, u1, y, v0, 0.0, -1.0, 0.0, color),
            makeVertex(shape, normalTransform, u1, y, v1, 0.0, -1.0, 0.0, color),
            makeVertex(shape, normalTransform, u0, y, v1, 0.0, -1.0, 0.0, color));

        return;
    }

    if (key.direction == SurfaceFaceDirection::PositiveY)
    {
        const std::int64_t y = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, normalTransform, u0, y, v0, 0.0, 1.0, 0.0, color),
            makeVertex(shape, normalTransform, u0, y, v1, 0.0, 1.0, 0.0, color),
            makeVertex(shape, normalTransform, u1, y, v1, 0.0, 1.0, 0.0, color),
            makeVertex(shape, normalTransform, u1, y, v0, 0.0, 1.0, 0.0, color));

        return;
    }

    if (key.direction == SurfaceFaceDirection::NegativeZ)
    {
        const std::int64_t z = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, normalTransform, u0, v0, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, normalTransform, u0, v1, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, normalTransform, u1, v1, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, normalTransform, u1, v0, z, 0.0, 0.0, -1.0, color));

        return;
    }

    const std::int64_t z = key.fixedCoordinate;

    mesh.appendQuad(
        makeVertex(shape, normalTransform, u0, v0, z, 0.0, 0.0, 1.0, color),
        makeVertex(shape, normalTransform, u1, v0, z, 0.0, 0.0, 1.0, color),
        makeVertex(shape, normalTransform, u1, v1, z, 0.0, 0.0, 1.0, color),
        makeVertex(shape, normalTransform, u0, v1, z, 0.0, 0.0, 1.0, color));
}

// 对一个平面内的外表面单元执行矩形贪心合并。
void appendGreedyPlane(MyVoxel::VoxelSurfaceMesh& mesh, const MyVoxel::VoxelShape& shape, const MyMath::Matrix4& normalTransform, const SurfacePlaneKey& key, SurfaceCellSet cells, const MyVoxel::VoxelSurfaceColor& color)
{
    while (!cells.empty())
    {
        const SurfaceCell start = *cells.begin();
        std::int64_t width = 1;

        while (containsSurfaceCell(cells, start.u + width, start.v))
        {
            ++width;
        }

        std::int64_t height = 1;

        while (true)
        {
            bool completeRow = true;

            for (std::int64_t offset = 0; offset < width; ++offset)
            {
                if (!containsSurfaceCell(cells, start.u + offset, start.v + height))
                {
                    completeRow = false;
                    break;
                }
            }

            if (!completeRow)
            {
                break;
            }

            ++height;
        }

        for (std::int64_t offsetU = 0; offsetU < width; ++offsetU)
        {
            for (std::int64_t offsetV = 0; offsetV < height; ++offsetV)
            {
                cells.erase(SurfaceCell(start.u + offsetU, start.v + offsetV));
            }
        }

        appendMergedFace(mesh, shape, normalTransform, key, start.u, start.v, start.u + width, start.v + height, color);
    }
}

// 收集一个材料节点展开到最高层级后的暴露边界面。
void collectMaterialCellSurface(SurfacePlaneMap& planes, const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellAddress& address)
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
            if (!isMaximumLevelCellMaterial(shape, startX - 1, y, z))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::NegativeX, startX, y, z);
            }

            if (!isMaximumLevelCellMaterial(shape, endX + 1, y, z))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::PositiveX, endX + 1, y, z);
            }
        }
    }

    for (std::int64_t x = startX; x <= endX; ++x)
    {
        for (std::int64_t z = startZ; z <= endZ; ++z)
        {
            if (!isMaximumLevelCellMaterial(shape, x, startY - 1, z))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::NegativeY, startY, x, z);
            }

            if (!isMaximumLevelCellMaterial(shape, x, endY + 1, z))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::PositiveY, endY + 1, x, z);
            }
        }
    }

    for (std::int64_t x = startX; x <= endX; ++x)
    {
        for (std::int64_t y = startY; y <= endY; ++y)
        {
            if (!isMaximumLevelCellMaterial(shape, x, y, startZ - 1))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::NegativeZ, startZ, x, y);
            }

            if (!isMaximumLevelCellMaterial(shape, x, y, endZ + 1))
            {
                addSurfaceCell(planes, SurfaceFaceDirection::PositiveZ, endZ + 1, x, y);
            }
        }
    }
}

// 将已经收集的暴露表面单元构造为贪心表面网格。
MyVoxel::VoxelSurfaceMesh buildGreedyMesh(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelSurfaceColor& color, const SurfacePlaneMap& planes)
{
    const MyMath::Matrix4 normalTransform = createNormalTransform(shape);
    MyVoxel::VoxelSurfaceMesh mesh;

    for (SurfacePlaneMap::const_iterator iterator = planes.begin(); iterator != planes.end(); ++iterator)
    {
        appendGreedyPlane(mesh, shape, normalTransform, iterator->first, iterator->second, color);
    }

    return mesh;
}

// 使用轴对齐贪心网格算法提取完整体素外表面。
MyVoxel::VoxelSurfaceMesh extractGreedySurface(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelSurfaceColor& color)
{
    SurfacePlaneMap planes;

    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            collectMaterialCellSurface(planes, shape, address);
        });

    return buildGreedyMesh(shape, color, planes);
}

// 使用轴对齐贪心网格算法提取指定根节点内材料产生的外表面。
MyVoxel::VoxelSurfaceMesh extractGreedyRootSurface(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellIndex& rootIndex, const MyVoxel::VoxelSurfaceColor& color)
{
    SurfacePlaneMap planes;

    shape.forest().forEachMaterialCellInRootRange(
        rootIndex,
        rootIndex,
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            collectMaterialCellSurface(planes, shape, address);
        });

    return buildGreedyMesh(shape, color, planes);
}

}

namespace MyVoxel
{

VoxelSurfaceMesh VoxelSurfaceExtractor::extract(const VoxelShape& shape, const VoxelSurfaceColor& color)
{
    return extractGreedySurface(shape, color);
}

VoxelSurfaceMesh VoxelSurfaceExtractor::extract(const VoxelShape& shape, const VoxelSurfaceColor& color, VoxelSurfaceGenerationMode mode)
{
    VoxelSurfaceGenerationOptions options;
    options.mode = mode;
    return extract(shape, color, options);
}

VoxelSurfaceMesh VoxelSurfaceExtractor::extract(const VoxelShape& shape, const VoxelSurfaceColor& color, const VoxelSurfaceGenerationOptions& options)
{
    assert(std::isfinite(options.isoValue));
    assert(options.boundaryPadding >= 0);

    switch (options.mode)
    {
    case VoxelSurfaceGenerationMode::GreedyVoxel:
        return extractGreedySurface(shape, color);

    case VoxelSurfaceGenerationMode::SmoothMarchingTetrahedra:
        return VoxelSmoothSurfaceExtractor::extract(shape, color, options);
    }

    assert(false);
    return VoxelSurfaceMesh();
}

VoxelSurfaceMesh VoxelSurfaceExtractor::extractRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndex, const VoxelSurfaceColor& color)
{
    return extractGreedyRootSurface(shape, rootIndex, color);
}

}