#include "VoxelSurfaceExtractor.h"

#include <cstdint>
#include <limits>
#include <map>
#include <set>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 表示一个轴对齐外表面的朝向。
enum class SurfaceFaceDirection
{
    NegativeX,
    PositiveX,
    NegativeY,
    PositiveY,
    NegativeZ,
    PositiveZ
};

// 唯一标识一个固定坐标和法线方向的最高层级表面平面。
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

    SurfaceFaceDirection direction; // 当前表面平面的外法线方向。
    std::int64_t fixedCoordinate; // 当前平面固定轴上的最高层级网格坐标。
};

// 表示一个表面平面内的最高层级正方形单元。
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

    std::int64_t u; // 当前平面中的第一坐标。
    std::int64_t v; // 当前平面中的第二坐标。
};

using SurfaceCellSet = std::set<SurfaceCell>;
using SurfacePlaneMap = std::map<SurfacePlaneKey, SurfaceCellSet>;

// 返回指定材料节点展开到最高层级后的单轴体素数量。
std::int64_t maximumLevelScale(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    MYVOXEL_ASSERT_MESSAGE(level <= maximumLevel, "Voxel surface extraction level exceeds the maximum voxel level.");

    const unsigned int levelDifference = static_cast<unsigned int>(maximumLevel - level);

    MYVOXEL_ASSERT_MESSAGE(levelDifference < 63, "Voxel surface extraction level difference exceeds the supported 64-bit range.");
    return std::int64_t(1) << levelDifference;
}

// 将64位网格坐标转换为体素索引。
MyVoxel::VoxelIndex toVoxelIndex(std::int64_t value)
{
    const std::int64_t minimumValue = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximumValue = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(value >= minimumValue && value <= maximumValue, "Voxel surface grid coordinate exceeds the VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 使用最高层级网格坐标创建体素地址。
MyVoxel::VoxelCellAddress maximumLevelAddress(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    return MyVoxel::VoxelCellAddress(
        MyVoxel::VoxelCellIndex(toVoxelIndex(x), toVoxelIndex(y), toVoxelIndex(z)),
        shape.grid().maximumLevel());
}

// 判断指定最高层级体素是否包含材料。
bool isMaximumLevelCellMaterial(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    return shape.forest().state(maximumLevelAddress(shape, x, y, z)) == MyVoxel::VoxelState::Material;
}

// 将最高层级网格角点转换为VoxelShape局部坐标。
MyMath::Vector3 localGridPoint(const MyVoxel::VoxelShape& shape, std::int64_t x, std::int64_t y, std::int64_t z)
{
    const MyVoxel::VoxelGrid& grid = shape.grid();
    const MyMath::Vector3& origin = grid.origin();
    const double edgeLength = grid.minimumCellEdgeLength();

    return MyMath::Vector3(
        origin.x() + static_cast<double>(x) * edgeLength,
        origin.y() + static_cast<double>(y) * edgeLength,
        origin.z() + static_cast<double>(z) * edgeLength);
}

// 创建一个局部空间体素表面顶点。
MyVoxel::VoxelSurfaceVertex makeVertex(const MyVoxel::VoxelShape& shape,
                                       std::int64_t x,
                                       std::int64_t y,
                                       std::int64_t z,
                                       double normalX,
                                       double normalY,
                                       double normalZ,
                                       const MyVoxel::VoxelSurfaceColor& color)
{
    const MyMath::Vector3 point = localGridPoint(shape, x, y, z);

    return MyVoxel::VoxelSurfaceVertex(
        point.x(),
        point.y(),
        point.z(),
        normalX,
        normalY,
        normalZ,
        color);
}

// 向指定表面平面加入一个最高层级表面单元。
void addSurfaceCell(SurfacePlaneMap& planes, SurfaceFaceDirection direction, std::int64_t fixedCoordinate, std::int64_t u, std::int64_t v)
{
    planes[SurfacePlaneKey(direction, fixedCoordinate)].insert(SurfaceCell(u, v));
}

// 判断集合中是否存在指定表面单元。
bool containsSurfaceCell(const SurfaceCellSet& cells, std::int64_t u, std::int64_t v)
{
    return cells.find(SurfaceCell(u, v)) != cells.end();
}

// 追加一个已经完成贪心合并的矩形外表面。
void appendMergedFace(MyVoxel::VoxelSurfaceMesh& mesh,
                      const MyVoxel::VoxelShape& shape,
                      const SurfacePlaneKey& key,
                      std::int64_t u0,
                      std::int64_t v0,
                      std::int64_t u1,
                      std::int64_t v1,
                      const MyVoxel::VoxelSurfaceColor& color)
{
    switch (key.direction)
    {
    case SurfaceFaceDirection::NegativeX:
    {
        const std::int64_t x = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, x, u0, v0, -1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u0, v1, -1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u1, v1, -1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u1, v0, -1.0, 0.0, 0.0, color));

        return;
    }

    case SurfaceFaceDirection::PositiveX:
    {
        const std::int64_t x = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, x, u0, v0, 1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u1, v0, 1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u1, v1, 1.0, 0.0, 0.0, color),
            makeVertex(shape, x, u0, v1, 1.0, 0.0, 0.0, color));

        return;
    }

    case SurfaceFaceDirection::NegativeY:
    {
        const std::int64_t y = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, u0, y, v0, 0.0, -1.0, 0.0, color),
            makeVertex(shape, u1, y, v0, 0.0, -1.0, 0.0, color),
            makeVertex(shape, u1, y, v1, 0.0, -1.0, 0.0, color),
            makeVertex(shape, u0, y, v1, 0.0, -1.0, 0.0, color));

        return;
    }

    case SurfaceFaceDirection::PositiveY:
    {
        const std::int64_t y = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, u0, y, v0, 0.0, 1.0, 0.0, color),
            makeVertex(shape, u0, y, v1, 0.0, 1.0, 0.0, color),
            makeVertex(shape, u1, y, v1, 0.0, 1.0, 0.0, color),
            makeVertex(shape, u1, y, v0, 0.0, 1.0, 0.0, color));

        return;
    }

    case SurfaceFaceDirection::NegativeZ:
    {
        const std::int64_t z = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, u0, v0, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, u0, v1, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, u1, v1, z, 0.0, 0.0, -1.0, color),
            makeVertex(shape, u1, v0, z, 0.0, 0.0, -1.0, color));

        return;
    }

    case SurfaceFaceDirection::PositiveZ:
    {
        const std::int64_t z = key.fixedCoordinate;

        mesh.appendQuad(
            makeVertex(shape, u0, v0, z, 0.0, 0.0, 1.0, color),
            makeVertex(shape, u1, v0, z, 0.0, 0.0, 1.0, color),
            makeVertex(shape, u1, v1, z, 0.0, 0.0, 1.0, color),
            makeVertex(shape, u0, v1, z, 0.0, 0.0, 1.0, color));

        return;
    }
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel surface extraction encountered an unsupported face direction.");
}

// 对同一平面的表面单元执行矩形贪心合并。
void appendGreedyPlane(MyVoxel::VoxelSurfaceMesh& mesh,
                       const MyVoxel::VoxelShape& shape,
                       const SurfacePlaneKey& key,
                       SurfaceCellSet cells,
                       const MyVoxel::VoxelSurfaceColor& color)
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

        appendMergedFace(mesh, shape, key, start.u, start.v, start.u + width, start.v + height, color);
    }
}

// 收集一个Material节点展开到最高层级后产生的全部暴露表面单元。
void collectMaterialCellSurface(SurfacePlaneMap& planes, const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellAddress& address)
{
    const MyVoxel::VoxelLevel maximumLevel = shape.grid().maximumLevel();

    MYVOXEL_ASSERT_MESSAGE(address.level <= maximumLevel, "Material cell level exceeds the VoxelShape maximum level.");

    const std::int64_t scale = maximumLevelScale(address.level, maximumLevel);
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

// 将已经收集的外表面单元转换为贪心合并三角网格。
MyVoxel::VoxelSurfaceMesh buildMesh(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelSurfaceColor& color, const SurfacePlaneMap& planes)
{
    MyVoxel::VoxelSurfaceMesh mesh;

    for (SurfacePlaneMap::const_iterator iterator = planes.begin(); iterator != planes.end(); ++iterator)
    {
        appendGreedyPlane(mesh, shape, iterator->first, iterator->second, color);
    }

    return mesh;
}

// 提取完整体素形体外表面。
MyVoxel::VoxelSurfaceMesh extractCompleteSurface(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelSurfaceColor& color)
{
    SurfacePlaneMap planes;

    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            collectMaterialCellSurface(planes, shape, address);
        });

    return buildMesh(shape, color, planes);
}

// 提取指定根节点内材料产生的外表面。
MyVoxel::VoxelSurfaceMesh extractRootSurface(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellIndex& rootIndex, const MyVoxel::VoxelSurfaceColor& color)
{
    SurfacePlaneMap planes;

    shape.forest().forEachMaterialCellInRootRange(
        rootIndex,
        rootIndex,
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            collectMaterialCellSurface(planes, shape, address);
        });

    return buildMesh(shape, color, planes);
}

}

namespace MyVoxel
{

VoxelSurfaceMesh VoxelSurfaceExtractor::extract(const VoxelShape& shape, const VoxelSurfaceColor& color)
{
    MYVOXEL_ASSERT_MESSAGE(shape.grid().isValid(), "Voxel surface extraction requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(shape.transform().isAffine(), "Voxel surface extraction requires an affine VoxelShape transform.");

    return extractCompleteSurface(shape, color);
}

VoxelSurfaceMesh VoxelSurfaceExtractor::extractRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndex, const VoxelSurfaceColor& color)
{
    MYVOXEL_ASSERT_MESSAGE(shape.grid().isValid(), "Root surface extraction requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(shape.transform().isAffine(), "Root surface extraction requires an affine VoxelShape transform.");

    return extractRootSurface(shape, rootIndex, color);
}

}