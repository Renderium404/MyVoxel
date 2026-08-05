#include "VoxelSurfaceMesher.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/Stopwatch.h"

#include "VoxelFaceExtractor.h"

namespace
{

// 返回非零64位掩码中最低置位的位索引。
unsigned int firstSetBitIndex(std::uint64_t mask)
{
    MYVOXEL_ASSERT_MESSAGE(mask != 0, "Cannot query the first set bit of an empty surface mask.");

#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long bitIndex = 0;
    _BitScanForward64(&bitIndex, mask);
    return static_cast<unsigned int>(bitIndex);
#elif defined(_MSC_VER)
    unsigned long bitIndex = 0;
    const unsigned long lowerMask =
        static_cast<unsigned long>(mask & 0xFFFFFFFFULL);

    if (_BitScanForward(&bitIndex, lowerMask))
    {
        return static_cast<unsigned int>(bitIndex);
    }

    const unsigned long upperMask =
        static_cast<unsigned long>(mask >> 32U);
    const unsigned char found =
        _BitScanForward(&bitIndex, upperMask);

    MYVOXEL_ASSERT_MESSAGE(found != 0, "Cannot find a set bit in a non-empty surface mask.");
    return static_cast<unsigned int>(bitIndex) + 32U;
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned int>(__builtin_ctzll(mask));
#else
    unsigned int bitIndex = 0;

    while ((mask & static_cast<std::uint64_t>(1ULL)) == 0)
    {
        mask >>= 1U;
        ++bitIndex;
    }

    return bitIndex;
#endif
}

// 返回64位掩码中的置位数量。
std::size_t bitCount(std::uint64_t mask)
{
#if defined(_MSC_VER) && defined(_M_X64)
    return static_cast<std::size_t>(__popcnt64(mask));
#elif defined(_MSC_VER)
    return static_cast<std::size_t>(
        __popcnt(static_cast<unsigned int>(mask & 0xFFFFFFFFULL)) +
        __popcnt(static_cast<unsigned int>(mask >> 32U)));
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<std::size_t>(__builtin_popcountll(mask));
#else
    std::size_t count = 0;

    while (mask != 0)
    {
        mask &= mask - static_cast<std::uint64_t>(1ULL);
        ++count;
    }

    return count;
#endif
}

// 保存一个根内单位面的二维局部坐标和最终颜色。
struct ColoredFaceCell
{
    ColoredFaceCell(std::size_t uValue, std::size_t vValue, const MyVoxel::Geometry::MeshColor& colorValue)
        : u(uValue)
        , v(vValue)
        , color(colorValue)
    {
    }

    std::size_t u; // 当前方向平面内第一坐标。
    std::size_t v; // 当前方向平面内第二坐标。
    MyVoxel::Geometry::MeshColor color; // 当前单位面的最终颜色。
};

// 保存贪心合并二维掩码中的一个单元。
struct FaceMaskCell
{
    FaceMaskCell()
        : active(false)
    {
    }

    bool active; // 当前单位面是否仍未被贪心矩形使用。
    MyVoxel::Geometry::MeshColor color; // 当前单位面的最终颜色。
};

using FacePlane = std::vector<ColoredFaceCell>;
using FacePlaneArray = std::vector<FacePlane>;
using RootFaceMap = std::map<MyVoxel::VoxelCellIndex, MyVoxel::VoxelFaceSet::Container>;

// 返回第0层根展开到最高层后的单轴体素数量。
std::size_t rootAxisCellCount(const MyVoxel::VoxelShape& shape)
{
    MYVOXEL_ASSERT_MESSAGE(shape.grid().maximumLevel() >= MyVoxel::BaseVoxelLevel,
                           "Voxel surface root level exceeds the grid maximum level.");

    const unsigned int difference = static_cast<unsigned int>(shape.grid().maximumLevel() - MyVoxel::BaseVoxelLevel);
    const unsigned int sizeBitCount = static_cast<unsigned int>(sizeof(std::size_t) * 8U);

    MYVOXEL_ASSERT_MESSAGE(difference < sizeBitCount,
                           "Voxel surface root resolution exceeds the supported size_t range.");

    return static_cast<std::size_t>(1) << difference;
}

// 返回正除数下的向下取整整数商。
std::int64_t floorDivide(std::int64_t value, std::int64_t divisor)
{
    MYVOXEL_ASSERT_MESSAGE(divisor > 0, "Voxel surface floor division requires a positive divisor.");

    std::int64_t quotient = value / divisor;
    const std::int64_t remainder = value % divisor;

    if (remainder < 0)
    {
        --quotient;
    }

    return quotient;
}

// 将64位整数转换为VoxelIndex。
MyVoxel::VoxelIndex voxelIndex(std::int64_t value)
{
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(value >= minimum && value <= maximum, "Voxel surface index exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回最高层体素索引所属的第0层根索引。
MyVoxel::VoxelIndex rootIndexFromCellIndex(MyVoxel::VoxelIndex cellIndex, std::int64_t rootScale)
{
    return voxelIndex(floorDivide(static_cast<std::int64_t>(cellIndex), rootScale));
}

// 返回最高层单位面所属的第0层根索引。
MyVoxel::VoxelCellIndex rootIndexOfFace(const MyVoxel::VoxelFaceAddress& face, std::int64_t rootScale)
{
    return MyVoxel::VoxelCellIndex(
        rootIndexFromCellIndex(face.cellIndex.x, rootScale),
        rootIndexFromCellIndex(face.cellIndex.y, rootScale),
        rootIndexFromCellIndex(face.cellIndex.z, rootScale));
}

// 返回第0层根索引在最高层网格中的最小体素坐标。
std::int64_t rootMinimumCoordinate(MyVoxel::VoxelIndex rootIndex, std::int64_t rootScale)
{
    MYVOXEL_ASSERT_MESSAGE(rootScale > 0, "Voxel surface root scale must be positive.");

    const std::int64_t value = static_cast<std::int64_t>(rootIndex);
    const std::int64_t minimum = (std::numeric_limits<std::int64_t>::min)();
    const std::int64_t maximum = (std::numeric_limits<std::int64_t>::max)();

    if (value > 0)
    {
        MYVOXEL_ASSERT_MESSAGE(value <= maximum / rootScale, "Voxel surface root coordinate multiplication overflowed.");
    }
    else if (value < 0)
    {
        MYVOXEL_ASSERT_MESSAGE(value >= minimum / rootScale, "Voxel surface root coordinate multiplication underflowed.");
    }

    return value * rootScale;
}

// 将最高层绝对体素坐标转换为指定根内部局部坐标。
std::size_t rootLocalCoordinate(MyVoxel::VoxelIndex cellIndex, std::int64_t rootMinimum, std::size_t rootCellCount)
{
    const std::int64_t localCoordinate = static_cast<std::int64_t>(cellIndex) - rootMinimum;

    MYVOXEL_ASSERT_MESSAGE(localCoordinate >= 0, "Voxel face lies below its root minimum coordinate.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::uint64_t>(localCoordinate) < static_cast<std::uint64_t>(rootCellCount),
                           "Voxel face lies outside its root maximum coordinate.");

    return static_cast<std::size_t>(localCoordinate);
}

// 返回方向和根内轴向切片对应的平面数组索引。
std::size_t facePlaneIndex(MyVoxel::VoxelFaceDirection direction, std::size_t slice, std::size_t rootCellCount)
{
    MYVOXEL_ASSERT_MESSAGE(MyVoxel::isValidVoxelFaceDirection(direction), "Voxel surface face direction is invalid.");
    MYVOXEL_ASSERT_MESSAGE(slice < rootCellCount, "Voxel surface face slice exceeds the root resolution.");

    return static_cast<std::size_t>(direction) * rootCellCount + slice;
}

// 将一个单位面转换为当前方向平面内的slice、u和v局部坐标。
void facePlaneCoordinates(MyVoxel::VoxelFaceDirection direction,
                          std::size_t localX,
                          std::size_t localY,
                          std::size_t localZ,
                          std::size_t& slice,
                          std::size_t& u,
                          std::size_t& v)
{
    switch (direction)
    {
    case MyVoxel::VoxelFaceDirection::NegativeX:
    case MyVoxel::VoxelFaceDirection::PositiveX:
        slice = localX;
        u = localY;
        v = localZ;
        return;

    case MyVoxel::VoxelFaceDirection::NegativeY:
    case MyVoxel::VoxelFaceDirection::PositiveY:
        slice = localY;
        u = localX;
        v = localZ;
        return;

    case MyVoxel::VoxelFaceDirection::NegativeZ:
    case MyVoxel::VoxelFaceDirection::PositiveZ:
        slice = localZ;
        u = localX;
        v = localY;
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel surface encountered an unknown face direction.");
    slice = 0;
    u = 0;
    v = 0;
}

// 创建最高层网格交点对应的局部空间顶点。
MyVoxel::Geometry::MeshVertex makeVertex(const MyVoxel::VoxelGrid& grid,
                                         std::int64_t x,
                                         std::int64_t y,
                                         std::int64_t z,
                                         double normalX,
                                         double normalY,
                                         double normalZ)
{
    const MyMath::Vector3& origin = grid.origin();
    const double edgeLength = grid.minimumCellEdgeLength();

    return MyVoxel::Geometry::MeshVertex(
        origin.x() + static_cast<double>(x) * edgeLength,
        origin.y() + static_cast<double>(y) * edgeLength,
        origin.z() + static_cast<double>(z) * edgeLength,
        normalX,
        normalY,
        normalZ);
}

// 追加一个已经按颜色贪心合并的矩形面。
void appendMergedFace(MyVoxel::Geometry::Mesh& mesh,
                      const MyVoxel::VoxelGrid& grid,
                      MyVoxel::VoxelFaceDirection direction,
                      std::int64_t fixedCoordinate,
                      std::int64_t u0,
                      std::int64_t v0,
                      std::int64_t u1,
                      std::int64_t v1,
                      const MyVoxel::Geometry::MeshColor& color)
{
    switch (direction)
    {
    case MyVoxel::VoxelFaceDirection::NegativeX:
        mesh.appendQuad(
            makeVertex(grid, fixedCoordinate, u0, v0, -1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u0, v1, -1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u1, v1, -1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u1, v0, -1.0, 0.0, 0.0),
            color);
        return;

    case MyVoxel::VoxelFaceDirection::PositiveX:
        mesh.appendQuad(
            makeVertex(grid, fixedCoordinate, u0, v0, 1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u1, v0, 1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u1, v1, 1.0, 0.0, 0.0),
            makeVertex(grid, fixedCoordinate, u0, v1, 1.0, 0.0, 0.0),
            color);
        return;

    case MyVoxel::VoxelFaceDirection::NegativeY:
        mesh.appendQuad(
            makeVertex(grid, u0, fixedCoordinate, v0, 0.0, -1.0, 0.0),
            makeVertex(grid, u1, fixedCoordinate, v0, 0.0, -1.0, 0.0),
            makeVertex(grid, u1, fixedCoordinate, v1, 0.0, -1.0, 0.0),
            makeVertex(grid, u0, fixedCoordinate, v1, 0.0, -1.0, 0.0),
            color);
        return;

    case MyVoxel::VoxelFaceDirection::PositiveY:
        mesh.appendQuad(
            makeVertex(grid, u0, fixedCoordinate, v0, 0.0, 1.0, 0.0),
            makeVertex(grid, u0, fixedCoordinate, v1, 0.0, 1.0, 0.0),
            makeVertex(grid, u1, fixedCoordinate, v1, 0.0, 1.0, 0.0),
            makeVertex(grid, u1, fixedCoordinate, v0, 0.0, 1.0, 0.0),
            color);
        return;

    case MyVoxel::VoxelFaceDirection::NegativeZ:
        mesh.appendQuad(
            makeVertex(grid, u0, v0, fixedCoordinate, 0.0, 0.0, -1.0),
            makeVertex(grid, u0, v1, fixedCoordinate, 0.0, 0.0, -1.0),
            makeVertex(grid, u1, v1, fixedCoordinate, 0.0, 0.0, -1.0),
            makeVertex(grid, u1, v0, fixedCoordinate, 0.0, 0.0, -1.0),
            color);
        return;

    case MyVoxel::VoxelFaceDirection::PositiveZ:
        mesh.appendQuad(
            makeVertex(grid, u0, v0, fixedCoordinate, 0.0, 0.0, 1.0),
            makeVertex(grid, u1, v0, fixedCoordinate, 0.0, 0.0, 1.0),
            makeVertex(grid, u1, v1, fixedCoordinate, 0.0, 0.0, 1.0),
            makeVertex(grid, u0, v1, fixedCoordinate, 0.0, 0.0, 1.0),
            color);
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel surface encountered an unknown face direction.");
}

// 返回指定方向平面在最高层网格中的固定坐标和两个平面坐标原点。
void mergedPlaneCoordinates(MyVoxel::VoxelFaceDirection direction,
                            std::size_t slice,
                            std::int64_t rootMinimumX,
                            std::int64_t rootMinimumY,
                            std::int64_t rootMinimumZ,
                            std::int64_t& fixedCoordinate,
                            std::int64_t& uOrigin,
                            std::int64_t& vOrigin)
{
    const std::int64_t sliceCoordinate = static_cast<std::int64_t>(slice);

    switch (direction)
    {
    case MyVoxel::VoxelFaceDirection::NegativeX:
        fixedCoordinate = rootMinimumX + sliceCoordinate;
        uOrigin = rootMinimumY;
        vOrigin = rootMinimumZ;
        return;

    case MyVoxel::VoxelFaceDirection::PositiveX:
        fixedCoordinate = rootMinimumX + sliceCoordinate + 1;
        uOrigin = rootMinimumY;
        vOrigin = rootMinimumZ;
        return;

    case MyVoxel::VoxelFaceDirection::NegativeY:
        fixedCoordinate = rootMinimumY + sliceCoordinate;
        uOrigin = rootMinimumX;
        vOrigin = rootMinimumZ;
        return;

    case MyVoxel::VoxelFaceDirection::PositiveY:
        fixedCoordinate = rootMinimumY + sliceCoordinate + 1;
        uOrigin = rootMinimumX;
        vOrigin = rootMinimumZ;
        return;

    case MyVoxel::VoxelFaceDirection::NegativeZ:
        fixedCoordinate = rootMinimumZ + sliceCoordinate;
        uOrigin = rootMinimumX;
        vOrigin = rootMinimumY;
        return;

    case MyVoxel::VoxelFaceDirection::PositiveZ:
        fixedCoordinate = rootMinimumZ + sliceCoordinate + 1;
        uOrigin = rootMinimumX;
        vOrigin = rootMinimumY;
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel surface encountered an unknown face direction.");
    fixedCoordinate = 0;
    uOrigin = 0;
    vOrigin = 0;
}

// 将一个方向平面执行按颜色矩形贪心合并并追加到网格。
std::size_t appendGreedyPlane(MyVoxel::Geometry::Mesh& mesh,
                              const MyVoxel::VoxelGrid& grid,
                              MyVoxel::VoxelFaceDirection direction,
                              std::size_t slice,
                              std::int64_t rootMinimumX,
                              std::int64_t rootMinimumY,
                              std::int64_t rootMinimumZ,
                              std::size_t rootCellCount,
                              const FacePlane& plane,
                              std::vector<FaceMaskCell>& faceMask)
{
    MYVOXEL_ASSERT_MESSAGE(rootCellCount <= (std::numeric_limits<std::size_t>::max)() / rootCellCount,
                           "Voxel surface face mask size overflowed.");

    std::size_t mergedQuadCount = 0;

    const std::size_t planeCellCount = rootCellCount * rootCellCount;

    MYVOXEL_ASSERT_MESSAGE(faceMask.size() == planeCellCount, "Voxel surface face mask size is inconsistent.");

    for (std::size_t maskIndex = 0; maskIndex < faceMask.size(); ++maskIndex)
    {
        faceMask[maskIndex].active = false;
    }

    for (FacePlane::const_iterator iterator = plane.begin(); iterator != plane.end(); ++iterator)
    {
        MYVOXEL_ASSERT_MESSAGE(iterator->u < rootCellCount, "Voxel surface face U coordinate exceeds the root resolution.");
        MYVOXEL_ASSERT_MESSAGE(iterator->v < rootCellCount, "Voxel surface face V coordinate exceeds the root resolution.");

        FaceMaskCell& maskCell = faceMask[iterator->v * rootCellCount + iterator->u];

        MYVOXEL_ASSERT_MESSAGE(!maskCell.active, "Voxel surface face plane contains a duplicate unit face.");

        maskCell.active = true;
        maskCell.color = iterator->color;
    }

    std::int64_t fixedCoordinate = 0;
    std::int64_t uOrigin = 0;
    std::int64_t vOrigin = 0;

    mergedPlaneCoordinates(direction,
                           slice,
                           rootMinimumX,
                           rootMinimumY,
                           rootMinimumZ,
                           fixedCoordinate,
                           uOrigin,
                           vOrigin);

    for (std::size_t v = 0; v < rootCellCount; ++v)
    {
        for (std::size_t u = 0; u < rootCellCount; ++u)
        {
            FaceMaskCell& startCell = faceMask[v * rootCellCount + u];

            if (!startCell.active)
            {
                continue;
            }

            const MyVoxel::Geometry::MeshColor color = startCell.color;
            std::size_t width = 1;

            while (u + width < rootCellCount)
            {
                const FaceMaskCell& nextCell = faceMask[v * rootCellCount + u + width];

                if (!nextCell.active || nextCell.color != color)
                {
                    break;
                }

                ++width;
            }

            std::size_t height = 1;

            while (v + height < rootCellCount)
            {
                bool rowMatches = true;

                for (std::size_t offset = 0; offset < width; ++offset)
                {
                    const FaceMaskCell& nextCell = faceMask[(v + height) * rootCellCount + u + offset];

                    if (!nextCell.active || nextCell.color != color)
                    {
                        rowMatches = false;
                        break;
                    }
                }

                if (!rowMatches)
                {
                    break;
                }

                ++height;
            }

            for (std::size_t offsetV = 0; offsetV < height; ++offsetV)
            {
                for (std::size_t offsetU = 0; offsetU < width; ++offsetU)
                {
                    faceMask[(v + offsetV) * rootCellCount + u + offsetU].active = false;
                }
            }

            const std::int64_t u0 = uOrigin + static_cast<std::int64_t>(u);
            const std::int64_t v0 = vOrigin + static_cast<std::int64_t>(v);
            const std::int64_t u1 = u0 + static_cast<std::int64_t>(width);
            const std::int64_t v1 = v0 + static_cast<std::int64_t>(height);

            appendMergedFace(mesh, grid, direction, fixedCoordinate, u0, v0, u1, v1, color);
            ++mergedQuadCount;
        }
    }

    return mergedQuadCount;
}

// 返回保存一个平面行所需的64位字数量。
std::size_t planeWordCount(std::size_t rootCellCount)
{
    const std::size_t bitsPerWord = 64;
    const std::size_t maximumSize = (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        rootCellCount > 0,
        "Voxel surface root resolution must be positive.");
    MYVOXEL_ASSERT_MESSAGE(
        rootCellCount <= maximumSize - (bitsPerWord - 1),
        "Voxel surface plane word count overflowed.");

    return (rootCellCount + bitsPerWord - 1) / bitsPerWord;
}

// 返回二维活动面位图中的线性字索引。
std::size_t activeWordIndex(
    std::size_t v,
    std::size_t wordIndex,
    std::size_t wordsPerRow)
{
    return v * wordsPerRow + wordIndex;
}

// 判断二维活动面位图中的指定单元是否仍然有效。
bool isActivePlaneCell(
    const std::vector<std::uint64_t>& activeWords,
    std::size_t wordsPerRow,
    std::size_t v,
    std::size_t u)
{
    const std::size_t wordIndex = u >> 6U;
    const unsigned int bitIndex = static_cast<unsigned int>(u & 63U);

    return (activeWords[activeWordIndex(v, wordIndex, wordsPerRow)] &
            (static_cast<std::uint64_t>(1ULL) << bitIndex)) != 0;
}

// 返回闭区间[firstBit, lastBit]均为1的64位掩码。
std::uint64_t bitRangeMask(unsigned int firstBit, unsigned int lastBit)
{
    MYVOXEL_ASSERT_MESSAGE(
        firstBit <= lastBit && lastBit < 64U,
        "Voxel surface bit range is invalid.");

    const std::uint64_t lowerMask =
        lastBit == 63U
            ? ~static_cast<std::uint64_t>(0)
            : (static_cast<std::uint64_t>(1ULL) << (lastBit + 1U)) -
                static_cast<std::uint64_t>(1ULL);

    return lowerMask & (~static_cast<std::uint64_t>(0) << firstBit);
}

// 清除二维活动面位图中同一行的连续区间。
void clearActivePlaneRange(
    std::vector<std::uint64_t>& activeWords,
    std::size_t wordsPerRow,
    std::size_t v,
    std::size_t u,
    std::size_t width)
{
    MYVOXEL_ASSERT_MESSAGE(width > 0, "Voxel surface active range must be non-empty.");

    const std::size_t endU = u + width - 1;
    const std::size_t firstWordIndex = u >> 6U;
    const std::size_t lastWordIndex = endU >> 6U;
    const unsigned int firstBit = static_cast<unsigned int>(u & 63U);
    const unsigned int lastBit = static_cast<unsigned int>(endU & 63U);

    if (firstWordIndex == lastWordIndex)
    {
        activeWords[activeWordIndex(v, firstWordIndex, wordsPerRow)] &=
            ~bitRangeMask(firstBit, lastBit);
        return;
    }

    activeWords[activeWordIndex(v, firstWordIndex, wordsPerRow)] &=
        ~(~static_cast<std::uint64_t>(0) << firstBit);

    for (std::size_t wordIndex = firstWordIndex + 1;
         wordIndex < lastWordIndex;
         ++wordIndex)
    {
        activeWords[activeWordIndex(v, wordIndex, wordsPerRow)] = 0;
    }

    activeWords[activeWordIndex(v, lastWordIndex, wordsPerRow)] &=
        ~bitRangeMask(0U, lastBit);
}

// 将Root中的单位面地址转换为方向切片局部坐标。
void rootFacePlaneCoordinates(
    const MyVoxel::VoxelRootFaceMasks& faceMasks,
    const MyVoxel::VoxelFaceAddress& face,
    std::size_t& slice,
    std::size_t& u,
    std::size_t& v)
{
    MYVOXEL_ASSERT_MESSAGE(
        faceMasks.contains(face),
        "Voxel root surface color references a face outside the Root surface.");

    const std::size_t axisCellCount = faceMasks.axisCellCount();
    const MyVoxel::VoxelCellIndex& rootIndex = faceMasks.rootIndex();
    const std::int64_t rootMinimumX =
        rootMinimumCoordinate(rootIndex.x, static_cast<std::int64_t>(axisCellCount));
    const std::int64_t rootMinimumY =
        rootMinimumCoordinate(rootIndex.y, static_cast<std::int64_t>(axisCellCount));
    const std::int64_t rootMinimumZ =
        rootMinimumCoordinate(rootIndex.z, static_cast<std::int64_t>(axisCellCount));
    const std::int64_t localX =
        static_cast<std::int64_t>(face.cellIndex.x) - rootMinimumX;
    const std::int64_t localY =
        static_cast<std::int64_t>(face.cellIndex.y) - rootMinimumY;
    const std::int64_t localZ =
        static_cast<std::int64_t>(face.cellIndex.z) - rootMinimumZ;

    MYVOXEL_ASSERT_MESSAGE(
        localX >= 0 && localY >= 0 && localZ >= 0 &&
        localX < static_cast<std::int64_t>(axisCellCount) &&
        localY < static_cast<std::int64_t>(axisCellCount) &&
        localZ < static_cast<std::int64_t>(axisCellCount),
        "Voxel root surface color face lies outside the owning Root.");

    switch (face.direction)
    {
    case MyVoxel::VoxelFaceDirection::NegativeX:
    case MyVoxel::VoxelFaceDirection::PositiveX:
        slice = static_cast<std::size_t>(localX);
        u = static_cast<std::size_t>(localY);
        v = static_cast<std::size_t>(localZ);
        return;

    case MyVoxel::VoxelFaceDirection::NegativeY:
    case MyVoxel::VoxelFaceDirection::PositiveY:
        slice = static_cast<std::size_t>(localY);
        u = static_cast<std::size_t>(localX);
        v = static_cast<std::size_t>(localZ);
        return;

    case MyVoxel::VoxelFaceDirection::NegativeZ:
    case MyVoxel::VoxelFaceDirection::PositiveZ:
        slice = static_cast<std::size_t>(localZ);
        u = static_cast<std::size_t>(localX);
        v = static_cast<std::size_t>(localY);
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel root surface color has an invalid face direction.");
    slice = 0;
    u = 0;
    v = 0;
}

// 将Root方向面掩码中的一个切片复制到活动位图，并叠加预先分组的单独颜色。
std::size_t prepareMaskedPlane(
    const MyVoxel::VoxelRootFaceMasks& faceMasks,
    const MyVoxel::VoxelRootSurfaceColorData& colorData,
    MyVoxel::VoxelFaceDirection direction,
    std::size_t slice,
    std::vector<std::uint64_t>& activeWords,
    std::vector<FaceMaskCell>& faceMask)
{
    const std::size_t rootCellCount = faceMasks.axisCellCount();
    const std::size_t wordsPerRow = faceMasks.wordsPerPlaneRow();
    std::size_t faceCount = 0;

    MYVOXEL_ASSERT_MESSAGE(
        colorData.isInitialized(),
        "Voxel surface prepared color data is not initialized.");
    MYVOXEL_ASSERT_MESSAGE(
        activeWords.size() == rootCellCount * wordsPerRow,
        "Voxel surface active plane word count is inconsistent.");
    MYVOXEL_ASSERT_MESSAGE(
        faceMask.size() == rootCellCount * rootCellCount,
        "Voxel surface color mask size is inconsistent.");

    for (std::size_t v = 0; v < rootCellCount; ++v)
    {
        for (std::size_t wordIndex = 0;
             wordIndex < wordsPerRow;
             ++wordIndex)
        {
            const std::uint64_t word =
                faceMasks.planeWord(direction, slice, v, wordIndex);

            activeWords[activeWordIndex(v, wordIndex, wordsPerRow)] = word;
            faceCount += bitCount(word);
        }
    }

    for (MyVoxel::VoxelRootSurfaceColorData::ConstIterator iterator =
             colorData.begin(direction, slice);
         iterator != colorData.end(direction, slice);
         ++iterator)
    {
        MYVOXEL_ASSERT_MESSAGE(
            iterator->u < rootCellCount && iterator->v < rootCellCount,
            "Voxel surface prepared color coordinate exceeds the Root resolution.");
        MYVOXEL_ASSERT_MESSAGE(
            isActivePlaneCell(
                activeWords,
                wordsPerRow,
                iterator->v,
                iterator->u),
            "Voxel surface prepared color references an inactive face.");

        faceMask[iterator->v * rootCellCount + iterator->u].color =
            iterator->color;
    }

    return faceCount;
}

// 将当前切片叠加的单独颜色恢复为默认颜色。
void resetPreparedPlaneColors(
    const MyVoxel::VoxelRootSurfaceColorData& colorData,
    MyVoxel::VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t rootCellCount,
    const MyVoxel::Geometry::MeshColor& defaultColor,
    std::vector<FaceMaskCell>& faceMask)
{
    for (MyVoxel::VoxelRootSurfaceColorData::ConstIterator iterator =
             colorData.begin(direction, slice);
         iterator != colorData.end(direction, slice);
         ++iterator)
    {
        faceMask[iterator->v * rootCellCount + iterator->u].color =
            defaultColor;
    }
}

// 使用活动位行驱动一个方向切片的按颜色矩形贪心合并。
std::size_t appendGreedyMaskedPlane(
    MyVoxel::Geometry::Mesh& mesh,
    const MyVoxel::VoxelGrid& grid,
    MyVoxel::VoxelFaceDirection direction,
    std::size_t slice,
    std::int64_t rootMinimumX,
    std::int64_t rootMinimumY,
    std::int64_t rootMinimumZ,
    std::size_t rootCellCount,
    std::vector<std::uint64_t>& activeWords,
    const std::vector<FaceMaskCell>& faceMask)
{
    const std::size_t wordsPerRow = planeWordCount(rootCellCount);
    std::size_t mergedQuadCount = 0;
    std::int64_t fixedCoordinate = 0;
    std::int64_t uOrigin = 0;
    std::int64_t vOrigin = 0;

    mergedPlaneCoordinates(
        direction,
        slice,
        rootMinimumX,
        rootMinimumY,
        rootMinimumZ,
        fixedCoordinate,
        uOrigin,
        vOrigin);

    for (std::size_t v = 0; v < rootCellCount; ++v)
    {
        for (;;)
        {
            std::size_t firstWordIndex = wordsPerRow;

            for (std::size_t wordIndex = 0;
                 wordIndex < wordsPerRow;
                 ++wordIndex)
            {
                if (activeWords[activeWordIndex(v, wordIndex, wordsPerRow)] != 0)
                {
                    firstWordIndex = wordIndex;
                    break;
                }
            }

            if (firstWordIndex == wordsPerRow)
            {
                break;
            }

            const std::uint64_t firstWord =
                activeWords[activeWordIndex(v, firstWordIndex, wordsPerRow)];
            const unsigned int bitIndex =
                firstSetBitIndex(firstWord);
            const std::size_t u =
                firstWordIndex * static_cast<std::size_t>(64) +
                bitIndex;
            const MyVoxel::Geometry::MeshColor color =
                faceMask[v * rootCellCount + u].color;
            std::size_t width = 1;

            while (u + width < rootCellCount)
            {
                if (!isActivePlaneCell(
                        activeWords,
                        wordsPerRow,
                        v,
                        u + width) ||
                    faceMask[v * rootCellCount + u + width].color != color)
                {
                    break;
                }

                ++width;
            }

            std::size_t height = 1;

            while (v + height < rootCellCount)
            {
                bool rowMatches = true;

                for (std::size_t offset = 0;
                     offset < width;
                     ++offset)
                {
                    if (!isActivePlaneCell(
                            activeWords,
                            wordsPerRow,
                            v + height,
                            u + offset) ||
                        faceMask[(v + height) * rootCellCount + u + offset].color != color)
                    {
                        rowMatches = false;
                        break;
                    }
                }

                if (!rowMatches)
                {
                    break;
                }

                ++height;
            }

            for (std::size_t offsetV = 0;
                 offsetV < height;
                 ++offsetV)
            {
                clearActivePlaneRange(
                    activeWords,
                    wordsPerRow,
                    v + offsetV,
                    u,
                    width);
            }

            const std::int64_t u0 =
                uOrigin + static_cast<std::int64_t>(u);
            const std::int64_t v0 =
                vOrigin + static_cast<std::int64_t>(v);
            const std::int64_t u1 =
                u0 + static_cast<std::int64_t>(width);
            const std::int64_t v1 =
                v0 + static_cast<std::int64_t>(height);

            appendMergedFace(
                mesh,
                grid,
                direction,
                fixedCoordinate,
                u0,
                v0,
                u1,
                v1,
                color);
            ++mergedQuadCount;
        }
    }

    return mergedQuadCount;
}

// 只使用Root方向面掩码中的一个方向构建局部网格。
MyVoxel::Geometry::Mesh buildRootMaskDirectionMesh(
    const MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelCellIndex& rootIndex,
    MyVoxel::VoxelFaceDirection direction,
    const MyVoxel::VoxelRootFaceMasks& faceMasks,
    const MyVoxel::VoxelRootSurfaceColorData& colorData,
    const MyVoxel::Geometry::MeshColor& defaultColor,
    std::size_t reserveQuadCount,
    MyVoxel::VoxelSurfaceMeshingStatistics* statistics)
{
    if (statistics)
    {
        statistics->clear();
    }

    MyVoxel::Geometry::Mesh mesh;

    if (faceMasks.isEmpty())
    {
        return mesh;
    }

    const std::size_t maximumVertexCount =
        static_cast<std::size_t>(
            (std::numeric_limits<std::uint32_t>::max)());
    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        reserveQuadCount <= maximumVertexCount / 4 &&
        reserveQuadCount <= maximumSize / 6,
        "VoxelSurfaceMesher direction reserve capacity overflowed.");

    if (reserveQuadCount > 0)
    {
        mesh.reserve(
            reserveQuadCount * 4,
            reserveQuadCount * 6);
    }

    MYVOXEL_ASSERT_MESSAGE(
        MyVoxel::isValidVoxelFaceDirection(direction),
        "VoxelSurfaceMesher root direction is invalid.");

    const std::size_t rootCellCount = rootAxisCellCount(shape);

    MYVOXEL_ASSERT_MESSAGE(
        faceMasks.isInitialized() &&
        faceMasks.rootIndex() == rootIndex &&
        faceMasks.axisCellCount() == rootCellCount,
        "VoxelSurfaceMesher root face masks do not match the requested root.");

    MYVOXEL_ASSERT_MESSAGE(
        rootCellCount <=
            static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)()),
        "Voxel surface root resolution exceeds int64 range.");
    MYVOXEL_ASSERT_MESSAGE(
        rootCellCount <=
            (std::numeric_limits<std::size_t>::max)() / rootCellCount,
        "Voxel surface face mask size overflowed.");

    const std::int64_t rootScale =
        static_cast<std::int64_t>(rootCellCount);
    const std::int64_t rootMinimumX =
        rootMinimumCoordinate(rootIndex.x, rootScale);
    const std::int64_t rootMinimumY =
        rootMinimumCoordinate(rootIndex.y, rootScale);
    const std::int64_t rootMinimumZ =
        rootMinimumCoordinate(rootIndex.z, rootScale);
    const std::size_t planeCellCount =
        rootCellCount * rootCellCount;
    const std::size_t wordsPerRow =
        faceMasks.wordsPerPlaneRow();

    std::vector<FaceMaskCell> faceMask(planeCellCount);

    for (std::size_t maskIndex = 0; maskIndex < faceMask.size(); ++maskIndex)
    {
        faceMask[maskIndex].color = defaultColor;
    }

    std::vector<std::uint64_t> activeWords(
        rootCellCount * wordsPerRow,
        static_cast<std::uint64_t>(0));
    double planeBuildMilliseconds = 0.0;
    double greedyMergeMilliseconds = 0.0;
    std::size_t sourceFaceCount = 0;
    std::size_t nonEmptyPlaneCount = 0;
    std::size_t mergedQuadCount = 0;

    for (std::size_t slice = 0;
         slice < rootCellCount;
         ++slice)
    {
        MyVoxel::Foundation::Stopwatch planeTimer;
        const std::size_t planeFaceCount = prepareMaskedPlane(
            faceMasks,
            colorData,
            direction,
            slice,
            activeWords,
            faceMask);
        planeBuildMilliseconds +=
            planeTimer.elapsedMilliseconds();
        sourceFaceCount += planeFaceCount;

        if (planeFaceCount == 0)
        {
            continue;
        }

        ++nonEmptyPlaneCount;

        MyVoxel::Foundation::Stopwatch greedyTimer;
        mergedQuadCount += appendGreedyMaskedPlane(
            mesh,
            shape.grid(),
            direction,
            slice,
            rootMinimumX,
            rootMinimumY,
            rootMinimumZ,
            rootCellCount,
            activeWords,
            faceMask);
        greedyMergeMilliseconds +=
            greedyTimer.elapsedMilliseconds();

        resetPreparedPlaneColors(
            colorData,
            direction,
            slice,
            rootCellCount,
            defaultColor,
            faceMask);
    }

    if (statistics)
    {
        statistics->facePlaneBuildMilliseconds =
            planeBuildMilliseconds;
        statistics->greedyMergeMilliseconds =
            greedyMergeMilliseconds;
        statistics->sourceFaceCount =
            sourceFaceCount;
        statistics->nonEmptyPlaneCount =
            nonEmptyPlaneCount;
        statistics->mergedQuadCount =
            mergedQuadCount;
    }

    MYVOXEL_ASSERT_MESSAGE(
        mesh.isValid(),
        "VoxelSurfaceMesher produced an invalid root direction mesh.");

    return mesh;
}

// 直接使用Root方向面掩码构建局部网格。
MyVoxel::Geometry::Mesh buildRootMaskMesh(
    const MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelCellIndex& rootIndex,
    const MyVoxel::VoxelRootFaceMasks& faceMasks,
    const MyVoxel::VoxelFaceColorMap& colors,
    const MyVoxel::Geometry::MeshColor& defaultColor,
    MyVoxel::VoxelSurfaceMeshingStatistics* statistics)
{
    if (statistics)
    {
        statistics->clear();
    }

    MyVoxel::Geometry::Mesh mesh;

    if (faceMasks.isEmpty())
    {
        return mesh;
    }

    MyVoxel::VoxelRootSurfaceColorData colorData;
    colorData.build(faceMasks, colors);

    for (unsigned int directionValue = 0;
         directionValue < MyVoxel::VoxelFaceDirectionCount;
         ++directionValue)
    {
        MyVoxel::VoxelSurfaceMeshingStatistics directionStatistics;
        MyVoxel::Geometry::Mesh directionMesh =
            buildRootMaskDirectionMesh(
                shape,
                rootIndex,
                static_cast<MyVoxel::VoxelFaceDirection>(directionValue),
                faceMasks,
                colorData,
                defaultColor,
                0,
                statistics ? &directionStatistics : nullptr);

        mesh.append(directionMesh);

        if (statistics)
        {
            statistics->add(directionStatistics);
        }
    }

    MYVOXEL_ASSERT_MESSAGE(
        mesh.isValid(),
        "VoxelSurfaceMesher produced an invalid root mask mesh.");

    return mesh;
}

// 构建一个根拥有的全部单位面网格。
MyVoxel::Geometry::Mesh buildRootMesh(const MyVoxel::VoxelShape& shape,
                                      const MyVoxel::VoxelCellIndex& rootIndex,
                                      const MyVoxel::VoxelFaceSet::Container& faces,
                                      const MyVoxel::VoxelFaceColorMap& colors,
                                      const MyVoxel::Geometry::MeshColor& defaultColor,
                                      MyVoxel::VoxelSurfaceMeshingStatistics* statistics)
{
    if (statistics)
    {
        statistics->clear();
        statistics->sourceFaceCount = faces.size();
    }

    MyVoxel::Geometry::Mesh mesh;

    if (faces.empty())
    {
        return mesh;
    }

    const std::size_t rootCellCount = rootAxisCellCount(shape);

    MYVOXEL_ASSERT_MESSAGE(rootCellCount <= static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)()),
                           "Voxel surface root resolution exceeds int64 range.");

    MYVOXEL_ASSERT_MESSAGE(rootCellCount <= (std::numeric_limits<std::size_t>::max)() / MyVoxel::VoxelFaceDirectionCount,
                           "Voxel surface face plane array size overflowed.");

    MYVOXEL_ASSERT_MESSAGE(rootCellCount <= (std::numeric_limits<std::size_t>::max)() / rootCellCount,
                           "Voxel surface face mask size overflowed.");

    const std::int64_t rootScale = static_cast<std::int64_t>(rootCellCount);
    const std::int64_t rootMinimumX = rootMinimumCoordinate(rootIndex.x, rootScale);
    const std::int64_t rootMinimumY = rootMinimumCoordinate(rootIndex.y, rootScale);
    const std::int64_t rootMinimumZ = rootMinimumCoordinate(rootIndex.z, rootScale);
    const std::size_t planeCount = rootCellCount * MyVoxel::VoxelFaceDirectionCount;
    const std::size_t planeCellCount = rootCellCount * rootCellCount;

    MyVoxel::Foundation::Stopwatch facePlaneTimer;
    FacePlaneArray planes(planeCount);

    for (MyVoxel::VoxelFaceSet::Container::const_iterator iterator = faces.begin(); iterator != faces.end(); ++iterator)
    {
        const MyVoxel::VoxelFaceAddress& face = *iterator;

        MYVOXEL_ASSERT_MESSAGE(MyVoxel::isValidVoxelFaceDirection(face.direction), "Voxel surface face direction is invalid.");
        MYVOXEL_ASSERT_MESSAGE(rootIndexOfFace(face, rootScale) == rootIndex,
                               "VoxelSurfaceMesher root input contains a face owned by another root.");

        const std::size_t localX = rootLocalCoordinate(face.cellIndex.x, rootMinimumX, rootCellCount);
        const std::size_t localY = rootLocalCoordinate(face.cellIndex.y, rootMinimumY, rootCellCount);
        const std::size_t localZ = rootLocalCoordinate(face.cellIndex.z, rootMinimumZ, rootCellCount);

        std::size_t slice = 0;
        std::size_t u = 0;
        std::size_t v = 0;

        facePlaneCoordinates(face.direction, localX, localY, localZ, slice, u, v);

        planes[facePlaneIndex(face.direction, slice, rootCellCount)].push_back(
            ColoredFaceCell(u, v, colors.colorOrDefault(face, defaultColor)));
    }

    if (statistics)
    {
        statistics->facePlaneBuildMilliseconds = facePlaneTimer.elapsedMilliseconds();
    }

    MyVoxel::Foundation::Stopwatch greedyTimer;
    std::vector<FaceMaskCell> faceMask(planeCellCount);
    std::size_t nonEmptyPlaneCount = 0;
    std::size_t mergedQuadCount = 0;

    for (unsigned int directionValue = 0; directionValue < MyVoxel::VoxelFaceDirectionCount; ++directionValue)
    {
        const MyVoxel::VoxelFaceDirection direction = static_cast<MyVoxel::VoxelFaceDirection>(directionValue);

        for (std::size_t slice = 0; slice < rootCellCount; ++slice)
        {
            const FacePlane& plane = planes[facePlaneIndex(direction, slice, rootCellCount)];

            if (plane.empty())
            {
                continue;
            }

            ++nonEmptyPlaneCount;
            mergedQuadCount += appendGreedyPlane(mesh,
                                                 shape.grid(),
                                                 direction,
                                                 slice,
                                                 rootMinimumX,
                                                 rootMinimumY,
                                                 rootMinimumZ,
                                                 rootCellCount,
                                                 plane,
                                                 faceMask);
        }
    }

    if (statistics)
    {
        statistics->greedyMergeMilliseconds = greedyTimer.elapsedMilliseconds();
        statistics->nonEmptyPlaneCount = nonEmptyPlaneCount;
        statistics->mergedQuadCount = mergedQuadCount;
    }

    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "VoxelSurfaceMesher produced an invalid root mesh.");
    return mesh;
}

}

namespace MyVoxel
{

VoxelRootSurfaceColorData::Cell::Cell()
    : u(0)
    , v(0)
{
}

VoxelRootSurfaceColorData::Cell::Cell(
    std::size_t uValue,
    std::size_t vValue,
    const Geometry::MeshColor& colorValue)
    : u(uValue)
    , v(vValue)
    , color(colorValue)
{
}

VoxelRootSurfaceColorData::VoxelRootSurfaceColorData()
    : m_axisCellCount(0)
{
}

void VoxelRootSurfaceColorData::build(
    const VoxelRootFaceMasks& faceMasks,
    const VoxelFaceColorMap& colors)
{
    MYVOXEL_ASSERT_MESSAGE(
        faceMasks.isInitialized(),
        "Voxel root surface color data requires initialized face masks.");

    m_axisCellCount = faceMasks.axisCellCount();

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        DirectionData& directionData =
            m_directions[directionValue];

        directionData.cells.clear();
        directionData.sliceOffsets.assign(
            m_axisCellCount + 1,
            static_cast<std::size_t>(0));
    }

    for (VoxelFaceColorMap::ConstIterator iterator = colors.begin();
         iterator != colors.end();
         ++iterator)
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasks.contains(iterator->first),
            "Voxel root surface color map contains a face outside the Root surface.");

        std::size_t slice = 0;
        std::size_t u = 0;
        std::size_t v = 0;

        rootFacePlaneCoordinates(
            faceMasks,
            iterator->first,
            slice,
            u,
            v);

        const unsigned int directionValue =
            static_cast<unsigned int>(iterator->first.direction);

        MYVOXEL_ASSERT_MESSAGE(
            directionValue < VoxelFaceDirectionCount,
            "Voxel root surface color direction is invalid.");

        ++m_directions[directionValue].sliceOffsets[slice + 1];
    }

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        DirectionData& directionData =
            m_directions[directionValue];

        for (std::size_t slice = 0;
             slice < m_axisCellCount;
             ++slice)
        {
            directionData.sliceOffsets[slice + 1] +=
                directionData.sliceOffsets[slice];
        }

        directionData.cells.resize(
            directionData.sliceOffsets[m_axisCellCount]);
    }

    std::vector<std::size_t> writeOffsets(
        VoxelFaceDirectionCount * m_axisCellCount,
        static_cast<std::size_t>(0));

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        for (std::size_t slice = 0;
             slice < m_axisCellCount;
             ++slice)
        {
            writeOffsets[
                directionValue * m_axisCellCount + slice] =
                m_directions[directionValue].sliceOffsets[slice];
        }
    }

    for (VoxelFaceColorMap::ConstIterator iterator = colors.begin();
         iterator != colors.end();
         ++iterator)
    {
        std::size_t slice = 0;
        std::size_t u = 0;
        std::size_t v = 0;

        rootFacePlaneCoordinates(
            faceMasks,
            iterator->first,
            slice,
            u,
            v);

        const unsigned int directionValue =
            static_cast<unsigned int>(iterator->first.direction);
        std::size_t& writeOffset =
            writeOffsets[
                directionValue * m_axisCellCount + slice];

        m_directions[directionValue].cells[writeOffset] =
            Cell(u, v, iterator->second);
        ++writeOffset;
    }
}

bool VoxelRootSurfaceColorData::isInitialized() const
{
    return m_axisCellCount > 0;
}

std::size_t VoxelRootSurfaceColorData::colorCount() const
{
    std::size_t count = 0;

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        count += m_directions[directionValue].cells.size();
    }

    return count;
}

VoxelRootSurfaceColorData::ConstIterator
VoxelRootSurfaceColorData::begin(
    VoxelFaceDirection direction,
    std::size_t slice) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Voxel root surface color data is not initialized.");
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel root surface color direction is invalid.");
    MYVOXEL_ASSERT_MESSAGE(
        slice < m_axisCellCount,
        "Voxel root surface color slice exceeds the Root resolution.");

    const DirectionData& directionData =
        m_directions[static_cast<unsigned int>(direction)];

    return directionData.cells.begin() +
        directionData.sliceOffsets[slice];
}

VoxelRootSurfaceColorData::ConstIterator
VoxelRootSurfaceColorData::end(
    VoxelFaceDirection direction,
    std::size_t slice) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Voxel root surface color data is not initialized.");
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel root surface color direction is invalid.");
    MYVOXEL_ASSERT_MESSAGE(
        slice < m_axisCellCount,
        "Voxel root surface color slice exceeds the Root resolution.");

    const DirectionData& directionData =
        m_directions[static_cast<unsigned int>(direction)];

    return directionData.cells.begin() +
        directionData.sliceOffsets[slice + 1];
}

VoxelSurfaceMeshingStatistics::VoxelSurfaceMeshingStatistics()
{
    clear();
}

void VoxelSurfaceMeshingStatistics::clear()
{
    facePlaneBuildMilliseconds = 0.0;
    greedyMergeMilliseconds = 0.0;
    sourceFaceCount = 0;
    nonEmptyPlaneCount = 0;
    mergedQuadCount = 0;
}

void VoxelSurfaceMeshingStatistics::add(const VoxelSurfaceMeshingStatistics& other)
{
    facePlaneBuildMilliseconds += other.facePlaneBuildMilliseconds;
    greedyMergeMilliseconds += other.greedyMergeMilliseconds;
    sourceFaceCount += other.sourceFaceCount;
    nonEmptyPlaneCount += other.nonEmptyPlaneCount;
    mergedQuadCount += other.mergedQuadCount;
}

Geometry::Mesh VoxelSurfaceMesher::build(const VoxelShape& shape, const Geometry::MeshColor& defaultColor)
{
    const VoxelFaceColorMap colors;
    return build(shape, colors, defaultColor);
}

Geometry::Mesh VoxelSurfaceMesher::build(const VoxelShape& shape,
                                         const VoxelFaceColorMap& colors,
                                         const Geometry::MeshColor& defaultColor)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface meshing requires a valid VoxelShape.");

    const VoxelFaceSet faces = VoxelFaceExtractor::extract(shape);
    return build(shape, faces, colors, defaultColor);
}

Geometry::Mesh VoxelSurfaceMesher::build(const VoxelShape& shape,
                                         const VoxelFaceSet& faces,
                                         const VoxelFaceColorMap& colors,
                                         const Geometry::MeshColor& defaultColor)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface mesh construction requires a valid VoxelShape.");

    Geometry::Mesh mesh;

    if (faces.isEmpty())
    {
        return mesh;
    }

    const std::size_t rootCellCount = rootAxisCellCount(shape);

    MYVOXEL_ASSERT_MESSAGE(rootCellCount <= static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)()),
                           "Voxel surface root resolution exceeds int64 range.");

    const std::int64_t rootScale = static_cast<std::int64_t>(rootCellCount);
    RootFaceMap rootFaces;

    for (VoxelFaceSet::ConstIterator iterator = faces.begin(); iterator != faces.end(); ++iterator)
    {
        rootFaces[rootIndexOfFace(*iterator, rootScale)].push_back(*iterator);
    }

    for (RootFaceMap::const_iterator iterator = rootFaces.begin(); iterator != rootFaces.end(); ++iterator)
    {
        mesh.append(buildRootMesh(shape, iterator->first, iterator->second, colors, defaultColor, nullptr));
    }

    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "VoxelSurfaceMesher produced an invalid complete mesh.");
    return mesh;
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             const Geometry::MeshColor& defaultColor)
{
    const VoxelFaceColorMap colors;
    return buildRoot(shape, rootIndex, colors, defaultColor);
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             const VoxelFaceColorMap& colors,
                                             const Geometry::MeshColor& defaultColor)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Root voxel surface meshing requires a valid VoxelShape.");

    const VoxelRootSurface surface =
        VoxelFaceExtractor::extractRootSurface(shape, rootIndex);

    return buildRoot(
        shape,
        rootIndex,
        surface.faceMasks,
        colors,
        defaultColor);
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             const VoxelFaceSet& faces,
                                             const VoxelFaceColorMap& colors,
                                             const Geometry::MeshColor& defaultColor)
{
    return buildRoot(shape, rootIndex, faces, colors, defaultColor, nullptr);
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             const VoxelFaceSet& faces,
                                             const VoxelFaceColorMap& colors,
                                             const Geometry::MeshColor& defaultColor,
                                             VoxelSurfaceMeshingStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Root voxel surface mesh construction requires a valid VoxelShape.");
    return buildRootMesh(shape, rootIndex, faces.faces(), colors, defaultColor, statistics);
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelFaceColorMap& colors,
    const Geometry::MeshColor& defaultColor)
{
    return buildRoot(
        shape,
        rootIndex,
        faceMasks,
        colors,
        defaultColor,
        nullptr);
}

Geometry::Mesh VoxelSurfaceMesher::buildRoot(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelFaceColorMap& colors,
    const Geometry::MeshColor& defaultColor,
    VoxelSurfaceMeshingStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root voxel surface mask meshing requires a valid VoxelShape.");

    return buildRootMaskMesh(
        shape,
        rootIndex,
        faceMasks,
        colors,
        defaultColor,
        statistics);
}

Geometry::Mesh VoxelSurfaceMesher::buildRootDirection(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelFaceColorMap& colors,
    const Geometry::MeshColor& defaultColor)
{
    return buildRootDirection(
        shape,
        rootIndex,
        direction,
        faceMasks,
        colors,
        defaultColor,
        nullptr);
}

Geometry::Mesh VoxelSurfaceMesher::buildRootDirection(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelFaceColorMap& colors,
    const Geometry::MeshColor& defaultColor,
    VoxelSurfaceMeshingStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root voxel surface direction meshing requires a valid VoxelShape.");

    VoxelRootSurfaceColorData colorData;
    colorData.build(faceMasks, colors);

    return buildRootMaskDirectionMesh(
        shape,
        rootIndex,
        direction,
        faceMasks,
        colorData,
        defaultColor,
        0,
        statistics);
}

Geometry::Mesh VoxelSurfaceMesher::buildRootDirection(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelRootSurfaceColorData& colorData,
    const Geometry::MeshColor& defaultColor)
{
    return buildRootDirection(
        shape,
        rootIndex,
        direction,
        faceMasks,
        colorData,
        defaultColor,
        nullptr);
}

Geometry::Mesh VoxelSurfaceMesher::buildRootDirection(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelRootSurfaceColorData& colorData,
    const Geometry::MeshColor& defaultColor,
    VoxelSurfaceMeshingStatistics* statistics)
{
    return buildRootDirection(
        shape,
        rootIndex,
        direction,
        faceMasks,
        colorData,
        defaultColor,
        0,
        statistics);
}

Geometry::Mesh VoxelSurfaceMesher::buildRootDirection(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction,
    const VoxelRootFaceMasks& faceMasks,
    const VoxelRootSurfaceColorData& colorData,
    const Geometry::MeshColor& defaultColor,
    std::size_t reserveQuadCount,
    VoxelSurfaceMeshingStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root voxel surface prepared-color direction meshing requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        colorData.isInitialized(),
        "Root voxel surface direction meshing requires prepared color data.");

    return buildRootMaskDirectionMesh(
        shape,
        rootIndex,
        direction,
        faceMasks,
        colorData,
        defaultColor,
        reserveQuadCount,
        statistics);
}

}
