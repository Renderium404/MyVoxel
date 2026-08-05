#include "VoxelFaceExtractor.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/ParallelExecutionStatistics.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"
#include "MyVoxel/Foundation/Stopwatch.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Tree/VoxelTreeAccessor.h"

namespace
{

const unsigned int BitsPerWord = 64; // 一个std::uint64_t固定保存64个连续X方向最高层体素的材料状态。
const std::size_t MinimumParallelOccupancyCount = 4; // 至少四个实际源根时启用并行占用展开。
const std::size_t MinimumParallelFaceMaskCount = 4; // 至少四个目标Root时并行生成方向面掩码。

// 返回非零64位掩码中最低置位的位索引。
unsigned int firstSetBitIndex(std::uint64_t mask)
{
    MYVOXEL_ASSERT_MESSAGE(mask != 0, "Cannot query the first set bit of an empty mask.");

#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long bitIndex = 0;
    _BitScanForward64(&bitIndex, mask);
    return static_cast<unsigned int>(bitIndex);
#elif defined(_MSC_VER)
    unsigned long bitIndex = 0;
    const unsigned long lowerMask = static_cast<unsigned long>(mask & 0xFFFFFFFFULL);

    if (_BitScanForward(&bitIndex, lowerMask))
    {
        return static_cast<unsigned int>(bitIndex);
    }

    const unsigned long upperMask = static_cast<unsigned long>(mask >> 32U);
    const unsigned char found = _BitScanForward(&bitIndex, upperMask);

    MYVOXEL_ASSERT_MESSAGE(found != 0, "Cannot find a set bit in a non-empty mask.");
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

// 返回最低置位的位索引，并从原掩码中删除该位。
unsigned int takeFirstSetBitIndex(std::uint64_t& mask)
{
    const unsigned int bitIndex = firstSetBitIndex(mask);
    mask &= mask - static_cast<std::uint64_t>(1ULL);
    return bitIndex;
}

// 返回保存指定数量连续位所需的64位字数量。
std::size_t occupancyWordCount(std::size_t bitCount)
{
    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        bitCount > 0,
        "Root occupancy bit count must be positive.");
    MYVOXEL_ASSERT_MESSAGE(
        bitCount <= maximumSize - static_cast<std::size_t>(BitsPerWord - 1U),
        "Root occupancy row word count overflowed.");

    return (bitCount + static_cast<std::size_t>(BitsPerWord - 1U)) /
           static_cast<std::size_t>(BitsPerWord);
}

// 返回最低位到highestBit均为1的64位掩码。
std::uint64_t lowerBitsMask(unsigned int highestBit)
{
    MYVOXEL_ASSERT_MESSAGE(
        highestBit < BitsPerWord,
        "Root occupancy bit index exceeds one word.");

    if (highestBit == BitsPerWord - 1U)
    {
        return ~static_cast<std::uint64_t>(0);
    }

    return (static_cast<std::uint64_t>(1ULL) << (highestBit + 1U)) -
           static_cast<std::uint64_t>(1ULL);
}

// 返回闭区间[firstBit, lastBit]均为1的64位掩码。
std::uint64_t bitRangeMask(unsigned int firstBit, unsigned int lastBit)
{
    MYVOXEL_ASSERT_MESSAGE(
        firstBit <= lastBit,
        "Root occupancy bit range must be ordered.");
    MYVOXEL_ASSERT_MESSAGE(
        lastBit < BitsPerWord,
        "Root occupancy bit range exceeds one word.");

    return (~static_cast<std::uint64_t>(0) << firstBit) &
           lowerBitsMask(lastBit);
}

// 返回指定层级体素展开到最高层后单轴覆盖的体素数量。
std::int64_t levelScale(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    MYVOXEL_ASSERT_MESSAGE(level <= maximumLevel, "Face extraction level exceeds the maximum voxel level.");

    const unsigned int difference = static_cast<unsigned int>(maximumLevel - level);

    MYVOXEL_ASSERT_MESSAGE(difference < 63, "Face extraction level difference exceeds the supported 64-bit range.");
    return static_cast<std::int64_t>(1) << difference;
}

// 返回第0层根展开到最高层后的单轴体素数量。
std::size_t rootAxisCellCount(const MyVoxel::VoxelShape& shape)
{
    const std::int64_t count = levelScale(MyVoxel::BaseVoxelLevel, shape.grid().maximumLevel());

    MYVOXEL_ASSERT_MESSAGE(count > 0, "Root face extraction resolution must be positive.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::uint64_t>(count) <= static_cast<std::uint64_t>((std::numeric_limits<std::size_t>::max)()),
                           "Root face extraction resolution exceeds size_t range.");

    return static_cast<std::size_t>(count);
}

// 将第0层根索引转换为最高层最小体素索引。
std::int64_t rootMinimumCoordinate(MyVoxel::VoxelIndex rootIndex, std::int64_t rootScale)
{
    MYVOXEL_ASSERT_MESSAGE(rootScale > 0, "Root face extraction scale must be positive.");

    const std::int64_t value = static_cast<std::int64_t>(rootIndex);
    const std::int64_t minimum = (std::numeric_limits<std::int64_t>::min)();
    const std::int64_t maximum = (std::numeric_limits<std::int64_t>::max)();

    if (value > 0)
    {
        MYVOXEL_ASSERT_MESSAGE(value <= maximum / rootScale, "Root face coordinate multiplication overflowed.");
    }
    else if (value < 0)
    {
        MYVOXEL_ASSERT_MESSAGE(value >= minimum / rootScale, "Root face coordinate multiplication underflowed.");
    }

    return value * rootScale;
}

// 尝试为根索引应用指定偏移，索引溢出时返回false。
bool offsetRootIndex(const MyVoxel::VoxelCellIndex& source, int offsetX, int offsetY, int offsetZ, MyVoxel::VoxelCellIndex& result)
{
    const std::int64_t x = static_cast<std::int64_t>(source.x) + static_cast<std::int64_t>(offsetX);
    const std::int64_t y = static_cast<std::int64_t>(source.y) + static_cast<std::int64_t>(offsetY);
    const std::int64_t z = static_cast<std::int64_t>(source.z) + static_cast<std::int64_t>(offsetZ);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    if (x < minimum || x > maximum || y < minimum || y > maximum || z < minimum || z > maximum)
    {
        return false;
    }

    result = MyVoxel::VoxelCellIndex(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y), static_cast<MyVoxel::VoxelIndex>(z));
    return true;
}

// 将一个压缩Material体素覆盖的最高层区域写入所属根的核心占用位图。
void fillMaterialCell(MyVoxel::VoxelRootOccupancy& occupancy,
                      const MyVoxel::VoxelCellAddress& address,
                      MyVoxel::VoxelLevel maximumLevel,
                      std::int64_t rootMinimumX,
                      std::int64_t rootMinimumY,
                      std::int64_t rootMinimumZ)
{
    const std::int64_t scale = levelScale(address.level, maximumLevel);
    const std::int64_t startX = rootMinimumCoordinate(address.index.x, scale);
    const std::int64_t startY = rootMinimumCoordinate(address.index.y, scale);
    const std::int64_t startZ = rootMinimumCoordinate(address.index.z, scale);
    const std::int64_t localStartX = startX - rootMinimumX;
    const std::int64_t localStartY = startY - rootMinimumY;
    const std::int64_t localStartZ = startZ - rootMinimumZ;
    const std::int64_t localEndX = localStartX + scale - 1;
    const std::int64_t localEndY = localStartY + scale - 1;
    const std::int64_t localEndZ = localStartZ + scale - 1;
    const std::int64_t axisCellCount = static_cast<std::int64_t>(occupancy.axisCellCount());

    MYVOXEL_ASSERT_MESSAGE(
        localStartX >= 0 && localStartY >= 0 && localStartZ >= 0 &&
        localEndX < axisCellCount && localEndY < axisCellCount && localEndZ < axisCellCount,
        "Material cell exceeds its owning root occupancy.");

    for (std::int64_t localZ = localStartZ; localZ <= localEndZ; ++localZ)
    {
        for (std::int64_t localY = localStartY; localY <= localEndY; ++localY)
        {
            occupancy.setMaterialXRange(
                static_cast<std::size_t>(localStartX),
                static_cast<std::size_t>(localEndX),
                static_cast<std::size_t>(localY),
                static_cast<std::size_t>(localZ));
        }
    }
}

// 将指定非均匀根中的压缩Material体素写入核心占用位图。
void fillRootOccupancy(const MyVoxel::VoxelShape& shape,
                       const MyVoxel::VoxelCellIndex& rootIndex,
                       MyVoxel::VoxelRootOccupancy& occupancy)
{
    const std::int64_t rootScale = static_cast<std::int64_t>(occupancy.axisCellCount());
    const std::int64_t rootMinimumX = rootMinimumCoordinate(rootIndex.x, rootScale);
    const std::int64_t rootMinimumY = rootMinimumCoordinate(rootIndex.y, rootScale);
    const std::int64_t rootMinimumZ = rootMinimumCoordinate(rootIndex.z, rootScale);

    shape.forest().forEachMaterialCellInRootRange(
        rootIndex,
        rootIndex,
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            fillMaterialCell(
                occupancy,
                address,
                shape.grid().maximumLevel(),
                rootMinimumX,
                rootMinimumY,
                rootMinimumZ);
        });
}

// 保存一个批次中去重后的源根最高层核心占用位图。
class RootOccupancyBatch
{
public:
    using RootIndexSet = std::set<MyVoxel::VoxelCellIndex>;
    using OccupancyMap = std::map<MyVoxel::VoxelCellIndex, MyVoxel::VoxelRootOccupancy>;

    RootOccupancyBatch()
        : m_axisCellCount(0)
    {
    }

    // 构建指定目标根批次所需的全部源根核心占用数据。
    void build(const MyVoxel::VoxelShape& shape,
               const std::vector<MyVoxel::VoxelCellIndex>& targetRootIndices,
               MyVoxel::VoxelFaceExtractionStatistics* statistics)
    {
        m_axisCellCount = rootAxisCellCount(shape);
        m_occupancies.clear();

        RootIndexSet sourceRootIndices;
        std::size_t sourceRequestCount = 0;

        const int offsets[7][3] =
        {
            { 0, 0, 0 },
            { -1, 0, 0 },
            { 1, 0, 0 },
            { 0, -1, 0 },
            { 0, 1, 0 },
            { 0, 0, -1 },
            { 0, 0, 1 }
        }; // 只有存在材料所有者的目标根才需要自身及六个面邻根。

        for (std::size_t targetIndex = 0; targetIndex < targetRootIndices.size(); ++targetIndex)
        {
            const MyVoxel::VoxelCellIndex& targetRootIndex = targetRootIndices[targetIndex];

            if (!shape.forest().getTree(targetRootIndex))
            {
                continue;
            }

            for (unsigned int offsetIndex = 0; offsetIndex < 7; ++offsetIndex)
            {
                MyVoxel::VoxelCellIndex sourceRootIndex;

                if (!offsetRootIndex(
                        targetRootIndex,
                        offsets[offsetIndex][0],
                        offsets[offsetIndex][1],
                        offsets[offsetIndex][2],
                        sourceRootIndex))
                {
                    continue;
                }

                ++sourceRequestCount;
                sourceRootIndices.insert(sourceRootIndex);
            }
        }

        std::vector<MyVoxel::VoxelCellIndex> buildRootIndices;
        std::vector<unsigned char> uniformMaterialFlags;

        buildRootIndices.reserve(sourceRootIndices.size());
        uniformMaterialFlags.reserve(sourceRootIndices.size());

        for (RootIndexSet::const_iterator rootIterator = sourceRootIndices.begin();
             rootIterator != sourceRootIndices.end();
             ++rootIterator)
        {
            const MyVoxel::VoxelTree* tree = shape.forest().getTree(*rootIterator);

            if (!tree)
            {
                continue;
            }

            buildRootIndices.push_back(*rootIterator);
            uniformMaterialFlags.push_back(
                tree->state() == MyVoxel::VoxelState::Material
                    ? static_cast<unsigned char>(1)
                    : static_cast<unsigned char>(0));
        }

        std::vector<std::unique_ptr<MyVoxel::VoxelRootOccupancy> > builtOccupancies(buildRootIndices.size());
        std::vector<double> buildCpuMilliseconds(buildRootIndices.size(), 0.0);

        if (!buildRootIndices.empty())
        {
            MyVoxel::Foundation::ParallelExecutionStatistics parallelStatistics;
            MyVoxel::Foundation::ParallelOptions options;

            options.minimumParallelTaskCount = MinimumParallelOccupancyCount;
            options.statistics = statistics ? &parallelStatistics : nullptr;

            MyVoxel::Foundation::ParallelExecutor::global().execute(
                0,
                buildRootIndices.size(),
                [&](std::size_t blockBegin, std::size_t blockEnd)
                {
                    for (std::size_t taskIndex = blockBegin; taskIndex < blockEnd; ++taskIndex)
                    {
                        MyVoxel::Foundation::Stopwatch taskTimer;
                        const bool uniformMaterial = uniformMaterialFlags[taskIndex] != 0;
                        std::unique_ptr<MyVoxel::VoxelRootOccupancy> occupancy(
                            new MyVoxel::VoxelRootOccupancy(buildRootIndices[taskIndex], m_axisCellCount, uniformMaterial));

                        if (!uniformMaterial)
                        {
                            fillRootOccupancy(shape, buildRootIndices[taskIndex], *occupancy);
                        }

                        buildCpuMilliseconds[taskIndex] = taskTimer.elapsedMilliseconds();
                        builtOccupancies[taskIndex] = std::move(occupancy);
                    }
                },
                options);

            if (statistics)
            {
                ++statistics->occupancyBuildCallCount;
                statistics->occupancyBuildTaskCount += parallelStatistics.taskCount;
                statistics->occupancyBuildParticipantCount += parallelStatistics.workers.size();
                statistics->occupancyBuildBatchSizeTotal += parallelStatistics.batchSize;

                if (parallelStatistics.serialExecution)
                {
                    ++statistics->occupancyBuildSerialCallCount;
                }

                if (parallelStatistics.nestedSerialExecution)
                {
                    ++statistics->occupancyBuildNestedSerialCallCount;
                }

                for (std::size_t taskIndex = 0; taskIndex < buildCpuMilliseconds.size(); ++taskIndex)
                {
                    statistics->occupancyBuildCpuMilliseconds += buildCpuMilliseconds[taskIndex];
                }
            }
        }

        for (std::size_t taskIndex = 0; taskIndex < buildRootIndices.size(); ++taskIndex)
        {
            MYVOXEL_ASSERT_MESSAGE(
                builtOccupancies[taskIndex],
                "Root occupancy build task did not produce an occupancy result.");

            const std::pair<OccupancyMap::iterator, bool> inserted =
                m_occupancies.insert(
                    std::make_pair(
                        buildRootIndices[taskIndex],
                        std::move(*builtOccupancies[taskIndex])));

            MYVOXEL_ASSERT_MESSAGE(inserted.second, "Duplicate root occupancy was built in one batch.");
        }

        if (statistics)
        {
            statistics->occupancySourceRequestCount += sourceRequestCount;
            statistics->uniqueOccupancyRootCount += sourceRootIndices.size();
            statistics->builtOccupancyRootCount += m_occupancies.size();
        }
    }

    // 返回批次中的指定源根占用数据，不存在的空根返回空指针。
    const MyVoxel::VoxelRootOccupancy* find(const MyVoxel::VoxelCellIndex& rootIndex) const
    {
        const OccupancyMap::const_iterator iterator = m_occupancies.find(rootIndex);
        return iterator == m_occupancies.end() ? nullptr : &iterator->second;
    }

    // 返回当前批次根单轴最高层体素数量。
    std::size_t axisCellCount() const
    {
        return m_axisCellCount;
    }

private:
    std::size_t m_axisCellCount; // 当前批次所有根共享的单轴最高层体素数量。
    OccupancyMap m_occupancies; // 去重后的非空源根索引与核心占用位图。
};

// 返回可选根占用中的指定YZ行字，空根统一返回零。
std::uint64_t rowWord(const MyVoxel::VoxelRootOccupancy* occupancy,
                      std::size_t y,
                      std::size_t z,
                      std::size_t wordIndex)
{
    return occupancy ? occupancy->rowWord(y, z, wordIndex) : static_cast<std::uint64_t>(0);
}

// 返回可选根占用中的指定最高层体素是否包含材料，空根统一返回false。
bool isMaterial(const MyVoxel::VoxelRootOccupancy* occupancy, std::size_t x, std::size_t y, std::size_t z)
{
    return occupancy && occupancy->isMaterial(x, y, z);
}

// 返回批次中指定偏移根的核心占用数据，索引溢出或空根返回空指针。
const MyVoxel::VoxelRootOccupancy* offsetOccupancy(const RootOccupancyBatch& batch,
                                     const MyVoxel::VoxelCellIndex& rootIndex,
                                     int offsetX,
                                     int offsetY,
                                     int offsetZ)
{
    MyVoxel::VoxelCellIndex targetRootIndex;

    if (!offsetRootIndex(rootIndex, offsetX, offsetY, offsetZ, targetRootIndex))
    {
        return nullptr;
    }

    return batch.find(targetRootIndex);
}

// 返回缓存映射中的指定Root占用，不存在的空Root返回空指针。
const MyVoxel::VoxelRootOccupancy* findOccupancy(
    const MyVoxel::VoxelRootOccupancyMap& occupancies,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    const MyVoxel::VoxelRootOccupancyMap::const_iterator iterator =
        occupancies.find(rootIndex);

    return iterator == occupancies.end()
        ? nullptr
        : &iterator->second;
}

// 返回缓存映射中指定偏移Root的占用，索引溢出或空Root返回空指针。
const MyVoxel::VoxelRootOccupancy* offsetOccupancy(
    const MyVoxel::VoxelRootOccupancyMap& occupancies,
    const MyVoxel::VoxelCellIndex& rootIndex,
    int offsetX,
    int offsetY,
    int offsetZ)
{
    MyVoxel::VoxelCellIndex targetRootIndex;

    if (!offsetRootIndex(
            rootIndex,
            offsetX,
            offsetY,
            offsetZ,
            targetRootIndex))
    {
        return nullptr;
    }

    return findOccupancy(occupancies, targetRootIndex);
}

// 返回批次中指定Root及其六个面邻Root的只读占用。
MyVoxel::VoxelRootOccupancyNeighborhood rootOccupancyNeighborhood(
    const RootOccupancyBatch& batch,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    return MyVoxel::VoxelRootOccupancyNeighborhood(
        batch.find(rootIndex),
        offsetOccupancy(batch, rootIndex, -1, 0, 0),
        offsetOccupancy(batch, rootIndex, 1, 0, 0),
        offsetOccupancy(batch, rootIndex, 0, -1, 0),
        offsetOccupancy(batch, rootIndex, 0, 1, 0),
        offsetOccupancy(batch, rootIndex, 0, 0, -1),
        offsetOccupancy(batch, rootIndex, 0, 0, 1));
}

// 返回缓存映射中指定Root及其六个面邻Root的只读占用。
MyVoxel::VoxelRootOccupancyNeighborhood rootOccupancyNeighborhood(
    const MyVoxel::VoxelRootOccupancyMap& occupancies,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    return MyVoxel::VoxelRootOccupancyNeighborhood(
        findOccupancy(occupancies, rootIndex),
        offsetOccupancy(occupancies, rootIndex, -1, 0, 0),
        offsetOccupancy(occupancies, rootIndex, 1, 0, 0),
        offsetOccupancy(occupancies, rootIndex, 0, -1, 0),
        offsetOccupancy(occupancies, rootIndex, 0, 1, 0),
        offsetOccupancy(occupancies, rootIndex, 0, 0, -1),
        offsetOccupancy(occupancies, rootIndex, 0, 0, 1));
}

// 将同一XYZ拥有者布局中的六方向掩码写入按方向切片组织的Root面掩码。
void storeFaceMaskWord(MyVoxel::VoxelRootFaceMasks& rootMasks,
                       const MyVoxel::VoxelFaceMaskWord& faces,
                       std::size_t localY,
                       std::size_t localZ,
                       std::size_t wordIndex)
{
    rootMasks.setPlaneWord(
        MyVoxel::VoxelFaceDirection::NegativeY,
        localY,
        localZ,
        wordIndex,
        faces.negativeY);
    rootMasks.setPlaneWord(
        MyVoxel::VoxelFaceDirection::PositiveY,
        localY,
        localZ,
        wordIndex,
        faces.positiveY);
    rootMasks.setPlaneWord(
        MyVoxel::VoxelFaceDirection::NegativeZ,
        localZ,
        localY,
        wordIndex,
        faces.negativeZ);
    rootMasks.setPlaneWord(
        MyVoxel::VoxelFaceDirection::PositiveZ,
        localZ,
        localY,
        wordIndex,
        faces.positiveZ);

    std::uint64_t negativeX = faces.negativeX;

    while (negativeX != 0)
    {
        const unsigned int bitIndex = takeFirstSetBitIndex(negativeX);
        const std::size_t localX =
            wordIndex * static_cast<std::size_t>(BitsPerWord) + bitIndex;

        rootMasks.setFace(
            localX,
            localY,
            localZ,
            MyVoxel::VoxelFaceDirection::NegativeX);
    }

    std::uint64_t positiveX = faces.positiveX;

    while (positiveX != 0)
    {
        const unsigned int bitIndex = takeFirstSetBitIndex(positiveX);
        const std::size_t localX =
            wordIndex * static_cast<std::size_t>(BitsPerWord) + bitIndex;

        rootMasks.setFace(
            localX,
            localY,
            localZ,
            MyVoxel::VoxelFaceDirection::PositiveX);
    }
}

// 提取指定根拥有的方向面掩码。
MyVoxel::VoxelRootFaceMasks extractRootMaskData(
    const RootOccupancyBatch& batch,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    return MyVoxel::VoxelFaceExtractor::buildRootFaceMasks(
        rootIndex,
        batch.axisCellCount(),
        rootOccupancyNeighborhood(batch, rootIndex));
}

// 根据已缓存占用提取指定Root拥有的方向面掩码。
MyVoxel::VoxelRootFaceMasks extractRootMaskData(
    const MyVoxel::VoxelRootOccupancyMap& occupancies,
    const MyVoxel::VoxelCellIndex& rootIndex,
    std::size_t axisCellCount)
{
    return MyVoxel::VoxelFaceExtractor::buildRootFaceMasks(
        rootIndex,
        axisCellCount,
        rootOccupancyNeighborhood(occupancies, rootIndex));
}


// 将终止Material游标覆盖的局部立方体写入叶块材料掩码。
void fillLeafMaskCube(std::uint64_t& materialMask,
                      unsigned int originX,
                      unsigned int originY,
                      unsigned int originZ,
                      unsigned int cellScale)
{
    for (unsigned int localZ = 0; localZ < cellScale; ++localZ)
    {
        for (unsigned int localY = 0; localY < cellScale; ++localY)
        {
            for (unsigned int localX = 0; localX < cellScale; ++localX)
            {
                materialMask |=
                    MyVoxel::leafCellBit(
                        originX + localX,
                        originY + localY,
                        originZ + localZ);
            }
        }
    }
}

// 将当前游标在剩余层级内的材料状态转换为4×4×4叶块掩码。
void appendCursorLeafMask(const MyVoxel::VoxelTreeCursor& cursor,
                          unsigned int remainingLevelCount,
                          unsigned int originX,
                          unsigned int originY,
                          unsigned int originZ,
                          unsigned int cellScale,
                          std::uint64_t& materialMask)
{
    if (cursor.isEmpty())
    {
        return;
    }

    if (cursor.isMaterial())
    {
        fillLeafMaskCube(
            materialMask,
            originX,
            originY,
            originZ,
            cellScale);
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(
        remainingLevelCount > 0 && cellScale >= 2,
        "Subdivided voxel leaf cursor requires remaining child levels.");

    const unsigned int childScale = cellScale >> 1U;

    for (unsigned int cornerValue = 0;
         cornerValue < MyVoxel::VoxelCornerCount;
         ++cornerValue)
    {
        const MyVoxel::VoxelCorner corner =
            static_cast<MyVoxel::VoxelCorner>(cornerValue);
        const unsigned int childOriginX =
            originX + ((cornerValue & 1U) != 0 ? childScale : 0U);
        const unsigned int childOriginY =
            originY + ((cornerValue & 2U) != 0 ? childScale : 0U);
        const unsigned int childOriginZ =
            originZ + ((cornerValue & 4U) != 0 ? childScale : 0U);

        appendCursorLeafMask(
            cursor.child(corner),
            remainingLevelCount - 1U,
            childOriginX,
            childOriginY,
            childOriginZ,
            childScale,
            materialMask);
    }
}

// 返回指定掩码叶块地址在当前体素树中的64位材料状态。
std::uint64_t leafBlockMaterialMask(MyVoxel::VoxelTreeAccessor& accessor,
                                    const MyVoxel::VoxelCellAddress& blockAddress,
                                    MyVoxel::VoxelLevel maximumLevel)
{
    const MyVoxel::VoxelState state =
        accessor.seek(blockAddress);
    const unsigned int remainingLevelCount =
        static_cast<unsigned int>(maximumLevel - blockAddress.level);
    const unsigned int blockCellScale =
        static_cast<unsigned int>(1U << remainingLevelCount);

    MYVOXEL_ASSERT_MESSAGE(
        remainingLevelCount <= 2U &&
        blockCellScale <= MyVoxel::VoxelLeafAxisCellCount,
        "Incremental occupancy leaf block exceeds the 4x4x4 mask layout.");

    if (state == MyVoxel::VoxelState::Empty)
    {
        return MyVoxel::EmptyVoxelLeafMask;
    }

    if (state == MyVoxel::VoxelState::Material)
    {
        std::uint64_t materialMask =
            MyVoxel::EmptyVoxelLeafMask;

        fillLeafMaskCube(
            materialMask,
            0,
            0,
            0,
            blockCellScale);
        return materialMask;
    }

    const MyVoxel::VoxelTreeCursor& cursor =
        accessor.cursor();

    if (remainingLevelCount == 2U &&
        accessor.isExact() &&
        cursor.hasMaterialMask())
    {
        return cursor.materialMask();
    }

    std::uint64_t materialMask =
        MyVoxel::EmptyVoxelLeafMask;

    appendCursorLeafMask(
        cursor,
        remainingLevelCount,
        0,
        0,
        0,
        blockCellScale,
        materialMask);

    return materialMask;
}

// 返回指定Root局部掩码叶块在目标层级中的全局索引。
MyVoxel::VoxelIndex globalBlockCoordinate(
    MyVoxel::VoxelIndex rootIndex,
    std::size_t axisBlockCount,
    std::size_t localIndex)
{
    const std::int64_t value =
        static_cast<std::int64_t>(rootIndex) *
            static_cast<std::int64_t>(axisBlockCount) +
        static_cast<std::int64_t>(localIndex);
    const std::int64_t minimum =
        static_cast<std::int64_t>(
            (std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum =
        static_cast<std::int64_t>(
            (std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(
        value >= minimum && value <= maximum,
        "Incremental occupancy block index exceeds VoxelIndex range.");

    return static_cast<MyVoxel::VoxelIndex>(value);
}

}

namespace MyVoxel
{

VoxelRootOccupancy::VoxelRootOccupancy()
    : m_axisCellCount(0)
    , m_wordsPerRow(0)
    , m_uniformMaterial(false)
{
}

VoxelRootOccupancy::VoxelRootOccupancy(
    const VoxelCellIndex& rootIndexValue,
    std::size_t axisCellCountValue,
    bool uniformMaterialValue)
    : m_rootIndex(rootIndexValue)
    , m_axisCellCount(axisCellCountValue)
    , m_wordsPerRow(occupancyWordCount(axisCellCountValue))
    , m_uniformMaterial(uniformMaterialValue)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_axisCellCount > 0,
        "Root occupancy resolution must be positive.");

    if (m_uniformMaterial)
    {
        return;
    }

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        m_axisCellCount <= maximumSize / m_axisCellCount,
        "Root occupancy row count overflowed.");

    const std::size_t rowCount =
        m_axisCellCount * m_axisCellCount;

    MYVOXEL_ASSERT_MESSAGE(
        rowCount <= maximumSize / m_wordsPerRow,
        "Root occupancy word count overflowed.");

    m_words.assign(
        rowCount * m_wordsPerRow,
        static_cast<std::uint64_t>(0));
}

VoxelRootOccupancy::VoxelRootOccupancy(VoxelRootOccupancy&& other)
    : m_rootIndex(other.m_rootIndex)
    , m_axisCellCount(other.m_axisCellCount)
    , m_wordsPerRow(other.m_wordsPerRow)
    , m_uniformMaterial(other.m_uniformMaterial)
    , m_words(std::move(other.m_words))
{
    other.m_axisCellCount = 0;
    other.m_wordsPerRow = 0;
    other.m_uniformMaterial = false;
}

VoxelRootOccupancy& VoxelRootOccupancy::operator=(VoxelRootOccupancy&& other)
{
    if (this != &other)
    {
        m_rootIndex = other.m_rootIndex;
        m_axisCellCount = other.m_axisCellCount;
        m_wordsPerRow = other.m_wordsPerRow;
        m_uniformMaterial = other.m_uniformMaterial;
        m_words = std::move(other.m_words);

        other.m_axisCellCount = 0;
        other.m_wordsPerRow = 0;
        other.m_uniformMaterial = false;
    }

    return *this;
}

bool VoxelRootOccupancy::isInitialized() const
{
    return m_axisCellCount > 0;
}

bool VoxelRootOccupancy::isUniformMaterial() const
{
    return isInitialized() && m_uniformMaterial;
}

const VoxelCellIndex& VoxelRootOccupancy::rootIndex() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Root occupancy is not initialized.");
    return m_rootIndex;
}

std::size_t VoxelRootOccupancy::axisCellCount() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Root occupancy is not initialized.");
    return m_axisCellCount;
}

std::size_t VoxelRootOccupancy::wordsPerRow() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Root occupancy is not initialized.");
    return m_wordsPerRow;
}

std::uint64_t VoxelRootOccupancy::validWordMask(
    std::size_t wordIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(
        wordIndex < m_wordsPerRow,
        "Root occupancy word index exceeds one row.");

    if (wordIndex + 1 < m_wordsPerRow)
    {
        return ~static_cast<std::uint64_t>(0);
    }

    const unsigned int usedBitCount =
        static_cast<unsigned int>(
            m_axisCellCount -
            wordIndex * static_cast<std::size_t>(BitsPerWord));

    MYVOXEL_ASSERT_MESSAGE(
        usedBitCount > 0 && usedBitCount <= BitsPerWord,
        "Root occupancy final word bit count is invalid.");

    if (usedBitCount == BitsPerWord)
    {
        return ~static_cast<std::uint64_t>(0);
    }

    return (static_cast<std::uint64_t>(1ULL) << usedBitCount) -
           static_cast<std::uint64_t>(1ULL);
}

std::uint64_t VoxelRootOccupancy::rowWord(
    std::size_t y,
    std::size_t z,
    std::size_t wordIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(
        y < m_axisCellCount,
        "Root occupancy Y coordinate exceeds the root.");
    MYVOXEL_ASSERT_MESSAGE(
        z < m_axisCellCount,
        "Root occupancy Z coordinate exceeds the root.");
    MYVOXEL_ASSERT_MESSAGE(
        wordIndex < m_wordsPerRow,
        "Root occupancy word index exceeds one row.");

    if (m_uniformMaterial)
    {
        return validWordMask(wordIndex);
    }

    return m_words[rowWordIndex(y, z, wordIndex)];
}

bool VoxelRootOccupancy::isMaterial(
    std::size_t x,
    std::size_t y,
    std::size_t z) const
{
    MYVOXEL_ASSERT_MESSAGE(
        x < m_axisCellCount &&
        y < m_axisCellCount &&
        z < m_axisCellCount,
        "Root occupancy coordinate exceeds the root.");

    if (m_uniformMaterial)
    {
        return true;
    }

    const std::size_t wordIndex = x >> 6U;
    const unsigned int bitIndex =
        static_cast<unsigned int>(x & 63U);

    return (m_words[rowWordIndex(y, z, wordIndex)] &
            (static_cast<std::uint64_t>(1ULL) << bitIndex)) != 0;
}

void VoxelRootOccupancy::materializeUniformStorage()
{
    if (!m_uniformMaterial)
    {
        return;
    }

    const std::size_t rowCount =
        m_axisCellCount * m_axisCellCount;

    m_words.resize(rowCount * m_wordsPerRow);

    for (std::size_t rowIndex = 0;
         rowIndex < rowCount;
         ++rowIndex)
    {
        for (std::size_t wordIndex = 0;
             wordIndex < m_wordsPerRow;
             ++wordIndex)
        {
            m_words[rowIndex * m_wordsPerRow + wordIndex] =
                validWordMask(wordIndex);
        }
    }

    m_uniformMaterial = false;
}

void VoxelRootOccupancy::setMaterialXRange(
    std::size_t minX,
    std::size_t maxX,
    std::size_t y,
    std::size_t z)
{
    MYVOXEL_ASSERT_MESSAGE(
        !m_uniformMaterial,
        "Uniform material root occupancy does not require explicit writes.");
    MYVOXEL_ASSERT_MESSAGE(
        minX <= maxX && maxX < m_axisCellCount,
        "Root occupancy X range exceeds the root.");
    MYVOXEL_ASSERT_MESSAGE(
        y < m_axisCellCount && z < m_axisCellCount,
        "Root occupancy row coordinate exceeds the root.");

    const std::size_t firstWordOffset = minX >> 6U;
    const std::size_t lastWordOffset = maxX >> 6U;
    const unsigned int firstBit =
        static_cast<unsigned int>(minX & 63U);
    const unsigned int lastBit =
        static_cast<unsigned int>(maxX & 63U);
    const std::size_t rowStartIndex =
        rowWordIndex(y, z, 0);

    if (firstWordOffset == lastWordOffset)
    {
        m_words[rowStartIndex + firstWordOffset] |=
            bitRangeMask(firstBit, lastBit);
        return;
    }

    m_words[rowStartIndex + firstWordOffset] |=
        ~static_cast<std::uint64_t>(0) << firstBit;

    for (std::size_t wordOffset = firstWordOffset + 1;
         wordOffset < lastWordOffset;
         ++wordOffset)
    {
        m_words[rowStartIndex + wordOffset] =
            ~static_cast<std::uint64_t>(0);
    }

    m_words[rowStartIndex + lastWordOffset] |=
        lowerBitsMask(lastBit);
}

void VoxelRootOccupancy::replaceLeafBlock(
    std::size_t localBlockX,
    std::size_t localBlockY,
    std::size_t localBlockZ,
    std::size_t blockCellScale,
    std::uint64_t materialMask)
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Root occupancy leaf replacement requires initialized storage.");
    MYVOXEL_ASSERT_MESSAGE(
        blockCellScale > 0 &&
        blockCellScale <= VoxelLeafAxisCellCount,
        "Root occupancy leaf scale is invalid.");

    const std::size_t startX =
        localBlockX * blockCellScale;
    const std::size_t startY =
        localBlockY * blockCellScale;
    const std::size_t startZ =
        localBlockZ * blockCellScale;

    MYVOXEL_ASSERT_MESSAGE(
        startX + blockCellScale <= m_axisCellCount &&
        startY + blockCellScale <= m_axisCellCount &&
        startZ + blockCellScale <= m_axisCellCount,
        "Root occupancy leaf replacement exceeds the root.");

    if (m_uniformMaterial)
    {
        bool remainsFull = true;

        for (unsigned int localZ = 0;
             localZ < blockCellScale && remainsFull;
             ++localZ)
        {
            for (unsigned int localY = 0;
                 localY < blockCellScale && remainsFull;
                 ++localY)
            {
                for (unsigned int localX = 0;
                     localX < blockCellScale;
                     ++localX)
                {
                    if (!leafCellMaterial(
                            materialMask,
                            localX,
                            localY,
                            localZ))
                    {
                        remainsFull = false;
                        break;
                    }
                }
            }
        }

        if (remainsFull)
        {
            return;
        }

        materializeUniformStorage();
    }

    for (unsigned int localZ = 0;
         localZ < blockCellScale;
         ++localZ)
    {
        for (unsigned int localY = 0;
             localY < blockCellScale;
             ++localY)
        {
            const std::size_t y = startY + localY;
            const std::size_t z = startZ + localZ;

            for (unsigned int localX = 0;
                 localX < blockCellScale;
                 ++localX)
            {
                const std::size_t x = startX + localX;
                const std::size_t wordIndex = x >> 6U;
                const unsigned int bitIndex =
                    static_cast<unsigned int>(x & 63U);
                const std::uint64_t bit =
                    static_cast<std::uint64_t>(1ULL) << bitIndex;
                std::uint64_t& word =
                    m_words[rowWordIndex(y, z, wordIndex)];

                if (leafCellMaterial(
                        materialMask,
                        localX,
                        localY,
                        localZ))
                {
                    word |= bit;
                }
                else
                {
                    word &= ~bit;
                }
            }
        }
    }
}

std::size_t VoxelRootOccupancy::rowWordIndex(
    std::size_t y,
    std::size_t z,
    std::size_t wordIndex) const
{
    return (z * m_axisCellCount + y) *
               m_wordsPerRow +
           wordIndex;
}

VoxelRootOccupancyNeighborhood::VoxelRootOccupancyNeighborhood()
    : current(nullptr)
    , negativeX(nullptr)
    , positiveX(nullptr)
    , negativeY(nullptr)
    , positiveY(nullptr)
    , negativeZ(nullptr)
    , positiveZ(nullptr)
{
}

VoxelRootOccupancyNeighborhood::VoxelRootOccupancyNeighborhood(
    const VoxelRootOccupancy* currentOccupancy,
    const VoxelRootOccupancy* negativeXOccupancy,
    const VoxelRootOccupancy* positiveXOccupancy,
    const VoxelRootOccupancy* negativeYOccupancy,
    const VoxelRootOccupancy* positiveYOccupancy,
    const VoxelRootOccupancy* negativeZOccupancy,
    const VoxelRootOccupancy* positiveZOccupancy)
    : current(currentOccupancy)
    , negativeX(negativeXOccupancy)
    , positiveX(positiveXOccupancy)
    , negativeY(negativeYOccupancy)
    , positiveY(positiveYOccupancy)
    , negativeZ(negativeZOccupancy)
    , positiveZ(positiveZOccupancy)
{
}

VoxelFaceMaskWord::VoxelFaceMaskWord()
    : negativeX(0)
    , positiveX(0)
    , negativeY(0)
    , positiveY(0)
    , negativeZ(0)
    , positiveZ(0)
{
}

std::uint64_t VoxelFaceMaskWord::exposedCells() const
{
    return negativeX |
           positiveX |
           negativeY |
           positiveY |
           negativeZ |
           positiveZ;
}

VoxelFaceMaskWord VoxelFaceExtractor::buildFaceMaskWord(
    const VoxelRootOccupancyNeighborhood& neighborhood,
    std::size_t localY,
    std::size_t localZ,
    std::size_t wordIndex)
{
    VoxelFaceMaskWord faces;

    MYVOXEL_ASSERT_MESSAGE(
        neighborhood.current,
        "Root face mask word requires a material owner Root.");

    const VoxelRootOccupancy& currentOccupancy =
        *neighborhood.current;
    const std::size_t axisCellCount =
        currentOccupancy.axisCellCount();
    const std::size_t wordsPerRow =
        currentOccupancy.wordsPerRow();
    const std::uint64_t current =
        currentOccupancy.rowWord(
            localY,
            localZ,
            wordIndex);
    const std::uint64_t rootMaterial =
        current &
        currentOccupancy.validWordMask(wordIndex);

    if (rootMaterial == 0)
    {
        return faces;
    }

    const std::uint64_t previousWord =
        wordIndex > 0
            ? currentOccupancy.rowWord(
                localY,
                localZ,
                wordIndex - 1)
            : static_cast<std::uint64_t>(0);
    const std::uint64_t nextWord =
        wordIndex + 1 < wordsPerRow
            ? currentOccupancy.rowWord(
                localY,
                localZ,
                wordIndex + 1)
            : static_cast<std::uint64_t>(0);

    std::uint64_t negativeXNeighbors =
        (current << 1U) |
        (previousWord >> 63U);
    std::uint64_t positiveXNeighbors =
        (current >> 1U) |
        (nextWord << 63U);

    if (wordIndex == 0 &&
        isMaterial(
            neighborhood.negativeX,
            axisCellCount - 1,
            localY,
            localZ))
    {
        negativeXNeighbors |=
            static_cast<std::uint64_t>(1ULL);
    }

    if (wordIndex + 1 == wordsPerRow &&
        isMaterial(
            neighborhood.positiveX,
            0,
            localY,
            localZ))
    {
        const unsigned int finalBitIndex =
            static_cast<unsigned int>(
                (axisCellCount - 1) &
                static_cast<std::size_t>(63U));

        positiveXNeighbors |=
            static_cast<std::uint64_t>(1ULL) <<
            finalBitIndex;
    }

    const std::uint64_t negativeYNeighbors =
        localY > 0
            ? currentOccupancy.rowWord(
                localY - 1,
                localZ,
                wordIndex)
            : rowWord(
                neighborhood.negativeY,
                axisCellCount - 1,
                localZ,
                wordIndex);
    const std::uint64_t positiveYNeighbors =
        localY + 1 < axisCellCount
            ? currentOccupancy.rowWord(
                localY + 1,
                localZ,
                wordIndex)
            : rowWord(
                neighborhood.positiveY,
                0,
                localZ,
                wordIndex);
    const std::uint64_t negativeZNeighbors =
        localZ > 0
            ? currentOccupancy.rowWord(
                localY,
                localZ - 1,
                wordIndex)
            : rowWord(
                neighborhood.negativeZ,
                localY,
                axisCellCount - 1,
                wordIndex);
    const std::uint64_t positiveZNeighbors =
        localZ + 1 < axisCellCount
            ? currentOccupancy.rowWord(
                localY,
                localZ + 1,
                wordIndex)
            : rowWord(
                neighborhood.positiveZ,
                localY,
                0,
                wordIndex);

    faces.negativeX =
        rootMaterial &
        ~negativeXNeighbors;
    faces.positiveX =
        rootMaterial &
        ~positiveXNeighbors;
    faces.negativeY =
        rootMaterial &
        ~negativeYNeighbors;
    faces.positiveY =
        rootMaterial &
        ~positiveYNeighbors;
    faces.negativeZ =
        rootMaterial &
        ~negativeZNeighbors;
    faces.positiveZ =
        rootMaterial &
        ~positiveZNeighbors;

    return faces;
}

VoxelRootFaceMasks VoxelFaceExtractor::buildRootFaceMasks(
    const VoxelCellIndex& rootIndex,
    std::size_t axisCellCount,
    const VoxelRootOccupancyNeighborhood& neighborhood)
{
    VoxelRootFaceMasks rootMasks(
        rootIndex,
        axisCellCount);

    if (!neighborhood.current)
    {
        return rootMasks;
    }

    MYVOXEL_ASSERT_MESSAGE(
        neighborhood.current->axisCellCount() ==
            axisCellCount,
        "Root face mask neighborhood resolution is invalid.");

    const std::size_t wordsPerRow =
        neighborhood.current->wordsPerRow();

    for (std::size_t localY = 0;
         localY < axisCellCount;
         ++localY)
    {
        for (std::size_t localZ = 0;
             localZ < axisCellCount;
             ++localZ)
        {
            for (std::size_t wordIndex = 0;
                 wordIndex < wordsPerRow;
                 ++wordIndex)
            {
                const VoxelFaceMaskWord faces =
                    buildFaceMaskWord(
                        neighborhood,
                        localY,
                        localZ,
                        wordIndex);

                storeFaceMaskWord(
                    rootMasks,
                    faces,
                    localY,
                    localZ,
                    wordIndex);
            }
        }
    }

    return rootMasks;
}

VoxelRootSurface::VoxelRootSurface()
{
}

VoxelRootSurface::VoxelRootSurface(VoxelRootSurface&& other)
    : faceMasks(std::move(other.faceMasks))
    , faces(std::move(other.faces))
{
}

VoxelRootSurface& VoxelRootSurface::operator=(VoxelRootSurface&& other)
{
    if (this != &other)
    {
        faceMasks = std::move(other.faceMasks);
        faces = std::move(other.faces);
    }

    return *this;
}

VoxelFaceExtractionStatistics::VoxelFaceExtractionStatistics()
{
    clear();
}

void VoxelFaceExtractionStatistics::clear()
{
    occupancyBuildMilliseconds = 0.0;
    occupancyBuildCpuMilliseconds = 0.0;
    faceGenerationMilliseconds = 0.0;
    faceGenerationCpuMilliseconds = 0.0;
    faceSetBuildMilliseconds = 0.0;

    occupancySourceRequestCount = 0;
    uniqueOccupancyRootCount = 0;
    builtOccupancyRootCount = 0;
    occupancyBuildCallCount = 0;
    occupancyBuildTaskCount = 0;
    occupancyBuildParticipantCount = 0;
    occupancyBuildBatchSizeTotal = 0;
    occupancyBuildSerialCallCount = 0;
    occupancyBuildNestedSerialCallCount = 0;
    faceGenerationCallCount = 0;
    faceGenerationTaskCount = 0;
    faceGenerationParticipantCount = 0;
    faceGenerationBatchSizeTotal = 0;
    faceGenerationSerialCallCount = 0;
    faceGenerationNestedSerialCallCount = 0;
    emittedFaceCount = 0;
}

void VoxelFaceExtractionStatistics::add(const VoxelFaceExtractionStatistics& other)
{
    occupancyBuildMilliseconds += other.occupancyBuildMilliseconds;
    occupancyBuildCpuMilliseconds += other.occupancyBuildCpuMilliseconds;
    faceGenerationMilliseconds += other.faceGenerationMilliseconds;
    faceGenerationCpuMilliseconds += other.faceGenerationCpuMilliseconds;
    faceSetBuildMilliseconds += other.faceSetBuildMilliseconds;

    occupancySourceRequestCount += other.occupancySourceRequestCount;
    uniqueOccupancyRootCount += other.uniqueOccupancyRootCount;
    builtOccupancyRootCount += other.builtOccupancyRootCount;
    occupancyBuildCallCount += other.occupancyBuildCallCount;
    occupancyBuildTaskCount += other.occupancyBuildTaskCount;
    occupancyBuildParticipantCount += other.occupancyBuildParticipantCount;
    occupancyBuildBatchSizeTotal += other.occupancyBuildBatchSizeTotal;
    occupancyBuildSerialCallCount += other.occupancyBuildSerialCallCount;
    occupancyBuildNestedSerialCallCount += other.occupancyBuildNestedSerialCallCount;
    faceGenerationCallCount += other.faceGenerationCallCount;
    faceGenerationTaskCount += other.faceGenerationTaskCount;
    faceGenerationParticipantCount += other.faceGenerationParticipantCount;
    faceGenerationBatchSizeTotal += other.faceGenerationBatchSizeTotal;
    faceGenerationSerialCallCount += other.faceGenerationSerialCallCount;
    faceGenerationNestedSerialCallCount += other.faceGenerationNestedSerialCallCount;
    emittedFaceCount += other.emittedFaceCount;
}

bool VoxelFaceExtractor::extractRootOccupancy(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelRootOccupancy& occupancy)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root occupancy extraction requires a valid VoxelShape.");

    const VoxelTree* tree =
        shape.forest().getTree(rootIndex);

    if (!tree)
    {
        occupancy = VoxelRootOccupancy();
        return false;
    }

    const bool uniformMaterial =
        tree->state() == VoxelState::Material;
    VoxelRootOccupancy result(
        rootIndex,
        rootAxisCellCount(shape),
        uniformMaterial);

    if (!uniformMaterial)
    {
        fillRootOccupancy(
            shape,
            rootIndex,
            result);
    }

    occupancy = std::move(result);
    return true;
}

void VoxelFaceExtractor::extractRootOccupancies(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    VoxelRootOccupancyMap& occupancies)
{
    extractRootOccupancies(
        shape,
        rootIndices,
        occupancies,
        nullptr);
}

void VoxelFaceExtractor::extractRootOccupancies(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    VoxelRootOccupancyMap& occupancies,
    VoxelFaceExtractionStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root occupancy extraction requires a valid VoxelShape.");

    occupancies.clear();

    if (rootIndices.empty())
    {
        return;
    }

    const std::size_t axisCellCount =
        rootAxisCellCount(shape);
    std::vector<VoxelCellIndex> buildRootIndices;
    std::vector<unsigned char> uniformMaterialFlags;

    buildRootIndices.reserve(rootIndices.size());
    uniformMaterialFlags.reserve(rootIndices.size());

    for (std::size_t rootPosition = 0;
         rootPosition < rootIndices.size();
         ++rootPosition)
    {
        const VoxelTree* tree =
            shape.forest().getTree(rootIndices[rootPosition]);

        if (!tree)
        {
            continue;
        }

        buildRootIndices.push_back(rootIndices[rootPosition]);
        uniformMaterialFlags.push_back(
            tree->state() == VoxelState::Material
                ? static_cast<unsigned char>(1)
                : static_cast<unsigned char>(0));
    }

    std::vector<VoxelRootOccupancy> builtOccupancies(
        buildRootIndices.size());
    std::vector<double> buildCpuMilliseconds(
        buildRootIndices.size(),
        0.0);
    Foundation::ParallelExecutionStatistics parallelStatistics;
    Foundation::ParallelOptions options;

    options.minimumParallelTaskCount =
        MinimumParallelOccupancyCount;
    options.statistics =
        statistics ? &parallelStatistics : nullptr;

    Foundation::Stopwatch wallTimer;

    Foundation::ParallelExecutor::global().execute(
        0,
        buildRootIndices.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t taskIndex = blockBegin;
                 taskIndex < blockEnd;
                 ++taskIndex)
            {
                Foundation::Stopwatch taskTimer;
                const bool uniformMaterial =
                    uniformMaterialFlags[taskIndex] != 0;
                VoxelRootOccupancy occupancy(
                    buildRootIndices[taskIndex],
                    axisCellCount,
                    uniformMaterial);

                if (!uniformMaterial)
                {
                    fillRootOccupancy(
                        shape,
                        buildRootIndices[taskIndex],
                        occupancy);
                }

                buildCpuMilliseconds[taskIndex] =
                    taskTimer.elapsedMilliseconds();
                builtOccupancies[taskIndex] =
                    std::move(occupancy);
            }
        },
        options);

    for (std::size_t taskIndex = 0;
         taskIndex < buildRootIndices.size();
         ++taskIndex)
    {
        const std::pair<VoxelRootOccupancyMap::iterator, bool> inserted =
            occupancies.insert(
                std::make_pair(
                    buildRootIndices[taskIndex],
                    std::move(builtOccupancies[taskIndex])));

        MYVOXEL_ASSERT_MESSAGE(
            inserted.second,
            "Duplicate Root occupancy was extracted.");
    }

    if (!statistics)
    {
        return;
    }

    statistics->occupancyBuildMilliseconds +=
        wallTimer.elapsedMilliseconds();
    statistics->uniqueOccupancyRootCount +=
        rootIndices.size();
    statistics->builtOccupancyRootCount +=
        occupancies.size();
    ++statistics->occupancyBuildCallCount;
    statistics->occupancyBuildTaskCount +=
        parallelStatistics.taskCount;
    statistics->occupancyBuildParticipantCount +=
        parallelStatistics.workers.size();
    statistics->occupancyBuildBatchSizeTotal +=
        parallelStatistics.batchSize;

    if (parallelStatistics.serialExecution)
    {
        ++statistics->occupancyBuildSerialCallCount;
    }

    if (parallelStatistics.nestedSerialExecution)
    {
        ++statistics->occupancyBuildNestedSerialCallCount;
    }

    for (std::size_t taskIndex = 0;
         taskIndex < buildCpuMilliseconds.size();
         ++taskIndex)
    {
        statistics->occupancyBuildCpuMilliseconds +=
            buildCpuMilliseconds[taskIndex];
    }
}

void VoxelFaceExtractor::updateRootOccupancy(
    const VoxelShape& shape,
    const VoxelChangeSet::DirtyCellRegion& dirtyRegion,
    VoxelRootOccupancy& occupancy)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Incremental Root occupancy update requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        dirtyRegion.axisCellCount() > 0 &&
        occupancy.isInitialized(),
        "Incremental Root occupancy update requires initialized data.");
    MYVOXEL_ASSERT_MESSAGE(
        dirtyRegion.rootIndex() == occupancy.rootIndex(),
        "Incremental Root occupancy update requires the same Root.");
    MYVOXEL_ASSERT_MESSAGE(
        dirtyRegion.trackingLevel() <= shape.grid().maximumLevel(),
        "Incremental Root occupancy block level exceeds the grid.");

    const VoxelTree* tree =
        shape.forest().getTree(dirtyRegion.rootIndex());

    MYVOXEL_ASSERT_MESSAGE(
        tree,
        "Incremental Root occupancy update requires an existing Root tree.");

    const unsigned int remainingLevelCount =
        static_cast<unsigned int>(
            shape.grid().maximumLevel() -
            dirtyRegion.trackingLevel());
    const std::size_t blockCellScale =
        static_cast<std::size_t>(1) << remainingLevelCount;

    MYVOXEL_ASSERT_MESSAGE(
        blockCellScale <= VoxelLeafAxisCellCount &&
        dirtyRegion.axisCellCount() * blockCellScale ==
            occupancy.axisCellCount(),
        "Incremental Root occupancy block layout does not match the cache.");

    const VoxelCellAddress rootAddress(
        dirtyRegion.rootIndex(),
        BaseVoxelLevel);
    VoxelTreeAccessor accessor(
        *tree,
        rootAddress,
        shape.grid().maximumLevel());
    std::vector<std::size_t> localBlockIndices;

    localBlockIndices.reserve(
        dirtyRegion.changedCellCount());
    dirtyRegion.appendChangedLocalCellIndices(
        localBlockIndices);

    for (std::size_t blockPosition = 0;
         blockPosition < localBlockIndices.size();
         ++blockPosition)
    {
        const std::size_t bitIndex =
            localBlockIndices[blockPosition];
        const std::size_t localBlockX =
            bitIndex % dirtyRegion.axisCellCount();
        const std::size_t planeIndex =
            bitIndex / dirtyRegion.axisCellCount();
        const std::size_t localBlockY =
            planeIndex % dirtyRegion.axisCellCount();
        const std::size_t localBlockZ =
            planeIndex / dirtyRegion.axisCellCount();
        const VoxelCellAddress blockAddress(
            VoxelCellIndex(
                globalBlockCoordinate(
                    dirtyRegion.rootIndex().x,
                    dirtyRegion.axisCellCount(),
                    localBlockX),
                globalBlockCoordinate(
                    dirtyRegion.rootIndex().y,
                    dirtyRegion.axisCellCount(),
                    localBlockY),
                globalBlockCoordinate(
                    dirtyRegion.rootIndex().z,
                    dirtyRegion.axisCellCount(),
                    localBlockZ)),
            dirtyRegion.trackingLevel());
        const std::uint64_t materialMask =
            leafBlockMaterialMask(
                accessor,
                blockAddress,
                shape.grid().maximumLevel());

        occupancy.replaceLeafBlock(
            localBlockX,
            localBlockY,
            localBlockZ,
            blockCellScale,
            materialMask);
    }
}

void VoxelFaceExtractor::buildRootMasks(
    const VoxelRootOccupancyMap& occupancies,
    const std::vector<VoxelCellIndex>& rootIndices,
    std::vector<VoxelRootFaceMasks>& faceMasks,
    VoxelFaceExtractionStatistics* statistics)
{
    faceMasks.clear();
    faceMasks.resize(rootIndices.size());

    if (rootIndices.empty())
    {
        return;
    }

    std::size_t axisCellCount = 0;

    if (!occupancies.empty())
    {
        axisCellCount =
            occupancies.begin()->second.axisCellCount();
    }

    MYVOXEL_ASSERT_MESSAGE(
        axisCellCount > 0,
        "Root mask generation requires at least one initialized occupancy.");

    std::vector<double> cpuMilliseconds(
        statistics ? rootIndices.size() : 0,
        0.0);
    std::vector<std::size_t> emittedFaceCounts(
        statistics ? rootIndices.size() : 0,
        0);
    Foundation::ParallelExecutionStatistics parallelStatistics;
    Foundation::ParallelOptions options;

    options.minimumParallelTaskCount =
        MinimumParallelFaceMaskCount;
    options.statistics =
        statistics ? &parallelStatistics : nullptr;

    Foundation::Stopwatch wallTimer;

    Foundation::ParallelExecutor::global().execute(
        0,
        rootIndices.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t rootPosition = blockBegin;
                 rootPosition < blockEnd;
                 ++rootPosition)
            {
                if (!statistics)
                {
                    faceMasks[rootPosition] =
                        extractRootMaskData(
                            occupancies,
                            rootIndices[rootPosition],
                            axisCellCount);
                    continue;
                }

                Foundation::Stopwatch taskTimer;

                faceMasks[rootPosition] =
                    extractRootMaskData(
                        occupancies,
                        rootIndices[rootPosition],
                        axisCellCount);
                cpuMilliseconds[rootPosition] =
                    taskTimer.elapsedMilliseconds();
                emittedFaceCounts[rootPosition] =
                    faceMasks[rootPosition].faceCount();
            }
        },
        options);

    if (!statistics)
    {
        return;
    }

    statistics->faceGenerationMilliseconds +=
        wallTimer.elapsedMilliseconds();
    ++statistics->faceGenerationCallCount;
    statistics->faceGenerationTaskCount +=
        parallelStatistics.taskCount;
    statistics->faceGenerationParticipantCount +=
        parallelStatistics.workers.size();
    statistics->faceGenerationBatchSizeTotal +=
        parallelStatistics.batchSize;

    if (parallelStatistics.serialExecution)
    {
        ++statistics->faceGenerationSerialCallCount;
    }

    if (parallelStatistics.nestedSerialExecution)
    {
        ++statistics->faceGenerationNestedSerialCallCount;
    }

    for (std::size_t rootPosition = 0;
         rootPosition < rootIndices.size();
         ++rootPosition)
    {
        statistics->faceGenerationCpuMilliseconds +=
            cpuMilliseconds[rootPosition];
        statistics->emittedFaceCount +=
            emittedFaceCounts[rootPosition];
    }
}

VoxelFaceSet VoxelFaceExtractor::extract(const VoxelShape& shape)
{
    return extract(shape, nullptr);
}

VoxelFaceSet VoxelFaceExtractor::extract(const VoxelShape& shape,
                                         VoxelFaceExtractionStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel face extraction requires a valid VoxelShape.");

    if (statistics)
    {
        statistics->clear();
    }

    std::vector<VoxelCellIndex> rootIndices;

    shape.forest().forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            rootIndices.push_back(rootAddress.index);
        });

    std::vector<VoxelFaceSet> rootFaceSets;
    VoxelFaceExtractionStatistics batchStatistics;

    extractRoots(
        shape,
        rootIndices,
        rootFaceSets,
        statistics ? &batchStatistics : nullptr);

    if (statistics)
    {
        statistics->add(batchStatistics);
    }

    VoxelFaceSet::Container faces;
    std::size_t faceCount = 0;

    for (std::size_t rootPosition = 0; rootPosition < rootFaceSets.size(); ++rootPosition)
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceCount <= (std::numeric_limits<std::size_t>::max)() - rootFaceSets[rootPosition].size(),
            "Complete voxel face array size overflowed.");

        faceCount += rootFaceSets[rootPosition].size();
    }

    faces.reserve(faceCount);

    for (std::size_t rootPosition = 0; rootPosition < rootFaceSets.size(); ++rootPosition)
    {
        const VoxelFaceSet::Container& rootFaces = rootFaceSets[rootPosition].faces();

        for (VoxelFaceSet::Container::const_iterator faceIterator = rootFaces.begin();
             faceIterator != rootFaces.end();
             ++faceIterator)
        {
            faces.push_back(*faceIterator);
        }
    }

    Foundation::Stopwatch faceSetTimer;
    VoxelFaceSet result(std::move(faces));

    if (statistics)
    {
        statistics->faceSetBuildMilliseconds += faceSetTimer.elapsedMilliseconds();
    }

    return result;
}

VoxelFaceSet VoxelFaceExtractor::extractRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex)
{
    return extractRoot(shape, rootIndex, nullptr);
}

VoxelFaceSet VoxelFaceExtractor::extractRoot(const VoxelShape& shape,
                                             const VoxelCellIndex& rootIndex,
                                             VoxelFaceExtractionStatistics* statistics)
{
    VoxelRootSurface surface =
        extractRootSurface(shape, rootIndex, statistics);

    return std::move(surface.faces);
}

void VoxelFaceExtractor::extractRoots(const VoxelShape& shape,
                                      const std::vector<VoxelCellIndex>& rootIndices,
                                      std::vector<VoxelFaceSet>& faceSets)
{
    extractRoots(shape, rootIndices, faceSets, nullptr);
}

void VoxelFaceExtractor::extractRoots(const VoxelShape& shape,
                                      const std::vector<VoxelCellIndex>& rootIndices,
                                      std::vector<VoxelFaceSet>& faceSets,
                                      VoxelFaceExtractionStatistics* statistics)
{
    std::vector<VoxelRootSurface> surfaces;

    extractRootSurfaces(shape, rootIndices, surfaces, statistics);

    faceSets.clear();
    faceSets.resize(surfaces.size());

    for (std::size_t rootPosition = 0;
         rootPosition < surfaces.size();
         ++rootPosition)
    {
        faceSets[rootPosition] = std::move(surfaces[rootPosition].faces);
    }
}

VoxelRootFaceMasks VoxelFaceExtractor::extractRootMasks(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex)
{
    return extractRootMasks(shape, rootIndex, nullptr);
}

VoxelRootFaceMasks VoxelFaceExtractor::extractRootMasks(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceExtractionStatistics* statistics)
{
    std::vector<VoxelCellIndex> rootIndices(1, rootIndex);
    std::vector<VoxelRootFaceMasks> faceMasks;

    extractRootMasks(
        shape,
        rootIndices,
        faceMasks,
        statistics);

    MYVOXEL_ASSERT_MESSAGE(
        faceMasks.size() == 1,
        "Single root face mask extraction returned an invalid result count.");

    return std::move(faceMasks[0]);
}

void VoxelFaceExtractor::extractRootMasks(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    std::vector<VoxelRootFaceMasks>& faceMasks)
{
    extractRootMasks(
        shape,
        rootIndices,
        faceMasks,
        nullptr);
}

void VoxelFaceExtractor::extractRootMasks(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    std::vector<VoxelRootFaceMasks>& faceMasks,
    VoxelFaceExtractionStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Root voxel face mask extraction requires a valid VoxelShape.");

    if (statistics)
    {
        statistics->clear();
    }

    faceMasks.clear();
    faceMasks.resize(rootIndices.size());

    if (rootIndices.empty())
    {
        return;
    }

    Foundation::Stopwatch occupancyTimer;
    RootOccupancyBatch occupancyBatch;
    occupancyBatch.build(
        shape,
        rootIndices,
        statistics);

    if (statistics)
    {
        statistics->occupancyBuildMilliseconds +=
            occupancyTimer.elapsedMilliseconds();
    }

    Foundation::Stopwatch faceGenerationWallTimer;
    std::vector<double> faceGenerationCpuMilliseconds;
    std::vector<std::size_t> emittedFaceCounts;

    if (statistics)
    {
        faceGenerationCpuMilliseconds.assign(rootIndices.size(), 0.0);
        emittedFaceCounts.assign(rootIndices.size(), 0);
    }

    Foundation::ParallelExecutionStatistics parallelStatistics;
    Foundation::ParallelOptions options;

    options.minimumParallelTaskCount = MinimumParallelFaceMaskCount;
    options.statistics = statistics ? &parallelStatistics : nullptr;

    Foundation::ParallelExecutor::global().execute(
        0,
        rootIndices.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t rootPosition = blockBegin;
                 rootPosition < blockEnd;
                 ++rootPosition)
            {
                if (!statistics)
                {
                    faceMasks[rootPosition] =
                        extractRootMaskData(
                            occupancyBatch,
                            rootIndices[rootPosition]);
                    continue;
                }

                Foundation::Stopwatch taskTimer;

                faceMasks[rootPosition] =
                    extractRootMaskData(
                        occupancyBatch,
                        rootIndices[rootPosition]);

                faceGenerationCpuMilliseconds[rootPosition] =
                    taskTimer.elapsedMilliseconds();
                emittedFaceCounts[rootPosition] =
                    faceMasks[rootPosition].faceCount();
            }
        },
        options);

    if (statistics)
    {
        statistics->faceGenerationMilliseconds +=
            faceGenerationWallTimer.elapsedMilliseconds();
        ++statistics->faceGenerationCallCount;
        statistics->faceGenerationTaskCount += parallelStatistics.taskCount;
        statistics->faceGenerationParticipantCount += parallelStatistics.workers.size();
        statistics->faceGenerationBatchSizeTotal += parallelStatistics.batchSize;

        if (parallelStatistics.serialExecution)
        {
            ++statistics->faceGenerationSerialCallCount;
        }

        if (parallelStatistics.nestedSerialExecution)
        {
            ++statistics->faceGenerationNestedSerialCallCount;
        }

        for (std::size_t rootPosition = 0;
             rootPosition < rootIndices.size();
             ++rootPosition)
        {
            statistics->faceGenerationCpuMilliseconds +=
                faceGenerationCpuMilliseconds[rootPosition];
            statistics->emittedFaceCount +=
                emittedFaceCounts[rootPosition];
        }
    }
}

VoxelRootSurface VoxelFaceExtractor::extractRootSurface(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex)
{
    return extractRootSurface(shape, rootIndex, nullptr);
}

VoxelRootSurface VoxelFaceExtractor::extractRootSurface(
    const VoxelShape& shape,
    const VoxelCellIndex& rootIndex,
    VoxelFaceExtractionStatistics* statistics)
{
    std::vector<VoxelCellIndex> rootIndices(1, rootIndex);
    std::vector<VoxelRootSurface> surfaces;

    extractRootSurfaces(
        shape,
        rootIndices,
        surfaces,
        statistics);

    MYVOXEL_ASSERT_MESSAGE(
        surfaces.size() == 1,
        "Single root surface extraction returned an invalid result count.");

    return std::move(surfaces[0]);
}

void VoxelFaceExtractor::extractRootSurfaces(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    std::vector<VoxelRootSurface>& surfaces)
{
    extractRootSurfaces(
        shape,
        rootIndices,
        surfaces,
        nullptr);
}

void VoxelFaceExtractor::extractRootSurfaces(
    const VoxelShape& shape,
    const std::vector<VoxelCellIndex>& rootIndices,
    std::vector<VoxelRootSurface>& surfaces,
    VoxelFaceExtractionStatistics* statistics)
{
    std::vector<VoxelRootFaceMasks> faceMasks;

    extractRootMasks(
        shape,
        rootIndices,
        faceMasks,
        statistics);

    surfaces.clear();
    surfaces.resize(faceMasks.size());

    for (std::size_t rootPosition = 0;
         rootPosition < faceMasks.size();
         ++rootPosition)
    {
        surfaces[rootPosition].faceMasks =
            std::move(faceMasks[rootPosition]);

        Foundation::Stopwatch faceSetTimer;
        surfaces[rootPosition].faces =
            surfaces[rootPosition].faceMasks.toFaceSet();

        if (statistics)
        {
            statistics->faceSetBuildMilliseconds +=
                faceSetTimer.elapsedMilliseconds();
        }
    }
}

}