#include "VoxelSurfaceCache.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/ParallelExecutionStatistics.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"
#include "MyVoxel/Foundation/Stopwatch.h"

#include "VoxelFaceExtractor.h"
#include "VoxelSurfaceMesher.h"

namespace
{

const std::size_t MinimumParallelRootDirectionTaskCount = 4; // 一个非空Root固定产生六个方向任务，至少四个方向任务时启用并行。
const std::size_t MinimumParallelRootDifferenceTaskCount = 2; // 至少两个变化Root时并行生成新增和删除面地址。
const std::size_t DirectionMeshReserveGrowthDivisor = 8; // 历史四边形数量额外预留12.5%的增长空间。
const std::size_t DirectionMeshReserveMinimumSlack = 16; // 历史提示非零时至少额外预留十六个四边形。
const std::size_t IncrementalFaceMaskFullRebuildPercent = 25; // 扩展Dirty Leaf覆盖达到Root的25%时改用完整字级面掩码构建。

// 根据上一次实际四边形数量计算本次方向临时Mesh的保守预留容量。
std::size_t directionMeshReserveQuadCount(std::size_t previousQuadCount)
{
    if (previousQuadCount == 0)
    {
        return 0;
    }

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();
    const std::size_t maximumVertexCount =
        static_cast<std::size_t>(
            (std::numeric_limits<std::uint32_t>::max)());
    const std::size_t maximumQuadCount =
        (std::min)(
            maximumSize / 6,
            maximumVertexCount / 4);
    const std::size_t growth =
        previousQuadCount / DirectionMeshReserveGrowthDivisor;

    if (previousQuadCount >= maximumQuadCount)
    {
        return maximumQuadCount;
    }

    std::size_t reserveQuadCount =
        previousQuadCount;

    if (growth > maximumQuadCount - reserveQuadCount)
    {
        return maximumQuadCount;
    }

    reserveQuadCount += growth;

    if (DirectionMeshReserveMinimumSlack >
        maximumQuadCount - reserveQuadCount)
    {
        return maximumQuadCount;
    }

    reserveQuadCount +=
        DirectionMeshReserveMinimumSlack;

    return reserveQuadCount;
}

// 推进方向网格版本；极端溢出时回到1，0始终保留为从未构建。
void advanceDirectionMeshVersion(std::uint64_t& version)
{
    if (version == (std::numeric_limits<std::uint64_t>::max)())
    {
        version = static_cast<std::uint64_t>(1);
    }
    else
    {
        ++version;
    }
}

// 尝试为根索引应用指定偏移，任一分量溢出时返回false。
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

    result = MyVoxel::VoxelCellIndex(
        static_cast<MyVoxel::VoxelIndex>(x),
        static_cast<MyVoxel::VoxelIndex>(y),
        static_cast<MyVoxel::VoxelIndex>(z));

    return true;
}

// 保存一个修改Root准备完成、但尚未提交的核心占用结果。
struct PendingRootOccupancyChange
{
    PendingRootOccupancyChange()
        : hasOccupancy(false)
        , incremental(false)
        , cpuMilliseconds(0.0)
    {
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前占用变化所属的第0层Root索引。
    bool hasOccupancy; // 更新后Root是否仍然存在材料占用。
    bool incremental; // 是否从旧缓存复制并只更新变化掩码叶块。
    MyVoxel::VoxelRootOccupancy occupancy; // 更新后待提交的Root核心占用。
    double cpuMilliseconds; // 当前Root独立准备占用的CPU耗时。
};

using PendingRootOccupancyPositionMap =
    std::map<MyVoxel::VoxelCellIndex, std::size_t>;

// 返回旧缓存和待提交变化共同表示的当前Root占用。
const MyVoxel::VoxelRootOccupancy* currentRootOccupancy(
    const MyVoxel::VoxelRootOccupancyMap& oldOccupancies,
    const std::vector<PendingRootOccupancyChange>& changes,
    const PendingRootOccupancyPositionMap& changePositions,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    const PendingRootOccupancyPositionMap::const_iterator changeIterator =
        changePositions.find(rootIndex);

    if (changeIterator != changePositions.end())
    {
        const PendingRootOccupancyChange& change =
            changes[changeIterator->second];

        return change.hasOccupancy
            ? &change.occupancy
            : nullptr;
    }

    const MyVoxel::VoxelRootOccupancyMap::const_iterator oldIterator =
        oldOccupancies.find(rootIndex);

    return oldIterator == oldOccupancies.end()
        ? nullptr
        : &oldIterator->second;
}

// 返回旧缓存和待提交占用共同表示的当前Root六邻域。
MyVoxel::VoxelRootOccupancyNeighborhood currentRootOccupancyNeighborhood(
    const MyVoxel::VoxelRootOccupancyMap& oldOccupancies,
    const std::vector<PendingRootOccupancyChange>& changes,
    const PendingRootOccupancyPositionMap& changePositions,
    const MyVoxel::VoxelCellIndex& rootIndex)
{
    const MyVoxel::VoxelRootOccupancy* negativeX = nullptr;
    const MyVoxel::VoxelRootOccupancy* positiveX = nullptr;
    const MyVoxel::VoxelRootOccupancy* negativeY = nullptr;
    const MyVoxel::VoxelRootOccupancy* positiveY = nullptr;
    const MyVoxel::VoxelRootOccupancy* negativeZ = nullptr;
    const MyVoxel::VoxelRootOccupancy* positiveZ = nullptr;
    MyVoxel::VoxelCellIndex neighborIndex;

    if (offsetRootIndex(rootIndex, -1, 0, 0, neighborIndex))
    {
        negativeX = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    if (offsetRootIndex(rootIndex, 1, 0, 0, neighborIndex))
    {
        positiveX = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    if (offsetRootIndex(rootIndex, 0, -1, 0, neighborIndex))
    {
        negativeY = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    if (offsetRootIndex(rootIndex, 0, 1, 0, neighborIndex))
    {
        positiveY = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    if (offsetRootIndex(rootIndex, 0, 0, -1, neighborIndex))
    {
        negativeZ = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    if (offsetRootIndex(rootIndex, 0, 0, 1, neighborIndex))
    {
        positiveZ = currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            neighborIndex);
    }

    return MyVoxel::VoxelRootOccupancyNeighborhood(
        currentRootOccupancy(
            oldOccupancies,
            changes,
            changePositions,
            rootIndex),
        negativeX,
        positiveX,
        negativeY,
        positiveY,
        negativeZ,
        positiveZ);
}

// 返回当前网格用于掩码叶块脏区域的层级。
MyVoxel::VoxelLevel dirtyLeafBlockLevel(const MyVoxel::VoxelGrid& grid)
{
    const unsigned int coveredLevelCount = 2; // 一个64位掩码叶块覆盖当前逻辑体素下面两级。

    MYVOXEL_ASSERT_MESSAGE(
        static_cast<unsigned int>(grid.maximumLevel()) >= coveredLevelCount,
        "Voxel surface incremental extraction requires at least two voxel levels.");

    return static_cast<MyVoxel::VoxelLevel>(
        static_cast<unsigned int>(grid.maximumLevel()) -
        coveredLevelCount);
}

// 保存Surface内部扩展后的可写脏区域，不向Core变化记录写回数据。
class SurfaceDirtyCellRegion
{
public:
    SurfaceDirtyCellRegion(const MyVoxel::VoxelCellIndex& rootIndexValue,
                           MyVoxel::VoxelLevel trackingLevelValue)
        : m_rootIndex(rootIndexValue)
        , m_trackingLevel(trackingLevelValue)
        , m_axisCellCount(0)
        , m_totalCellCount(0)
        , m_changedCellCount(0)
        , m_fullRoot(false)
    {
        const unsigned int levelValue =
            static_cast<unsigned int>(trackingLevelValue);
        const unsigned int sizeBitCount =
            static_cast<unsigned int>(sizeof(std::size_t) * 8U);

        MYVOXEL_ASSERT_MESSAGE(
            levelValue < sizeBitCount,
            "Surface dirty region tracking level exceeds size_t range.");

        m_axisCellCount =
            static_cast<std::size_t>(1) << levelValue;

        const std::size_t maximumSize =
            (std::numeric_limits<std::size_t>::max)();

        MYVOXEL_ASSERT_MESSAGE(
            m_axisCellCount <= maximumSize / m_axisCellCount,
            "Surface dirty region plane size overflowed.");

        const std::size_t planeCellCount =
            m_axisCellCount * m_axisCellCount;

        MYVOXEL_ASSERT_MESSAGE(
            planeCellCount <= maximumSize / m_axisCellCount,
            "Surface dirty region volume size overflowed.");

        m_totalCellCount =
            planeCellCount * m_axisCellCount;

        MYVOXEL_ASSERT_MESSAGE(
            m_totalCellCount <= maximumSize - static_cast<std::size_t>(63),
            "Surface dirty region word count overflowed.");

        const std::size_t wordCount =
            (m_totalCellCount + static_cast<std::size_t>(63)) /
            static_cast<std::size_t>(64);

        m_changedWords.assign(
            wordCount,
            static_cast<std::uint64_t>(0));
    }

    const MyVoxel::VoxelCellIndex& rootIndex() const
    {
        return m_rootIndex;
    }

    MyVoxel::VoxelLevel trackingLevel() const
    {
        return m_trackingLevel;
    }

    std::size_t axisCellCount() const
    {
        return m_axisCellCount;
    }

    std::size_t changedCellCount() const
    {
        return m_changedCellCount;
    }

    bool isFullRoot() const
    {
        return m_fullRoot;
    }

    void addLocalCell(std::size_t localX,
                      std::size_t localY,
                      std::size_t localZ)
    {
        MYVOXEL_ASSERT_MESSAGE(
            localX < m_axisCellCount &&
            localY < m_axisCellCount &&
            localZ < m_axisCellCount,
            "Surface dirty region local cell exceeds the Root.");

        if (m_fullRoot)
        {
            return;
        }

        const std::size_t bitIndex =
            (localZ * m_axisCellCount + localY) *
                m_axisCellCount +
            localX;
        const std::size_t wordIndex =
            bitIndex >> 6U;
        const unsigned int wordBit =
            static_cast<unsigned int>(bitIndex & 63U);
        const std::uint64_t bit =
            static_cast<std::uint64_t>(1ULL) << wordBit;

        if ((m_changedWords[wordIndex] & bit) == 0)
        {
            m_changedWords[wordIndex] |= bit;
            ++m_changedCellCount;
        }
    }

    void markFullRoot()
    {
        m_changedWords.clear();
        m_changedCellCount = m_totalCellCount;
        m_fullRoot = true;
    }

    void appendChangedLocalCellIndices(
        std::vector<std::size_t>& cellIndices) const
    {
        cellIndices.reserve(
            cellIndices.size() + m_changedCellCount);

        if (m_fullRoot)
        {
            for (std::size_t cellIndex = 0;
                 cellIndex < m_totalCellCount;
                 ++cellIndex)
            {
                cellIndices.push_back(cellIndex);
            }

            return;
        }

        for (std::size_t wordIndex = 0;
             wordIndex < m_changedWords.size();
             ++wordIndex)
        {
            std::uint64_t word =
                m_changedWords[wordIndex];

            while (word != 0)
            {
                unsigned int wordBit = 0;
                std::uint64_t shiftedWord = word;

                while ((shiftedWord &
                        static_cast<std::uint64_t>(1ULL)) == 0)
                {
                    shiftedWord >>= 1U;
                    ++wordBit;
                }

                cellIndices.push_back(
                    wordIndex * static_cast<std::size_t>(64) +
                    wordBit);

                word &= word - static_cast<std::uint64_t>(1ULL);
            }
        }
    }

private:
    MyVoxel::VoxelCellIndex m_rootIndex; // 当前扩展脏区域所属的第0层Root索引。
    MyVoxel::VoxelLevel m_trackingLevel; // 当前扩展区域使用的固定跟踪层级。
    std::size_t m_axisCellCount; // 当前Root单轴包含的跟踪层级体素数量。
    std::size_t m_totalCellCount; // 当前Root包含的跟踪层级体素总数量。
    std::size_t m_changedCellCount; // 当前扩展区域中唯一变化体素数量。
    bool m_fullRoot; // 当前扩展区域是否覆盖完整Root。
    std::vector<std::uint64_t> m_changedWords; // 当前扩展区域的局部体素位图。
};

using SurfaceDirtyCellRegionMap =
    std::map<MyVoxel::VoxelCellIndex, SurfaceDirtyCellRegion>;

// 将一个局部掩码叶块及其六个面邻块写入跨Root脏区域映射。
void appendExpandedDirtyLeafBlock(
    const MyVoxel::VoxelCellIndex& sourceRootIndex,
    MyVoxel::VoxelLevel blockLevel,
    std::size_t axisBlockCount,
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ,
    SurfaceDirtyCellRegionMap& expandedRegions)
{
    const int offsets[7][3] =
    {
        { 0, 0, 0 },
        { -1, 0, 0 },
        { 1, 0, 0 },
        { 0, -1, 0 },
        { 0, 1, 0 },
        { 0, 0, -1 },
        { 0, 0, 1 }
    }; // 一个材料位变化只会影响自身和六个面邻体素拥有的表面。

    for (unsigned int offsetIndex = 0; offsetIndex < 7; ++offsetIndex)
    {
        MyVoxel::VoxelCellIndex targetRootIndex = sourceRootIndex;
        std::int64_t targetX =
            static_cast<std::int64_t>(localX) + offsets[offsetIndex][0];
        std::int64_t targetY =
            static_cast<std::int64_t>(localY) + offsets[offsetIndex][1];
        std::int64_t targetZ =
            static_cast<std::int64_t>(localZ) + offsets[offsetIndex][2];
        int rootOffsetX = 0;
        int rootOffsetY = 0;
        int rootOffsetZ = 0;

        if (targetX < 0)
        {
            targetX = static_cast<std::int64_t>(axisBlockCount - 1);
            rootOffsetX = -1;
        }
        else if (targetX >= static_cast<std::int64_t>(axisBlockCount))
        {
            targetX = 0;
            rootOffsetX = 1;
        }

        if (targetY < 0)
        {
            targetY = static_cast<std::int64_t>(axisBlockCount - 1);
            rootOffsetY = -1;
        }
        else if (targetY >= static_cast<std::int64_t>(axisBlockCount))
        {
            targetY = 0;
            rootOffsetY = 1;
        }

        if (targetZ < 0)
        {
            targetZ = static_cast<std::int64_t>(axisBlockCount - 1);
            rootOffsetZ = -1;
        }
        else if (targetZ >= static_cast<std::int64_t>(axisBlockCount))
        {
            targetZ = 0;
            rootOffsetZ = 1;
        }

        if ((rootOffsetX != 0 || rootOffsetY != 0 || rootOffsetZ != 0) &&
            !offsetRootIndex(
                sourceRootIndex,
                rootOffsetX,
                rootOffsetY,
                rootOffsetZ,
                targetRootIndex))
        {
            continue;
        }

        SurfaceDirtyCellRegionMap::iterator regionIterator =
            expandedRegions.find(targetRootIndex);

        if (regionIterator == expandedRegions.end())
        {
            regionIterator = expandedRegions.insert(
                std::make_pair(
                    targetRootIndex,
                    SurfaceDirtyCellRegion(
                        targetRootIndex,
                        blockLevel))).first;
        }

        regionIterator->second.addLocalCell(
            static_cast<std::size_t>(targetX),
            static_cast<std::size_t>(targetY),
            static_cast<std::size_t>(targetZ));
    }
}

// 返回第0层根展开到最高层后的单轴体素数量。
std::int64_t rootAxisCellCount(const MyVoxel::VoxelGrid& grid)
{
    MYVOXEL_ASSERT_MESSAGE(
        grid.maximumLevel() >= MyVoxel::BaseVoxelLevel,
        "Voxel surface cache root level exceeds the grid maximum level.");

    const unsigned int difference = static_cast<unsigned int>(grid.maximumLevel() - MyVoxel::BaseVoxelLevel);

    MYVOXEL_ASSERT_MESSAGE(
        difference < 63,
        "Voxel surface cache root resolution exceeds the supported 64-bit range.");

    return static_cast<std::int64_t>(1) << difference;
}

// 并行准备全部修改Root更新后的核心占用，但暂不修改正式缓存。
void preparePendingRootOccupancies(
    const MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelChangeSet& changes,
    const MyVoxel::VoxelRootOccupancyMap& oldOccupancies,
    std::vector<PendingRootOccupancyChange>& pendingChanges,
    PendingRootOccupancyPositionMap& changePositions,
    MyVoxel::VoxelSurfaceUpdateStatistics* statistics)
{
    pendingChanges.clear();
    changePositions.clear();
    pendingChanges.resize(changes.modifiedRootCount());

    std::size_t changePosition = 0;

    for (MyVoxel::VoxelChangeSet::RootIndexSet::const_iterator rootIterator =
             changes.modifiedRootIndices().begin();
         rootIterator != changes.modifiedRootIndices().end();
         ++rootIterator, ++changePosition)
    {
        pendingChanges[changePosition].rootIndex = *rootIterator;
        changePositions.insert(
            std::make_pair(*rootIterator, changePosition));
    }

    if (pendingChanges.empty())
    {
        return;
    }

    MyVoxel::Foundation::Stopwatch wallTimer;
    MyVoxel::Foundation::ParallelExecutionStatistics parallelStatistics;
    MyVoxel::Foundation::ParallelOptions options;

    options.minimumParallelTaskCount = 4; // 至少四个修改Root时并行准备占用。
    options.statistics = statistics ? &parallelStatistics : nullptr;

    MyVoxel::Foundation::ParallelExecutor::global().execute(
        0,
        pendingChanges.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t taskIndex = blockBegin;
                 taskIndex < blockEnd;
                 ++taskIndex)
            {
                PendingRootOccupancyChange& pending =
                    pendingChanges[taskIndex];
                MyVoxel::Foundation::Stopwatch taskTimer;
                const MyVoxel::VoxelTree* tree =
                    shape.forest().getTree(pending.rootIndex);

                if (!tree)
                {
                    pending.hasOccupancy = false;
                    pending.cpuMilliseconds =
                        taskTimer.elapsedMilliseconds();
                    continue;
                }

                const MyVoxel::VoxelRootOccupancyMap::const_iterator oldIterator =
                    oldOccupancies.find(pending.rootIndex);
                const MyVoxel::VoxelChangeSet::DirtyCellRegion* dirtyRegion =
                    changes.dirtyRegion(pending.rootIndex);
                const bool canUpdateIncrementally =
                    oldIterator != oldOccupancies.end() &&
                    dirtyRegion &&
                    dirtyRegion->trackingLevel() == dirtyLeafBlockLevel(shape.grid()) &&
                    !dirtyRegion->isFullRoot();

                if (canUpdateIncrementally)
                {
                    pending.occupancy = oldIterator->second;
                    MyVoxel::VoxelFaceExtractor::updateRootOccupancy(
                        shape,
                        *dirtyRegion,
                        pending.occupancy);
                    pending.hasOccupancy = true;
                    pending.incremental = true;
                }
                else
                {
                    pending.hasOccupancy =
                        MyVoxel::VoxelFaceExtractor::extractRootOccupancy(
                            shape,
                            pending.rootIndex,
                            pending.occupancy);
                    pending.incremental = false;
                }

                pending.cpuMilliseconds =
                    taskTimer.elapsedMilliseconds();
            }
        },
        options);

    if (!statistics)
    {
        return;
    }

    statistics->occupancyBuildMilliseconds +=
        wallTimer.elapsedMilliseconds();
    statistics->occupancySourceRequestCount +=
        pendingChanges.size();
    statistics->uniqueOccupancyRootCount +=
        pendingChanges.size();
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
         taskIndex < pendingChanges.size();
         ++taskIndex)
    {
        const PendingRootOccupancyChange& pending =
            pendingChanges[taskIndex];

        statistics->occupancyBuildCpuMilliseconds +=
            pending.cpuMilliseconds;

        if (!pending.hasOccupancy)
        {
            ++statistics->occupancyRemovedRootCount;
        }
        else if (pending.incremental)
        {
            ++statistics->occupancyIncrementalUpdateCount;
            ++statistics->builtOccupancyRootCount;
        }
        else
        {
            ++statistics->occupancyFullRebuildCount;
            ++statistics->builtOccupancyRootCount;
        }
    }
}

// 将实际变化掩码叶块扩展六邻块，并只保留当前或旧缓存中可能拥有表面的Root。
void buildExpandedDirtyLeafRegions(
    const MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelChangeSet& changes,
    const MyVoxel::VoxelRootOccupancyMap& oldOccupancies,
    const std::vector<PendingRootOccupancyChange>& pendingChanges,
    const PendingRootOccupancyPositionMap& changePositions,
    const MyVoxel::VoxelSurfaceCache::RootEntryMap& oldRoots,
    SurfaceDirtyCellRegionMap& expandedRegions,
    MyVoxel::VoxelSurfaceCache::RootIndexSet& affectedRoots,
    MyVoxel::VoxelSurfaceUpdateStatistics* statistics)
{
    MyVoxel::Foundation::Stopwatch timer;
    expandedRegions.clear();
    affectedRoots.clear();

    const MyVoxel::VoxelLevel blockLevel =
        dirtyLeafBlockLevel(shape.grid());
    const std::size_t axisBlockCount =
        static_cast<std::size_t>(1) << static_cast<unsigned int>(blockLevel);
    std::size_t sourceBlockCount = 0;

    for (MyVoxel::VoxelChangeSet::RootIndexSet::const_iterator rootIterator =
             changes.modifiedRootIndices().begin();
         rootIterator != changes.modifiedRootIndices().end();
         ++rootIterator)
    {
        const MyVoxel::VoxelRootOccupancyMap::const_iterator oldOccupancyIterator =
            oldOccupancies.find(*rootIterator);
        const MyVoxel::VoxelRootOccupancy* currentOccupancy =
            currentRootOccupancy(
                oldOccupancies,
                pendingChanges,
                changePositions,
                *rootIterator);
        const MyVoxel::VoxelChangeSet::DirtyCellRegion* sourceRegion =
            changes.dirtyRegion(*rootIterator);
        std::vector<std::size_t> localBlockIndices;

        if (!sourceRegion ||
            sourceRegion->trackingLevel() != blockLevel ||
            (oldOccupancyIterator == oldOccupancies.end() && currentOccupancy))
        {
            const std::size_t blockCount =
                axisBlockCount * axisBlockCount * axisBlockCount;

            localBlockIndices.reserve(blockCount);

            for (std::size_t blockIndex = 0;
                 blockIndex < blockCount;
                 ++blockIndex)
            {
                localBlockIndices.push_back(blockIndex);
            }
        }
        else
        {
            localBlockIndices.reserve(
                sourceRegion->changedCellCount());
            sourceRegion->appendChangedLocalCellIndices(
                localBlockIndices);
        }

        sourceBlockCount +=
            localBlockIndices.size();

        for (std::size_t blockPosition = 0;
             blockPosition < localBlockIndices.size();
             ++blockPosition)
        {
            const std::size_t bitIndex =
                localBlockIndices[blockPosition];
            const std::size_t localX =
                bitIndex % axisBlockCount;
            const std::size_t planeIndex =
                bitIndex / axisBlockCount;
            const std::size_t localY =
                planeIndex % axisBlockCount;
            const std::size_t localZ =
                planeIndex / axisBlockCount;

            appendExpandedDirtyLeafBlock(
                *rootIterator,
                blockLevel,
                axisBlockCount,
                localX,
                localY,
                localZ,
                expandedRegions);
        }
    }

    std::size_t expandedBlockCount = 0;

    for (SurfaceDirtyCellRegionMap::iterator regionIterator =
             expandedRegions.begin();
         regionIterator != expandedRegions.end();)
    {
        const bool hasCurrentOccupancy =
            currentRootOccupancy(
                oldOccupancies,
                pendingChanges,
                changePositions,
                regionIterator->first) != nullptr;
        const bool hadSurface =
            oldRoots.find(regionIterator->first) != oldRoots.end();

        if (!hasCurrentOccupancy && !hadSurface)
        {
            SurfaceDirtyCellRegionMap::iterator eraseIterator =
                regionIterator;
            ++regionIterator;
            expandedRegions.erase(eraseIterator);
            continue;
        }

        expandedBlockCount +=
            regionIterator->second.changedCellCount();
        affectedRoots.insert(regionIterator->first);
        ++regionIterator;
    }

    if (statistics)
    {
        statistics->dirtyRegionBuildMilliseconds +=
            timer.elapsedMilliseconds();
        statistics->affectedRootCount =
            affectedRoots.size();
        statistics->dirtyLeafBlockCount +=
            sourceBlockCount;
        statistics->expandedDirtyLeafBlockCount +=
            expandedBlockCount;
    }
}

// 返回一个64位字中从startBit开始的连续置位掩码。
std::uint64_t contiguousBitMask(
    std::size_t startBit,
    std::size_t bitCount)
{
    MYVOXEL_ASSERT_MESSAGE(
        bitCount > 0 &&
        bitCount <= 64 &&
        startBit < 64 &&
        startBit + bitCount <= 64,
        "Voxel surface contiguous bit mask range is invalid.");

    if (bitCount == 64)
    {
        return (std::numeric_limits<std::uint64_t>::max)();
    }

    return ((static_cast<std::uint64_t>(1ULL) << bitCount) -
            static_cast<std::uint64_t>(1ULL)) <<
           startBit;
}

// 判断扩展Dirty Leaf覆盖是否达到完整字级面掩码重建阈值。
bool shouldFullyRebuildFaceMasks(
    const SurfaceDirtyCellRegion& region)
{
    const std::size_t axisBlockCount =
        region.axisCellCount();
    const std::size_t blockCount =
        axisBlockCount *
        axisBlockCount *
        axisBlockCount;

    return region.isFullRoot() ||
           region.changedCellCount() * 100 >=
               blockCount *
                   IncrementalFaceMaskFullRebuildPercent;
}

// 并行复制旧Root面掩码，并按扩展Dirty Leaf执行64位局部更新或完整字级重建。
void updateIncrementalRootFaceMasks(
    const MyVoxel::VoxelShape& shape,
    const MyVoxel::VoxelRootOccupancyMap& oldOccupancies,
    const std::vector<PendingRootOccupancyChange>& pendingChanges,
    const PendingRootOccupancyPositionMap& changePositions,
    const MyVoxel::VoxelSurfaceCache::RootEntryMap& oldRoots,
    const SurfaceDirtyCellRegionMap& expandedRegions,
    std::vector<MyVoxel::VoxelCellIndex>& rootIndices,
    std::vector<MyVoxel::VoxelRootFaceMasks>& faceMasks,
    MyVoxel::VoxelSurfaceUpdateStatistics* statistics)
{
    rootIndices.clear();
    rootIndices.reserve(expandedRegions.size());

    for (SurfaceDirtyCellRegionMap::const_iterator regionIterator =
             expandedRegions.begin();
         regionIterator != expandedRegions.end();
         ++regionIterator)
    {
        rootIndices.push_back(regionIterator->first);
    }

    faceMasks.clear();
    faceMasks.resize(rootIndices.size());

    if (rootIndices.empty())
    {
        return;
    }

    const std::size_t axisCellCount =
        static_cast<std::size_t>(
            rootAxisCellCount(shape.grid()));
    const MyVoxel::VoxelLevel blockLevel =
        dirtyLeafBlockLevel(shape.grid());
    const unsigned int remainingLevelCount =
        static_cast<unsigned int>(
            shape.grid().maximumLevel() -
            blockLevel);
    const std::size_t blockCellScale =
        static_cast<std::size_t>(1) <<
        remainingLevelCount;

    MYVOXEL_ASSERT_MESSAGE(
        blockCellScale > 0 &&
        blockCellScale <= 64,
        "Voxel surface incremental face mask block scale is invalid.");

    std::vector<double> cpuMilliseconds(
        statistics ? rootIndices.size() : 0,
        0.0);
    std::vector<std::size_t> updatedCellCounts(
        statistics ? rootIndices.size() : 0,
        0);
    std::vector<std::size_t> updatedWordCounts(
        statistics ? rootIndices.size() : 0,
        0);
    std::vector<unsigned char> fullRebuildFlags(
        statistics ? rootIndices.size() : 0,
        static_cast<unsigned char>(0));
    std::vector<std::size_t> emittedFaceCounts(
        statistics ? rootIndices.size() : 0,
        0);
    MyVoxel::Foundation::ParallelExecutionStatistics parallelStatistics;
    MyVoxel::Foundation::ParallelOptions options;

    options.minimumParallelTaskCount = 4; // 至少四个受影响Root时并行更新面掩码。
    options.statistics =
        statistics ? &parallelStatistics : nullptr;

    MyVoxel::Foundation::Stopwatch wallTimer;

    MyVoxel::Foundation::ParallelExecutor::global().execute(
        0,
        rootIndices.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t rootPosition = blockBegin;
                 rootPosition < blockEnd;
                 ++rootPosition)
            {
                MyVoxel::Foundation::Stopwatch taskTimer;
                const MyVoxel::VoxelCellIndex& rootIndex =
                    rootIndices[rootPosition];
                const MyVoxel::VoxelSurfaceCache::RootEntryMap::const_iterator oldRootIterator =
                    oldRoots.find(rootIndex);
                const SurfaceDirtyCellRegionMap::const_iterator regionIterator =
                    expandedRegions.find(rootIndex);

                MYVOXEL_ASSERT_MESSAGE(
                    regionIterator != expandedRegions.end(),
                    "Voxel surface incremental mask task lost its dirty region.");

                const MyVoxel::VoxelRootOccupancyNeighborhood neighborhood =
                    currentRootOccupancyNeighborhood(
                        oldOccupancies,
                        pendingChanges,
                        changePositions,
                        rootIndex);
                MyVoxel::VoxelRootFaceMasks result;

                if (shouldFullyRebuildFaceMasks(
                        regionIterator->second))
                {
                    result =
                        MyVoxel::VoxelFaceExtractor::buildRootFaceMasks(
                            rootIndex,
                            axisCellCount,
                            neighborhood);

                    if (statistics)
                    {
                        fullRebuildFlags[rootPosition] =
                            static_cast<unsigned char>(1);
                    }
                }
                else
                {
                    result =
                        oldRootIterator == oldRoots.end()
                            ? MyVoxel::VoxelRootFaceMasks(
                                rootIndex,
                                axisCellCount)
                            : oldRootIterator->second.faceMasks;

                    std::vector<std::size_t> localBlockIndices;
                    localBlockIndices.reserve(
                        regionIterator->second.changedCellCount());
                    regionIterator->second.appendChangedLocalCellIndices(
                        localBlockIndices);

                    const std::size_t axisBlockCount =
                        regionIterator->second.axisCellCount();
                    std::vector<MyVoxel::VoxelFaceMaskWord> rowFaces(
                        blockCellScale);
                    std::size_t maskedWordUpdateCount = 0;

                    for (std::size_t blockPosition = 0;
                         blockPosition < localBlockIndices.size();
                         ++blockPosition)
                    {
                        const std::size_t bitIndex =
                            localBlockIndices[blockPosition];
                        const std::size_t localBlockX =
                            bitIndex % axisBlockCount;
                        const std::size_t planeIndex =
                            bitIndex / axisBlockCount;
                        const std::size_t localBlockY =
                            planeIndex % axisBlockCount;
                        const std::size_t localBlockZ =
                            planeIndex / axisBlockCount;
                        const std::size_t startX =
                            localBlockX *
                            blockCellScale;
                        const std::size_t startY =
                            localBlockY *
                            blockCellScale;
                        const std::size_t startZ =
                            localBlockZ *
                            blockCellScale;
                        const std::size_t xWordIndex =
                            startX >> 6U;
                        const std::size_t yWordIndex =
                            startY >> 6U;
                        const unsigned int xStartBit =
                            static_cast<unsigned int>(
                                startX & 63U);
                        const unsigned int yStartBit =
                            static_cast<unsigned int>(
                                startY & 63U);
                        const std::uint64_t xReplaceMask =
                            contiguousBitMask(
                                xStartBit,
                                blockCellScale);
                        const std::uint64_t yReplaceMask =
                            contiguousBitMask(
                                yStartBit,
                                blockCellScale);

                        MYVOXEL_ASSERT_MESSAGE(
                            xStartBit + blockCellScale <= 64 &&
                            yStartBit + blockCellScale <= 64,
                            "Voxel surface dirty Leaf crossed a face-mask word boundary.");

                        for (std::size_t localZ = 0;
                             localZ < blockCellScale;
                             ++localZ)
                        {
                            const std::size_t z =
                                startZ + localZ;

                            for (std::size_t localY = 0;
                                 localY < blockCellScale;
                                 ++localY)
                            {
                                const std::size_t y =
                                    startY + localY;
                                MyVoxel::VoxelFaceMaskWord& faces =
                                    rowFaces[localY];

                                if (neighborhood.current)
                                {
                                    faces =
                                        MyVoxel::VoxelFaceExtractor::buildFaceMaskWord(
                                            neighborhood,
                                            y,
                                            z,
                                            xWordIndex);
                                }
                                else
                                {
                                    faces =
                                        MyVoxel::VoxelFaceMaskWord();
                                }

                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::NegativeY,
                                    y,
                                    z,
                                    xWordIndex,
                                    faces.negativeY,
                                    xReplaceMask);
                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::PositiveY,
                                    y,
                                    z,
                                    xWordIndex,
                                    faces.positiveY,
                                    xReplaceMask);
                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::NegativeZ,
                                    z,
                                    y,
                                    xWordIndex,
                                    faces.negativeZ,
                                    xReplaceMask);
                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::PositiveZ,
                                    z,
                                    y,
                                    xWordIndex,
                                    faces.positiveZ,
                                    xReplaceMask);
                                maskedWordUpdateCount += 4;
                            }

                            for (std::size_t localX = 0;
                                 localX < blockCellScale;
                                 ++localX)
                            {
                                const std::size_t x =
                                    startX + localX;
                                const std::uint64_t xBit =
                                    static_cast<std::uint64_t>(1ULL) <<
                                    static_cast<unsigned int>(
                                        x & 63U);
                                std::uint64_t negativeXWord = 0;
                                std::uint64_t positiveXWord = 0;

                                for (std::size_t localY = 0;
                                     localY < blockCellScale;
                                     ++localY)
                                {
                                    const std::uint64_t yBit =
                                        static_cast<std::uint64_t>(1ULL) <<
                                        static_cast<unsigned int>(
                                            (startY + localY) &
                                            63U);

                                    if ((rowFaces[localY].negativeX &
                                         xBit) != 0)
                                    {
                                        negativeXWord |= yBit;
                                    }

                                    if ((rowFaces[localY].positiveX &
                                         xBit) != 0)
                                    {
                                        positiveXWord |= yBit;
                                    }
                                }

                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::NegativeX,
                                    x,
                                    z,
                                    yWordIndex,
                                    negativeXWord,
                                    yReplaceMask);
                                result.replacePlaneWordMasked(
                                    MyVoxel::VoxelFaceDirection::PositiveX,
                                    x,
                                    z,
                                    yWordIndex,
                                    positiveXWord,
                                    yReplaceMask);
                                maskedWordUpdateCount += 2;
                            }
                        }
                    }

                    result.releaseEmptyStorage();

                    if (statistics)
                    {
                        updatedCellCounts[rootPosition] =
                            localBlockIndices.size() *
                            blockCellScale *
                            blockCellScale *
                            blockCellScale;
                        updatedWordCounts[rootPosition] =
                            maskedWordUpdateCount;
                    }
                }

                faceMasks[rootPosition] =
                    std::move(result);

                if (statistics)
                {
                    emittedFaceCounts[rootPosition] =
                        faceMasks[rootPosition].faceCount();
                    cpuMilliseconds[rootPosition] =
                        taskTimer.elapsedMilliseconds();
                }
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
        statistics->faceMaskUpdatedCellCount +=
            updatedCellCounts[rootPosition];
        statistics->faceMaskUpdatedWordCount +=
            updatedWordCounts[rootPosition];
        statistics->emittedFaceCount +=
            emittedFaceCounts[rootPosition];

        if (fullRebuildFlags[rootPosition] != 0)
        {
            ++statistics->faceMaskFullRebuildRootCount;
        }
        else
        {
            ++statistics->faceMaskIncrementalRootCount;
        }
    }
}

// 返回正除数下的向下取整商。
std::int64_t floorDivide(std::int64_t value, std::int64_t divisor)
{
    MYVOXEL_ASSERT_MESSAGE(divisor > 0, "Voxel surface cache floor division requires a positive divisor.");

    std::int64_t quotient = value / divisor;

    if (value % divisor < 0)
    {
        --quotient;
    }

    return quotient;
}

// 将64位索引转换为VoxelIndex。
MyVoxel::VoxelIndex voxelIndex(std::int64_t value)
{
    const std::int64_t minimum =
        static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());

    const std::int64_t maximum =
        static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(
        value >= minimum && value <= maximum,
        "Voxel surface cache root index exceeds VoxelIndex range.");

    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回最高层单位面所属的第0层根索引。
MyVoxel::VoxelCellIndex rootIndexOfFace(const MyVoxel::VoxelGrid& grid, const MyVoxel::VoxelFaceAddress& face)
{
    const std::int64_t rootScale = rootAxisCellCount(grid);

    return MyVoxel::VoxelCellIndex(
        voxelIndex(floorDivide(static_cast<std::int64_t>(face.cellIndex.x), rootScale)),
        voxelIndex(floorDivide(static_cast<std::int64_t>(face.cellIndex.y), rootScale)),
        voxelIndex(floorDivide(static_cast<std::int64_t>(face.cellIndex.z), rootScale)));
}

// 合并两个局部轴对齐包围盒，任一包围盒无效时返回另一个包围盒。
MyVoxel::Bounds3 combinedBounds(const MyVoxel::Bounds3& first, const MyVoxel::Bounds3& second)
{
    if (!first.isValid())
    {
        return second;
    }

    if (!second.isValid())
    {
        return first;
    }

    const MyMath::Vector3& firstMinimum = first.minimum();
    const MyMath::Vector3& firstMaximum = first.maximum();
    const MyMath::Vector3& secondMinimum = second.minimum();
    const MyMath::Vector3& secondMaximum = second.maximum();

    return MyVoxel::Bounds3(
        MyMath::Vector3(
            (std::min)(firstMinimum.x(), secondMinimum.x()),
            (std::min)(firstMinimum.y(), secondMinimum.y()),
            (std::min)(firstMinimum.z(), secondMinimum.z())),
        MyMath::Vector3(
            (std::max)(firstMaximum.x(), secondMaximum.x()),
            (std::max)(firstMaximum.y(), secondMaximum.y()),
            (std::max)(firstMaximum.z(), secondMaximum.z())));
}

// 保存一个变化Root的新旧面掩码及其独立差分输出。
struct RootFaceDifferenceTask
{
    RootFaceDifferenceTask()
        : rootPosition(0)
        , newFaceMasks(nullptr)
        , oldFaceMasks(nullptr)
        , cpuMilliseconds(0.0)
    {
    }

    std::size_t rootPosition; // 当前任务在受影响Root有序数组中的位置。
    const MyVoxel::VoxelRootFaceMasks* newFaceMasks; // 当前Root重新提取得到的新方向面掩码。
    const MyVoxel::VoxelRootFaceMasks* oldFaceMasks; // 更新前的只读方向面掩码，新Root时为空。
    MyVoxel::VoxelFaceSet addedFaces; // 当前Root新增且已经排序去重的单位面集合。
    MyVoxel::VoxelFaceSet removedFaces; // 当前Root删除且已经排序去重的单位面集合。
    double cpuMilliseconds; // 当前任务生成差分地址并完成Root局部排序去重的耗时。
};

// 指定需要归并Root新增面或删除面。
enum class RootFaceDifferenceKind
{
    Added,
    Removed
};

// 返回指定Root差分任务中的有序面集合。
const MyVoxel::VoxelFaceSet& rootFaceDifferenceSet(
    const RootFaceDifferenceTask& task,
    RootFaceDifferenceKind kind)
{
    return kind == RootFaceDifferenceKind::Added
        ? task.addedFaces
        : task.removedFaces;
}

// 保存K路归并中一个有序Root面集合的当前读取位置。
struct RootFaceMergeCursor
{
    RootFaceMergeCursor(
        const MyVoxel::VoxelFaceSet::Container* facesValue,
        std::size_t faceIndexValue)
        : faces(facesValue)
        , faceIndex(faceIndexValue)
    {
        MYVOXEL_ASSERT_MESSAGE(
            facesValue && faceIndexValue < facesValue->size(),
            "Voxel surface Root face merge cursor requires a valid face.");
    }

    // 返回当前游标指向的面地址。
    const MyVoxel::VoxelFaceAddress& face() const
    {
        MYVOXEL_ASSERT_MESSAGE(
            faces && faceIndex < faces->size(),
            "Voxel surface Root face merge cursor is invalid.");
        return (*faces)[faceIndex];
    }

    const MyVoxel::VoxelFaceSet::Container* faces; // 当前游标读取的Root局部有序面数组。
    std::size_t faceIndex; // 当前游标在Root局部面数组中的位置。
};

// 将priority_queue调整为按最小VoxelFaceAddress优先弹出。
struct RootFaceMergeCursorGreater
{
    bool operator()(
        const RootFaceMergeCursor& first,
        const RootFaceMergeCursor& second) const
    {
        return second.face() < first.face();
    }
};

// 将多个Root局部有序唯一面集合归并为一个全局有序唯一面集合。
MyVoxel::VoxelFaceSet mergeRootFaceDifferences(
    const std::vector<RootFaceDifferenceTask>& tasks,
    RootFaceDifferenceKind kind,
    std::size_t totalFaceCount)
{
    MyVoxel::VoxelFaceSet::Container mergedFaces;
    mergedFaces.reserve(totalFaceCount);

    const MyVoxel::VoxelFaceSet::Container* firstFaces = nullptr;
    const MyVoxel::VoxelFaceSet::Container* secondFaces = nullptr;
    std::size_t nonEmptySetCount = 0;

    for (std::size_t taskIndex = 0;
         taskIndex < tasks.size();
         ++taskIndex)
    {
        const MyVoxel::VoxelFaceSet::Container& faces =
            rootFaceDifferenceSet(tasks[taskIndex], kind).faces();

        if (faces.empty())
        {
            continue;
        }

        if (!firstFaces)
        {
            firstFaces = &faces;
        }
        else if (!secondFaces)
        {
            secondFaces = &faces;
        }

        ++nonEmptySetCount;
    }

    if (nonEmptySetCount == 0)
    {
        return MyVoxel::VoxelFaceSet();
    }

    if (nonEmptySetCount == 1)
    {
        mergedFaces.insert(
            mergedFaces.end(),
            firstFaces->begin(),
            firstFaces->end());
    }
    else if (nonEmptySetCount == 2)
    {
        MYVOXEL_ASSERT_MESSAGE(
            firstFaces && secondFaces,
            "Voxel surface two-way face merge requires two source arrays.");

        mergedFaces.resize(totalFaceCount);

        std::merge(
            firstFaces->begin(),
            firstFaces->end(),
            secondFaces->begin(),
            secondFaces->end(),
            mergedFaces.begin());
    }
    else
    {
        std::priority_queue<
            RootFaceMergeCursor,
            std::vector<RootFaceMergeCursor>,
            RootFaceMergeCursorGreater> cursors;

        for (std::size_t taskIndex = 0;
             taskIndex < tasks.size();
             ++taskIndex)
        {
            const MyVoxel::VoxelFaceSet::Container& faces =
                rootFaceDifferenceSet(tasks[taskIndex], kind).faces();

            if (!faces.empty())
            {
                cursors.push(
                    RootFaceMergeCursor(
                        &faces,
                        0));
            }
        }

        while (!cursors.empty())
        {
            RootFaceMergeCursor cursor =
                cursors.top();
            cursors.pop();

            mergedFaces.push_back(
                cursor.face());

            ++cursor.faceIndex;

            if (cursor.faceIndex < cursor.faces->size())
            {
                cursors.push(cursor);
            }
        }
    }

    MYVOXEL_ASSERT_MESSAGE(
        mergedFaces.size() == totalFaceCount,
        "Voxel surface merged face count is invalid.");

    return MyVoxel::VoxelFaceSet::fromSortedUnique(
        std::move(mergedFaces));
}

// 并行生成全部变化Root的新增和删除单位面地址。
void buildRootFaceDifferences(
    std::vector<RootFaceDifferenceTask>& tasks,
    MyVoxel::VoxelSurfaceUpdateStatistics* statistics)
{
    if (tasks.empty())
    {
        return;
    }

    MyVoxel::Foundation::Stopwatch wallTimer;
    MyVoxel::Foundation::ParallelExecutionStatistics parallelStatistics;
    MyVoxel::Foundation::ParallelOptions options;

    options.minimumParallelTaskCount =
        MinimumParallelRootDifferenceTaskCount;
    options.statistics =
        statistics ? &parallelStatistics : nullptr;

    MyVoxel::Foundation::ParallelExecutor::global().execute(
        0,
        tasks.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t taskIndex = blockBegin;
                 taskIndex < blockEnd;
                 ++taskIndex)
            {
                RootFaceDifferenceTask& task =
                    tasks[taskIndex];

                MYVOXEL_ASSERT_MESSAGE(
                    task.newFaceMasks,
                    "Voxel surface Root difference task requires new face masks.");

                MyVoxel::Foundation::Stopwatch taskTimer;
                MyVoxel::VoxelFaceSet::Container addedFaceArray;
                MyVoxel::VoxelFaceSet::Container removedFaceArray;

                if (!task.oldFaceMasks)
                {
                    task.newFaceMasks->appendFaces(
                        addedFaceArray);
                }
                else
                {
                    task.newFaceMasks->appendDifferenceFaces(
                        *task.oldFaceMasks,
                        addedFaceArray);
                    task.oldFaceMasks->appendDifferenceFaces(
                        *task.newFaceMasks,
                        removedFaceArray);
                }

                task.addedFaces.assign(
                    std::move(addedFaceArray));
                task.removedFaces.assign(
                    std::move(removedFaceArray));

                task.cpuMilliseconds =
                    taskTimer.elapsedMilliseconds();
            }
        },
        options);

    if (!statistics)
    {
        return;
    }

    statistics->faceDifferenceMilliseconds +=
        wallTimer.elapsedMilliseconds();
    ++statistics->faceDifferenceCallCount;
    statistics->faceDifferenceTaskCount +=
        parallelStatistics.taskCount;
    statistics->faceDifferenceParticipantCount +=
        parallelStatistics.workers.size();
    statistics->faceDifferenceBatchSizeTotal +=
        parallelStatistics.batchSize;

    if (parallelStatistics.serialExecution)
    {
        ++statistics->faceDifferenceSerialCallCount;
    }

    if (parallelStatistics.nestedSerialExecution)
    {
        ++statistics->faceDifferenceNestedSerialCallCount;
    }

    for (std::size_t taskIndex = 0;
         taskIndex < tasks.size();
         ++taskIndex)
    {
        statistics->faceDifferenceCpuMilliseconds +=
            tasks[taskIndex].cpuMilliseconds;
    }
}

// 指定一个Root方向网格在本次事务中的处理动作。
enum class RootDirectionMeshAction
{
    Clear,
    Reuse,
    Rebuild
};

// 保存六个方向网格的重建、复用和清空计划及其临时输出。
struct RootDirectionMeshPlan
{
    using DirectionMeshes =
        MyVoxel::VoxelSurfaceCache::DirectionMeshes;

    RootDirectionMeshPlan()
        : existingMeshes(nullptr)
        , actions()
        , rebuiltMeshes()
    {
        actions.fill(RootDirectionMeshAction::Clear);
    }

    // 根据完整重建请求，将非空方向标记为重建，空方向标记为清空。
    void prepareFull(const MyVoxel::VoxelRootFaceMasks& faceMasks)
    {
        existingMeshes = nullptr;

        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            const MyVoxel::VoxelFaceDirection direction =
                static_cast<MyVoxel::VoxelFaceDirection>(directionValue);

            actions[directionValue] =
                faceMasks.directionIsEmpty(direction)
                    ? RootDirectionMeshAction::Clear
                    : RootDirectionMeshAction::Rebuild;
        }
    }

    // 根据新旧方向掩码建立增量计划，未变化方向复用旧网格。
    void prepareIncremental(
        const MyVoxel::VoxelRootFaceMasks& newFaceMasks,
        const MyVoxel::VoxelRootFaceMasks* oldFaceMasks,
        const DirectionMeshes* oldDirectionMeshes)
    {
        existingMeshes = oldDirectionMeshes;

        MYVOXEL_ASSERT_MESSAGE(
            (oldFaceMasks == nullptr) == (oldDirectionMeshes == nullptr),
            "Voxel surface incremental direction plan requires matching old masks and meshes.");

        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            const MyVoxel::VoxelFaceDirection direction =
                static_cast<MyVoxel::VoxelFaceDirection>(directionValue);

            if (oldFaceMasks &&
                newFaceMasks.directionEquals(
                    *oldFaceMasks,
                    direction))
            {
                actions[directionValue] =
                    RootDirectionMeshAction::Reuse;
            }
            else if (newFaceMasks.directionIsEmpty(direction))
            {
                actions[directionValue] =
                    RootDirectionMeshAction::Clear;
            }
            else
            {
                actions[directionValue] =
                    RootDirectionMeshAction::Rebuild;
            }
        }
    }

    // 判断指定方向是否需要执行Mesher。
    bool needsRebuild(unsigned int directionValue) const
    {
        MYVOXEL_ASSERT_MESSAGE(
            directionValue < MyVoxel::VoxelFaceDirectionCount,
            "Voxel surface direction plan index is invalid.");

        return actions[directionValue] ==
            RootDirectionMeshAction::Rebuild;
    }

    // 判断指定方向是否直接复用旧网格。
    bool reusesExisting(unsigned int directionValue) const
    {
        MYVOXEL_ASSERT_MESSAGE(
            directionValue < MyVoxel::VoxelFaceDirectionCount,
            "Voxel surface direction plan index is invalid.");

        return actions[directionValue] ==
            RootDirectionMeshAction::Reuse;
    }

    // 判断指定方向是否在更新后为空。
    bool clearsDirection(unsigned int directionValue) const
    {
        MYVOXEL_ASSERT_MESSAGE(
            directionValue < MyVoxel::VoxelFaceDirectionCount,
            "Voxel surface direction plan index is invalid.");

        return actions[directionValue] ==
            RootDirectionMeshAction::Clear;
    }

    // 判断当前Root是否至少需要构建一个方向。
    bool hasRebuildDirections() const
    {
        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            if (needsRebuild(directionValue))
            {
                return true;
            }
        }

        return false;
    }

    // 返回指定方向用于接收Mesher结果的临时网格。
    MyVoxel::Geometry::Mesh& rebuiltMesh(unsigned int directionValue)
    {
        MYVOXEL_ASSERT_MESSAGE(
            needsRebuild(directionValue),
            "Voxel surface requested a rebuilt Mesh for a non-rebuild direction.");

        return rebuiltMeshes[directionValue];
    }

    // 返回指定方向在当前事务完成后应参与Root组合的网格。
    const MyVoxel::Geometry::Mesh& resolvedMesh(
        unsigned int directionValue) const
    {
        MYVOXEL_ASSERT_MESSAGE(
            directionValue < MyVoxel::VoxelFaceDirectionCount,
            "Voxel surface direction plan index is invalid.");

        if (needsRebuild(directionValue))
        {
            return rebuiltMeshes[directionValue];
        }

        if (reusesExisting(directionValue))
        {
            MYVOXEL_ASSERT_MESSAGE(
                existingMeshes,
                "Voxel surface direction reuse requires existing direction meshes.");

            return (*existingMeshes)[directionValue];
        }

        return rebuiltMeshes[directionValue];
    }

    // 将构建和清空结果提交到Root方向网格，并仅为实际变化方向推进版本。
    void commit(
        DirectionMeshes& targetMeshes,
        MyVoxel::VoxelSurfaceCache::DirectionMeshVersions& targetVersions)
    {
        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            if (needsRebuild(directionValue))
            {
                targetMeshes[directionValue] =
                    std::move(rebuiltMeshes[directionValue]);
                advanceDirectionMeshVersion(
                    targetVersions[directionValue]);
            }
            else if (clearsDirection(directionValue) &&
                     !targetMeshes[directionValue].isEmpty())
            {
                targetMeshes[directionValue] =
                    MyVoxel::Geometry::Mesh();
                advanceDirectionMeshVersion(
                    targetVersions[directionValue]);
            }
        }
    }

    const DirectionMeshes* existingMeshes; // 增量事务中保持只读的旧六方向网格。
    std::array<RootDirectionMeshAction, MyVoxel::VoxelFaceDirectionCount> actions; // 六方向处理动作。
    DirectionMeshes rebuiltMeshes; // 仅保存本次实际重建方向的临时输出。
};

// 保存一次增量更新中一个根的新面掩码、局部颜色及其待提交网格。
struct RootSurfaceChange
{
    RootSurfaceChange(
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        MyVoxel::VoxelRootFaceMasks&& faceMasksValue,
        const MyVoxel::VoxelRootFaceMasks* oldFaceMasksValue,
        const MyVoxel::VoxelSurfaceCache::DirectionMeshes*
            oldDirectionMeshesValue,
        const MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints&
            directionQuadCountHintsValue)
        : rootIndex(rootIndexValue)
        , faceMasks(std::move(faceMasksValue))
        , directionQuadCountHints(directionQuadCountHintsValue)
    {
        directionMeshPlan.prepareIncremental(
            faceMasks,
            oldFaceMasksValue,
            oldDirectionMeshesValue);
    }

    // 返回当前根待提交的方向面掩码。
    const MyVoxel::VoxelRootFaceMasks& rootFaceMasks() const
    {
        return faceMasks;
    }

    // 返回当前根待提交的局部颜色映射。
    const MyVoxel::VoxelFaceColorMap& colorMap() const
    {
        return colors;
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前发生表面变化的第0层根索引。
    MyVoxel::VoxelRootFaceMasks faceMasks; // 当前根更新后的方向面掩码。
    MyVoxel::VoxelFaceColorMap colors; // 当前根更新后的局部单独面颜色。
    RootDirectionMeshPlan directionMeshPlan; // 六方向网格的重建、复用和清空计划。
    MyVoxel::Bounds3 localBounds; // 新根网格对应的局部轴对齐包围盒。
    MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints directionQuadCountHints; // 六个方向的历史及本次实际四边形数量。
    MyVoxel::VoxelSurfaceMeshingStatistics meshingStatistics; // 当前根实际重建方向的阶段统计。
};

// 保存一个只重新构建网格和包围盒的根任务。
struct RootMeshBuildItem
{
    RootMeshBuildItem(
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        const MyVoxel::VoxelRootFaceMasks* faceMasksValue,
        const MyVoxel::VoxelFaceColorMap* colorsValue,
        const MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints&
            directionQuadCountHintsValue)
        : rootIndex(rootIndexValue)
        , faceMasks(faceMasksValue)
        , colors(colorsValue)
        , directionQuadCountHints(directionQuadCountHintsValue)
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasksValue,
            "Voxel surface cache root build item requires face masks.");
        MYVOXEL_ASSERT_MESSAGE(
            colorsValue,
            "Voxel surface cache root build item requires a color map.");

        directionMeshPlan.prepareFull(*faceMasksValue);
    }

    // 返回当前根保持稳定的方向面掩码。
    const MyVoxel::VoxelRootFaceMasks& rootFaceMasks() const
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasks,
            "Voxel surface cache root build item requires face masks.");
        return *faceMasks;
    }

    // 返回当前根保持稳定的局部颜色映射。
    const MyVoxel::VoxelFaceColorMap& colorMap() const
    {
        MYVOXEL_ASSERT_MESSAGE(
            colors,
            "Voxel surface cache root build item requires a color map.");
        return *colors;
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前任务对应的第0层根索引。
    const MyVoxel::VoxelRootFaceMasks* faceMasks; // 当前根保持稳定的只读方向面掩码。
    const MyVoxel::VoxelFaceColorMap* colors; // 当前根保持稳定的只读局部颜色映射。
    RootDirectionMeshPlan directionMeshPlan; // 六方向完整重建计划。
    MyVoxel::Bounds3 localBounds; // 当前任务独立生成的根网格包围盒。
    MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints directionQuadCountHints; // 六个方向的历史及本次实际四边形数量。
    MyVoxel::VoxelSurfaceMeshingStatistics meshingStatistics; // 当前根实际重建方向的阶段统计。
};

// 保存一次只修改Root局部颜色的事务准备结果。
struct RootColorChange
{
    RootColorChange(
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        const MyVoxel::VoxelRootFaceMasks* faceMasksValue,
        const MyVoxel::VoxelFaceColorMap& colorsValue,
        const MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints&
            directionQuadCountHintsValue)
        : rootIndex(rootIndexValue)
        , faceMasks(faceMasksValue)
        , colors(colorsValue)
        , directionQuadCountHints(directionQuadCountHintsValue)
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasksValue,
            "Voxel surface color change requires face masks.");

        directionMeshPlan.prepareFull(*faceMasksValue);
    }

    // 返回当前根保持稳定的方向面掩码。
    const MyVoxel::VoxelRootFaceMasks& rootFaceMasks() const
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasks,
            "Voxel surface color change requires face masks.");
        return *faceMasks;
    }

    // 返回当前根待提交的局部颜色映射。
    const MyVoxel::VoxelFaceColorMap& colorMap() const
    {
        return colors;
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前颜色变化对应的第0层根索引。
    const MyVoxel::VoxelRootFaceMasks* faceMasks; // 当前根保持稳定的只读方向面掩码。
    MyVoxel::VoxelFaceColorMap colors; // 当前根待提交的局部颜色映射。
    RootDirectionMeshPlan directionMeshPlan; // 颜色变化触发的六方向完整重建计划。
    MyVoxel::Bounds3 localBounds; // 新根网格对应的局部轴对齐包围盒。
    MyVoxel::VoxelSurfaceCache::DirectionQuadCountHints directionQuadCountHints; // 六个方向的历史及本次实际四边形数量。
    MyVoxel::VoxelSurfaceMeshingStatistics meshingStatistics; // 当前根实际重建方向的阶段统计。
};

// 保存一个Root方向独立网格构建任务。
struct RootDirectionMeshBuildTask
{
    RootDirectionMeshBuildTask(
        std::size_t rootPositionValue,
        const MyVoxel::VoxelCellIndex& rootIndexValue,
        MyVoxel::VoxelFaceDirection directionValue,
        const MyVoxel::VoxelRootFaceMasks* faceMasksValue,
        const MyVoxel::VoxelRootSurfaceColorData* colorDataValue,
        std::size_t reserveQuadCountValue)
        : rootPosition(rootPositionValue)
        , rootIndex(rootIndexValue)
        , direction(directionValue)
        , faceMasks(faceMasksValue)
        , colorData(colorDataValue)
        , reserveQuadCount(reserveQuadCountValue)
    {
        MYVOXEL_ASSERT_MESSAGE(
            faceMasksValue,
            "Voxel surface direction task requires face masks.");
        MYVOXEL_ASSERT_MESSAGE(
            colorDataValue,
            "Voxel surface direction task requires prepared color data.");
        MYVOXEL_ASSERT_MESSAGE(
            MyVoxel::isValidVoxelFaceDirection(directionValue),
            "Voxel surface direction task received an invalid direction.");
    }

    std::size_t rootPosition; // 当前方向任务所属的Root任务位置。
    MyVoxel::VoxelCellIndex rootIndex; // 当前方向任务所属的第0层Root索引。
    MyVoxel::VoxelFaceDirection direction; // 当前任务独立处理的表面方向。
    const MyVoxel::VoxelRootFaceMasks* faceMasks; // 当前Root稳定的只读方向面掩码。
    const MyVoxel::VoxelRootSurfaceColorData* colorData; // 当前Root稳定的只读方向切片颜色数据。
    std::size_t reserveQuadCount; // 根据当前Root上一次实际结果计算的四边形预留容量。
    MyVoxel::Geometry::Mesh mesh; // 当前方向独立生成的临时网格。
    MyVoxel::VoxelSurfaceMeshingStatistics meshingStatistics; // 当前方向构建阶段统计。
};

// 将Root任务扁平化为Root与方向任务并行构建，再按Root和方向固定顺序合并。
template<typename RootItem>
void buildRootMeshes(const MyVoxel::VoxelShape& shape,
                     std::vector<RootItem>& items,
                     const MyVoxel::Geometry::MeshColor& defaultColor,
                     MyVoxel::VoxelSurfaceUpdateStatistics* statistics)
{
    if (items.empty())
    {
        return;
    }

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        items.size() <= maximumSize / MyVoxel::VoxelFaceDirectionCount,
        "Voxel surface Root direction count overflowed.");

    MyVoxel::Foundation::Stopwatch meshBuildTimer;
    MyVoxel::Foundation::Stopwatch planTimer;
    std::vector<MyVoxel::VoxelRootSurfaceColorData> rootColorData(
        items.size());
    std::vector<RootDirectionMeshBuildTask> directionTasks;
    directionTasks.reserve(
        items.size() * MyVoxel::VoxelFaceDirectionCount);

    std::size_t consideredDirectionCount = 0;
    std::size_t reusedDirectionCount = 0;
    std::size_t clearedDirectionCount = 0;
    std::size_t reusedQuadCount = 0;

    for (std::size_t itemIndex = 0;
         itemIndex < items.size();
         ++itemIndex)
    {
        RootItem& item = items[itemIndex];
        item.localBounds = MyVoxel::Bounds3();
        item.meshingStatistics.clear();

        if (item.directionMeshPlan.hasRebuildDirections())
        {
            rootColorData[itemIndex].build(
                item.rootFaceMasks(),
                item.colorMap());
        }

        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            ++consideredDirectionCount;

            if (item.directionMeshPlan.needsRebuild(directionValue))
            {
                directionTasks.push_back(
                    RootDirectionMeshBuildTask(
                        itemIndex,
                        item.rootIndex,
                        static_cast<MyVoxel::VoxelFaceDirection>(directionValue),
                        &item.rootFaceMasks(),
                        &rootColorData[itemIndex],
                        directionMeshReserveQuadCount(
                            item.directionQuadCountHints[directionValue])));
            }
            else if (item.directionMeshPlan.reusesExisting(directionValue))
            {
                const MyVoxel::Geometry::Mesh& reusedMesh =
                    item.directionMeshPlan.resolvedMesh(directionValue);

                MYVOXEL_ASSERT_MESSAGE(
                    reusedMesh.isValid(),
                    "Voxel surface reused direction Mesh is invalid.");
                MYVOXEL_ASSERT_MESSAGE(
                    reusedMesh.triangleCount() % 2 == 0,
                    "Voxel surface reused direction triangle count must be even.");
                MYVOXEL_ASSERT_MESSAGE(
                    reusedMesh.triangleCount() / 2 <=
                        maximumSize - reusedQuadCount,
                    "Voxel surface reused direction quad count overflowed.");

                ++reusedDirectionCount;
                reusedQuadCount +=
                    reusedMesh.triangleCount() / 2;
            }
            else
            {
                ++clearedDirectionCount;
                item.directionQuadCountHints[directionValue] = 0;
            }
        }
    }

    const double directionPlanMilliseconds =
        planTimer.elapsedMilliseconds();

    MyVoxel::Foundation::ParallelExecutionStatistics parallelStatistics;

    if (!directionTasks.empty())
    {
        MyVoxel::Foundation::ParallelOptions options;
        options.minimumParallelTaskCount =
            MinimumParallelRootDirectionTaskCount;
        options.statistics =
            statistics ? &parallelStatistics : nullptr;

        MyVoxel::Foundation::ParallelExecutor::global().execute(
            0,
            directionTasks.size(),
            [&](std::size_t blockBegin, std::size_t blockEnd)
            {
                for (std::size_t taskIndex = blockBegin;
                     taskIndex < blockEnd;
                     ++taskIndex)
                {
                    RootDirectionMeshBuildTask& task =
                        directionTasks[taskIndex];

                    task.mesh =
                        MyVoxel::VoxelSurfaceMesher::buildRootDirection(
                            shape,
                            task.rootIndex,
                            task.direction,
                            *task.faceMasks,
                            *task.colorData,
                            defaultColor,
                            task.reserveQuadCount,
                            statistics ? &task.meshingStatistics : nullptr);

                    MYVOXEL_ASSERT_MESSAGE(
                        task.mesh.isValid(),
                        "Voxel surface direction task produced an invalid mesh.");
                }
            },
            options);
    }

    MyVoxel::Foundation::Stopwatch finalizeTimer;
    std::size_t hintedTaskCount = 0;
    std::size_t reservedQuadCount = 0;
    std::size_t hintUnderestimateCount = 0;
    std::size_t unusedReservedQuadCount = 0;

    for (std::size_t taskIndex = 0;
         taskIndex < directionTasks.size();
         ++taskIndex)
    {
        RootDirectionMeshBuildTask& task =
            directionTasks[taskIndex];

        MYVOXEL_ASSERT_MESSAGE(
            task.rootPosition < items.size(),
            "Voxel surface direction task Root position is invalid.");

        const std::size_t directionIndex =
            static_cast<std::size_t>(task.direction);

        MYVOXEL_ASSERT_MESSAGE(
            directionIndex < MyVoxel::VoxelFaceDirectionCount,
            "Voxel surface direction task index is invalid.");
        MYVOXEL_ASSERT_MESSAGE(
            task.mesh.triangleCount() % 2 == 0,
            "Voxel surface direction mesh triangle count must be even.");

        const std::size_t actualQuadCount =
            task.mesh.triangleCount() / 2;

        items[task.rootPosition].directionQuadCountHints[directionIndex] =
            actualQuadCount;
        items[task.rootPosition].directionMeshPlan.rebuiltMesh(
            static_cast<unsigned int>(directionIndex)) =
                std::move(task.mesh);

        if (task.reserveQuadCount > 0)
        {
            ++hintedTaskCount;

            MYVOXEL_ASSERT_MESSAGE(
                task.reserveQuadCount <=
                    maximumSize - reservedQuadCount,
                "Voxel surface reserved direction quad count overflowed.");

            reservedQuadCount +=
                task.reserveQuadCount;

            if (actualQuadCount > task.reserveQuadCount)
            {
                ++hintUnderestimateCount;
            }
            else
            {
                const std::size_t unusedQuadCount =
                    task.reserveQuadCount - actualQuadCount;

                MYVOXEL_ASSERT_MESSAGE(
                    unusedQuadCount <=
                        maximumSize - unusedReservedQuadCount,
                    "Voxel surface unused reserved quad count overflowed.");

                unusedReservedQuadCount +=
                    unusedQuadCount;
            }
        }

        items[task.rootPosition].meshingStatistics.add(
            task.meshingStatistics);
    }

    for (std::size_t itemIndex = 0;
         itemIndex < items.size();
         ++itemIndex)
    {
        RootItem& item = items[itemIndex];
        MyVoxel::Bounds3 rootBounds;

        for (unsigned int directionValue = 0;
             directionValue < MyVoxel::VoxelFaceDirectionCount;
             ++directionValue)
        {
            const MyVoxel::Geometry::Mesh& directionMesh =
                item.directionMeshPlan.resolvedMesh(directionValue);

            MYVOXEL_ASSERT_MESSAGE(
                directionMesh.isValid(),
                "Voxel surface resolved direction Mesh is invalid.");

            rootBounds =
                combinedBounds(
                    rootBounds,
                    directionMesh.localBounds());
        }

        if (item.rootFaceMasks().isEmpty())
        {
            MYVOXEL_ASSERT_MESSAGE(
                !rootBounds.isValid(),
                "Empty voxel surface Root produced valid direction bounds.");
            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(
            rootBounds.isValid(),
            "Voxel surface direction Mesh set produced invalid Root bounds.");

        item.localBounds = rootBounds;
    }

    const double directionFinalizeMilliseconds =
        finalizeTimer.elapsedMilliseconds();

    if (statistics)
    {
        statistics->meshBuildWallMilliseconds +=
            meshBuildTimer.elapsedMilliseconds();
        statistics->rootColorPrepareMilliseconds +=
            directionPlanMilliseconds;
        statistics->directionMeshPlanMilliseconds +=
            directionPlanMilliseconds;
        statistics->directionFinalizeMilliseconds +=
            directionFinalizeMilliseconds;
        statistics->directionMeshHintedTaskCount +=
            hintedTaskCount;
        statistics->directionMeshReservedQuadCount +=
            reservedQuadCount;
        statistics->directionMeshHintUnderestimateCount +=
            hintUnderestimateCount;
        statistics->directionMeshUnusedReservedQuadCount +=
            unusedReservedQuadCount;
        statistics->directionMeshConsideredCount +=
            consideredDirectionCount;
        statistics->directionMeshReusedCount +=
            reusedDirectionCount;
        statistics->directionMeshClearedCount +=
            clearedDirectionCount;
        statistics->directionMeshReusedQuadCount +=
            reusedQuadCount;
        ++statistics->meshBuildCallCount;
        statistics->meshBuildTaskCount +=
            parallelStatistics.taskCount;
        statistics->meshBuildParticipantCount +=
            parallelStatistics.workers.size();
        statistics->meshBuildBatchSizeTotal +=
            parallelStatistics.batchSize;

        if (parallelStatistics.serialExecution)
        {
            ++statistics->meshBuildSerialCallCount;
        }

        if (parallelStatistics.nestedSerialExecution)
        {
            ++statistics->meshBuildNestedSerialCallCount;
        }

        for (std::size_t taskIndex = 0;
             taskIndex < directionTasks.size();
             ++taskIndex)
        {
            const MyVoxel::VoxelSurfaceMeshingStatistics& taskStatistics =
                directionTasks[taskIndex].meshingStatistics;

            statistics->facePlaneBuildCpuMilliseconds +=
                taskStatistics.facePlaneBuildMilliseconds;
            statistics->greedyMergeCpuMilliseconds +=
                taskStatistics.greedyMergeMilliseconds;
            statistics->meshedFaceCount +=
                taskStatistics.sourceFaceCount;
            statistics->nonEmptyPlaneCount +=
                taskStatistics.nonEmptyPlaneCount;
            statistics->mergedQuadCount +=
                taskStatistics.mergedQuadCount;
        }
    }
}

// 根据指定根集合准备只重新构建网格的任务。
void prepareRootMeshBuildItems(const MyVoxel::VoxelSurfaceCache::RootEntryMap& roots,
                               const MyVoxel::VoxelSurfaceCache::RootIndexSet& rootIndices,
                               std::vector<RootMeshBuildItem>& items)
{
    items.clear();
    items.reserve(rootIndices.size());

    for (MyVoxel::VoxelSurfaceCache::RootIndexSet::const_iterator rootIterator = rootIndices.begin();
         rootIterator != rootIndices.end();
         ++rootIterator)
    {
        const MyVoxel::VoxelSurfaceCache::RootEntryMap::const_iterator entryIterator =
            roots.find(*rootIterator);

        if (entryIterator == roots.end() || entryIterator->second.faceMasks.isEmpty())
        {
            continue;
        }

        items.push_back(
            RootMeshBuildItem(
                *rootIterator,
                &entryIterator->second.faceMasks,
                &entryIterator->second.colors,
                entryIterator->second.directionQuadCountHints));
    }
}

// 将已经成功构建的根网格和包围盒提交到现有根缓存。
void commitRootMeshBuildItems(MyVoxel::VoxelSurfaceCache::RootEntryMap& roots,
                              std::vector<RootMeshBuildItem>& items)
{
    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        RootMeshBuildItem& item = items[itemIndex];

        MyVoxel::VoxelSurfaceCache::RootEntryMap::iterator entryIterator =
            roots.find(item.rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            entryIterator != roots.end(),
            "Voxel surface cache root disappeared before mesh result submission.");

        item.directionMeshPlan.commit(
            entryIterator->second.directionMeshes,
            entryIterator->second.directionMeshVersions);
        entryIterator->second.invalidateCombinedMesh();
        entryIterator->second.localBounds = item.localBounds;
        entryIterator->second.directionQuadCountHints =
            item.directionQuadCountHints;
    }
}

// 将已经成功构建的局部颜色和根网格提交到现有根缓存。
void commitRootColorChanges(MyVoxel::VoxelSurfaceCache::RootEntryMap& roots,
                            std::vector<RootColorChange>& changes)
{
    for (std::size_t changeIndex = 0; changeIndex < changes.size(); ++changeIndex)
    {
        RootColorChange& change = changes[changeIndex];

        MyVoxel::VoxelSurfaceCache::RootEntryMap::iterator entryIterator =
            roots.find(change.rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            entryIterator != roots.end(),
            "Voxel surface cache root disappeared before color result submission.");

        entryIterator->second.colors = std::move(change.colors);
        change.directionMeshPlan.commit(
            entryIterator->second.directionMeshes,
            entryIterator->second.directionMeshVersions);
        entryIterator->second.invalidateCombinedMesh();
        entryIterator->second.localBounds = change.localBounds;
        entryIterator->second.directionQuadCountHints =
            change.directionQuadCountHints;
    }
}

}

namespace MyVoxel
{

bool VoxelSurfaceCacheUpdate::hasSurfaceChanges() const
{
    return !addedFaces.isEmpty() || !removedFaces.isEmpty();
}

VoxelSurfaceUpdateStatistics::VoxelSurfaceUpdateStatistics()
{
    clear();
}

void VoxelSurfaceUpdateStatistics::clear()
{
    affectedRootCollectionMilliseconds = 0.0;
    dirtyRegionBuildMilliseconds = 0.0;
    faceExtractionMilliseconds = 0.0;
    occupancyBuildMilliseconds = 0.0;
    occupancyBuildCpuMilliseconds = 0.0;
    faceGenerationMilliseconds = 0.0;
    faceGenerationCpuMilliseconds = 0.0;
    faceSetBuildMilliseconds = 0.0;
    oldFaceCopyMilliseconds = 0.0;
    faceComparisonMilliseconds = 0.0;
    faceDifferenceMilliseconds = 0.0;
    faceDifferenceCpuMilliseconds = 0.0;
    faceDifferenceMergeMilliseconds = 0.0;
    colorUpdateMilliseconds = 0.0;
    meshBuildWallMilliseconds = 0.0;
    rootColorPrepareMilliseconds = 0.0;
    facePlaneBuildCpuMilliseconds = 0.0;
    greedyMergeCpuMilliseconds = 0.0;
    directionFinalizeMilliseconds = 0.0;
    directionMeshPlanMilliseconds = 0.0;
    cacheCommitMilliseconds = 0.0;
    boundsUpdateMilliseconds = 0.0;
    totalMilliseconds = 0.0;

    affectedRootCount = 0;
    changedRootCount = 0;
    colorUpdatedRootCount = 0;
    colorCopiedEntryCount = 0;
    occupancySourceRequestCount = 0;
    uniqueOccupancyRootCount = 0;
    builtOccupancyRootCount = 0;
    occupancyIncrementalUpdateCount = 0;
    occupancyFullRebuildCount = 0;
    occupancyRemovedRootCount = 0;
    dirtyLeafBlockCount = 0;
    expandedDirtyLeafBlockCount = 0;
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
    faceMaskIncrementalRootCount = 0;
    faceMaskFullRebuildRootCount = 0;
    faceMaskUpdatedWordCount = 0;
    faceMaskUpdatedCellCount = 0;

    faceDifferenceCallCount = 0;
    faceDifferenceTaskCount = 0;
    faceDifferenceParticipantCount = 0;
    faceDifferenceBatchSizeTotal = 0;
    faceDifferenceSerialCallCount = 0;
    faceDifferenceNestedSerialCallCount = 0;

    emittedFaceCount = 0;
    meshedFaceCount = 0;
    nonEmptyPlaneCount = 0;
    mergedQuadCount = 0;
    directionMeshHintedTaskCount = 0;
    directionMeshReservedQuadCount = 0;
    directionMeshHintUnderestimateCount = 0;
    directionMeshUnusedReservedQuadCount = 0;
    directionMeshConsideredCount = 0;
    directionMeshReusedCount = 0;
    directionMeshClearedCount = 0;
    directionMeshReusedQuadCount = 0;

    meshBuildCallCount = 0;
    meshBuildTaskCount = 0;
    meshBuildParticipantCount = 0;
    meshBuildBatchSizeTotal = 0;
    meshBuildSerialCallCount = 0;
    meshBuildNestedSerialCallCount = 0;
}

void VoxelSurfaceUpdateStatistics::add(const VoxelSurfaceUpdateStatistics& other)
{
    affectedRootCollectionMilliseconds += other.affectedRootCollectionMilliseconds;
    dirtyRegionBuildMilliseconds += other.dirtyRegionBuildMilliseconds;
    faceExtractionMilliseconds += other.faceExtractionMilliseconds;
    occupancyBuildMilliseconds += other.occupancyBuildMilliseconds;
    occupancyBuildCpuMilliseconds += other.occupancyBuildCpuMilliseconds;
    faceGenerationMilliseconds += other.faceGenerationMilliseconds;
    faceGenerationCpuMilliseconds += other.faceGenerationCpuMilliseconds;
    faceSetBuildMilliseconds += other.faceSetBuildMilliseconds;
    oldFaceCopyMilliseconds += other.oldFaceCopyMilliseconds;
    faceComparisonMilliseconds += other.faceComparisonMilliseconds;
    faceDifferenceMilliseconds += other.faceDifferenceMilliseconds;
    faceDifferenceCpuMilliseconds += other.faceDifferenceCpuMilliseconds;
    faceDifferenceMergeMilliseconds += other.faceDifferenceMergeMilliseconds;
    colorUpdateMilliseconds += other.colorUpdateMilliseconds;
    meshBuildWallMilliseconds += other.meshBuildWallMilliseconds;
    rootColorPrepareMilliseconds += other.rootColorPrepareMilliseconds;
    facePlaneBuildCpuMilliseconds += other.facePlaneBuildCpuMilliseconds;
    greedyMergeCpuMilliseconds += other.greedyMergeCpuMilliseconds;
    directionFinalizeMilliseconds += other.directionFinalizeMilliseconds;
    directionMeshPlanMilliseconds += other.directionMeshPlanMilliseconds;
    cacheCommitMilliseconds += other.cacheCommitMilliseconds;
    boundsUpdateMilliseconds += other.boundsUpdateMilliseconds;
    totalMilliseconds += other.totalMilliseconds;

    affectedRootCount += other.affectedRootCount;
    changedRootCount += other.changedRootCount;
    colorUpdatedRootCount += other.colorUpdatedRootCount;
    colorCopiedEntryCount += other.colorCopiedEntryCount;
    occupancySourceRequestCount += other.occupancySourceRequestCount;
    uniqueOccupancyRootCount += other.uniqueOccupancyRootCount;
    builtOccupancyRootCount += other.builtOccupancyRootCount;
    occupancyIncrementalUpdateCount += other.occupancyIncrementalUpdateCount;
    occupancyFullRebuildCount += other.occupancyFullRebuildCount;
    occupancyRemovedRootCount += other.occupancyRemovedRootCount;
    dirtyLeafBlockCount += other.dirtyLeafBlockCount;
    expandedDirtyLeafBlockCount += other.expandedDirtyLeafBlockCount;
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
    faceMaskIncrementalRootCount += other.faceMaskIncrementalRootCount;
    faceMaskFullRebuildRootCount += other.faceMaskFullRebuildRootCount;
    faceMaskUpdatedWordCount += other.faceMaskUpdatedWordCount;
    faceMaskUpdatedCellCount += other.faceMaskUpdatedCellCount;

    faceDifferenceCallCount += other.faceDifferenceCallCount;
    faceDifferenceTaskCount += other.faceDifferenceTaskCount;
    faceDifferenceParticipantCount += other.faceDifferenceParticipantCount;
    faceDifferenceBatchSizeTotal += other.faceDifferenceBatchSizeTotal;
    faceDifferenceSerialCallCount += other.faceDifferenceSerialCallCount;
    faceDifferenceNestedSerialCallCount += other.faceDifferenceNestedSerialCallCount;

    emittedFaceCount += other.emittedFaceCount;
    meshedFaceCount += other.meshedFaceCount;
    nonEmptyPlaneCount += other.nonEmptyPlaneCount;
    mergedQuadCount += other.mergedQuadCount;
    directionMeshHintedTaskCount += other.directionMeshHintedTaskCount;
    directionMeshReservedQuadCount += other.directionMeshReservedQuadCount;
    directionMeshHintUnderestimateCount += other.directionMeshHintUnderestimateCount;
    directionMeshUnusedReservedQuadCount += other.directionMeshUnusedReservedQuadCount;
    directionMeshConsideredCount += other.directionMeshConsideredCount;
    directionMeshReusedCount += other.directionMeshReusedCount;
    directionMeshClearedCount += other.directionMeshClearedCount;
    directionMeshReusedQuadCount += other.directionMeshReusedQuadCount;

    meshBuildCallCount += other.meshBuildCallCount;
    meshBuildTaskCount += other.meshBuildTaskCount;
    meshBuildParticipantCount += other.meshBuildParticipantCount;
    meshBuildBatchSizeTotal += other.meshBuildBatchSizeTotal;
    meshBuildSerialCallCount += other.meshBuildSerialCallCount;
    meshBuildNestedSerialCallCount += other.meshBuildNestedSerialCallCount;
}

VoxelSurfaceCache::RootEntry::RootEntry()
    : directionMeshes()
    , directionMeshVersions()
    , directionQuadCountHints()
    , m_combinedMesh()
    , m_combinedMeshValid(false)
{
    directionMeshVersions.fill(
        static_cast<std::uint64_t>(0));
    directionQuadCountHints.fill(
        static_cast<std::size_t>(0));
}

void VoxelSurfaceCache::RootEntry::invalidateCombinedMesh()
{
    m_combinedMesh = Geometry::Mesh();
    m_combinedMeshValid = false;
}

const Geometry::Mesh& VoxelSurfaceCache::RootEntry::combinedMesh() const
{
    if (m_combinedMeshValid)
    {
        return m_combinedMesh;
    }

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();
    std::size_t vertexCount = 0;
    std::size_t indexCount = 0;

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        const Geometry::Mesh& directionMesh =
            directionMeshes[directionValue];

        MYVOXEL_ASSERT_MESSAGE(
            directionMesh.isValid(),
            "Voxel surface compatibility Root Mesh requires valid direction meshes.");
        MYVOXEL_ASSERT_MESSAGE(
            directionMesh.vertexCount() <=
                maximumSize - vertexCount,
            "Voxel surface compatibility Root vertex count overflowed.");
        MYVOXEL_ASSERT_MESSAGE(
            directionMesh.indexCount() <=
                maximumSize - indexCount,
            "Voxel surface compatibility Root index count overflowed.");

        vertexCount += directionMesh.vertexCount();
        indexCount += directionMesh.indexCount();
    }

    m_combinedMesh = Geometry::Mesh();
    m_combinedMesh.reserve(
        vertexCount,
        indexCount);

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        const Geometry::Mesh& directionMesh =
            directionMeshes[directionValue];

        if (!directionMesh.isEmpty())
        {
            m_combinedMesh.append(directionMesh);
        }
    }

    MYVOXEL_ASSERT_MESSAGE(
        m_combinedMesh.isValid(),
        "Voxel surface compatibility Root Mesh is invalid.");

    m_combinedMeshValid = true;
    return m_combinedMesh;
}

VoxelSurfaceCache::VoxelSurfaceCache()
    : m_initialized(false)
    , m_combinedFaceColorsDirty(false)
{
}

/// 缓存管理

void VoxelSurfaceCache::clear()
{
    m_initialized = false;
    m_occupancies.clear();
    m_roots.clear();
    m_combinedFaceColors.clear();
    m_combinedFaceColorsDirty = false;
    m_localBounds = Bounds3();
}

void VoxelSurfaceCache::rebuild(
    const VoxelShape& shape,
    const Geometry::MeshColor& defaultColorValue)
{
    MYVOXEL_ASSERT_MESSAGE(
        shape.isValid(),
        "Voxel surface cache rebuild requires a valid VoxelShape.");

    clear();

    try
    {
        m_grid = shape.grid();
        m_defaultColor = defaultColorValue;

        std::vector<VoxelCellIndex> rootIndexArray;

        shape.forest().forEachRootCell(
            [&](const VoxelCellAddress& rootAddress)
            {
                rootIndexArray.push_back(rootAddress.index);
            });

        VoxelFaceExtractor::extractRootOccupancies(
            shape,
            rootIndexArray,
            m_occupancies);

        std::vector<VoxelRootFaceMasks> rootFaceMasks;
        VoxelFaceExtractor::buildRootMasks(
            m_occupancies,
            rootIndexArray,
            rootFaceMasks,
            nullptr);

        MYVOXEL_ASSERT_MESSAGE(
            rootFaceMasks.size() == rootIndexArray.size(),
            "Voxel surface cache rebuild received an invalid root mask count.");

        RootIndexSet rootIndices;

        for (std::size_t rootPosition = 0;
             rootPosition < rootIndexArray.size();
             ++rootPosition)
        {
            VoxelRootFaceMasks& faceMasks = rootFaceMasks[rootPosition];

            if (faceMasks.isEmpty())
            {
                continue;
            }

            RootEntry entry;
            entry.faceMasks = std::move(faceMasks);

            const std::pair<RootEntryMap::iterator, bool> inserted =
                m_roots.insert(
                    std::make_pair(
                        rootIndexArray[rootPosition],
                        std::move(entry)));

            MYVOXEL_ASSERT_MESSAGE(
                inserted.second,
                "Voxel surface cache rebuild encountered a duplicate root index.");

            rootIndices.insert(rootIndexArray[rootPosition]);
        }

        rebuildRootMeshes(shape, rootIndices);
        m_initialized = true;
    }
    catch (...)
    {
        clear();
        throw;
    }
}

VoxelSurfaceCacheUpdate VoxelSurfaceCache::update(const VoxelShape& shape,
                                                  const VoxelChangeSet& changes,
                                                  const Geometry::MeshColor& newFaceColor)
{
    return update(shape, changes, newFaceColor, nullptr);
}

VoxelSurfaceCacheUpdate VoxelSurfaceCache::update(const VoxelShape& shape,
                                                  const VoxelChangeSet& changes,
                                                  const Geometry::MeshColor& newFaceColor,
                                                  VoxelSurfaceUpdateStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface cache update requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        isCompatible(shape),
        "Voxel surface cache update requires a cache built from the same VoxelGrid.");

    if (statistics)
    {
        statistics->clear();
    }

    Foundation::Stopwatch totalTimer;
    VoxelSurfaceCacheUpdate result;

    if (!changes.hasChanges())
    {
        if (statistics)
        {
            statistics->totalMilliseconds = totalTimer.elapsedMilliseconds();
        }

        return result;
    }

    Foundation::Stopwatch stageTimer;
    Foundation::Stopwatch extractionTimer;
    std::vector<PendingRootOccupancyChange> pendingOccupancyChanges;
    PendingRootOccupancyPositionMap pendingOccupancyPositions;

    preparePendingRootOccupancies(
        shape,
        changes,
        m_occupancies,
        pendingOccupancyChanges,
        pendingOccupancyPositions,
        statistics);

    SurfaceDirtyCellRegionMap expandedDirtyRegions;

    stageTimer.restart();

    buildExpandedDirtyLeafRegions(
        shape,
        changes,
        m_occupancies,
        pendingOccupancyChanges,
        pendingOccupancyPositions,
        m_roots,
        expandedDirtyRegions,
        result.affectedRootIndices,
        statistics);

    if (statistics)
    {
        statistics->affectedRootCollectionMilliseconds +=
            stageTimer.elapsedMilliseconds();
    }

    std::vector<RootSurfaceChange> rootChanges;
    rootChanges.reserve(result.affectedRootIndices.size());

    std::vector<VoxelCellIndex> affectedRootArray;
    std::vector<VoxelRootFaceMasks> extractedRootMasks;

    updateIncrementalRootFaceMasks(
        shape,
        m_occupancies,
        pendingOccupancyChanges,
        pendingOccupancyPositions,
        m_roots,
        expandedDirtyRegions,
        affectedRootArray,
        extractedRootMasks,
        statistics);

    if (statistics)
    {
        statistics->faceExtractionMilliseconds +=
            extractionTimer.elapsedMilliseconds();
    }

    MYVOXEL_ASSERT_MESSAGE(
        extractedRootMasks.size() == affectedRootArray.size(),
        "Incremental Root face mask update returned an invalid result count.");

    std::vector<RootFaceDifferenceTask> differenceTasks;
    differenceTasks.reserve(affectedRootArray.size());

    for (std::size_t rootPosition = 0;
         rootPosition < affectedRootArray.size();
         ++rootPosition)
    {
        const VoxelCellIndex& rootIndex =
            affectedRootArray[rootPosition];
        const RootEntryMap::const_iterator entryIterator =
            m_roots.find(rootIndex);
        const VoxelRootFaceMasks& newFaceMasks =
            extractedRootMasks[rootPosition];

        stageTimer.restart();

        const bool facesChanged =
            entryIterator == m_roots.end()
                ? !newFaceMasks.isEmpty()
                : newFaceMasks !=
                    entryIterator->second.faceMasks;

        if (statistics)
        {
            statistics->faceComparisonMilliseconds +=
                stageTimer.elapsedMilliseconds();
        }

        if (!facesChanged)
        {
            continue;
        }

        result.changedRootIndices.insert(rootIndex);
        differenceTasks.push_back(
            RootFaceDifferenceTask());

        RootFaceDifferenceTask& task =
            differenceTasks.back();

        task.rootPosition = rootPosition;
        task.newFaceMasks =
            &extractedRootMasks[rootPosition];
        task.oldFaceMasks =
            entryIterator == m_roots.end()
                ? nullptr
                : &entryIterator->second.faceMasks;
    }

    buildRootFaceDifferences(
        differenceTasks,
        statistics);

    stageTimer.restart();

    std::size_t totalAddedFaceCount = 0;
    std::size_t totalRemovedFaceCount = 0;
    const std::size_t maximumFaceCount =
        (std::numeric_limits<std::size_t>::max)();

    for (std::size_t taskIndex = 0;
         taskIndex < differenceTasks.size();
         ++taskIndex)
    {
        const RootFaceDifferenceTask& task =
            differenceTasks[taskIndex];

        MYVOXEL_ASSERT_MESSAGE(
            task.addedFaces.size() <=
                maximumFaceCount - totalAddedFaceCount,
            "Voxel surface added face count overflowed.");
        MYVOXEL_ASSERT_MESSAGE(
            task.removedFaces.size() <=
                maximumFaceCount - totalRemovedFaceCount,
            "Voxel surface removed face count overflowed.");

        totalAddedFaceCount +=
            task.addedFaces.size();
        totalRemovedFaceCount +=
            task.removedFaces.size();
    }

    if (statistics)
    {
        statistics->faceDifferenceMilliseconds +=
            stageTimer.elapsedMilliseconds();
    }

    for (std::size_t taskIndex = 0;
         taskIndex < differenceTasks.size();
         ++taskIndex)
    {
        RootFaceDifferenceTask& task =
            differenceTasks[taskIndex];

        MYVOXEL_ASSERT_MESSAGE(
            task.rootPosition < affectedRootArray.size() &&
            task.rootPosition < extractedRootMasks.size(),
            "Voxel surface Root difference task position is invalid.");

        const VoxelCellIndex& rootIndex =
            affectedRootArray[task.rootPosition];
        const RootEntryMap::const_iterator entryIterator =
            m_roots.find(rootIndex);
        VoxelRootFaceMasks& newFaceMasks =
            extractedRootMasks[task.rootPosition];

        stageTimer.restart();

        const VoxelFaceSet& addedRootFaces =
            task.addedFaces;
        const VoxelFaceSet& removedRootFaces =
            task.removedFaces;

        if (statistics)
        {
            statistics->faceDifferenceMilliseconds +=
                stageTimer.elapsedMilliseconds();
        }

        rootChanges.push_back(
            RootSurfaceChange(
                rootIndex,
                std::move(newFaceMasks),
                entryIterator != m_roots.end()
                    ? &entryIterator->second.faceMasks
                    : nullptr,
                entryIterator != m_roots.end()
                    ? &entryIterator->second.directionMeshes
                    : nullptr,
                entryIterator != m_roots.end()
                    ? entryIterator->second.directionQuadCountHints
                    : VoxelSurfaceCache::DirectionQuadCountHints()));

        RootSurfaceChange& change =
            rootChanges.back();

        stageTimer.restart();

        if (entryIterator != m_roots.end())
        {
            change.colors =
                entryIterator->second.colors;

            if (statistics)
            {
                statistics->colorCopiedEntryCount +=
                    entryIterator->second.colors.size();
            }
        }

        change.colors.erase(removedRootFaces);
        change.colors.set(
            addedRootFaces,
            newFaceColor);

        if (statistics)
        {
            statistics->colorUpdateMilliseconds +=
                stageTimer.elapsedMilliseconds();
            ++statistics->colorUpdatedRootCount;
        }
    }

    stageTimer.restart();

    result.addedFaces =
        mergeRootFaceDifferences(
            differenceTasks,
            RootFaceDifferenceKind::Added,
            totalAddedFaceCount);
    result.removedFaces =
        mergeRootFaceDifferences(
            differenceTasks,
            RootFaceDifferenceKind::Removed,
            totalRemovedFaceCount);

    if (statistics)
    {
        const double mergeMilliseconds =
            stageTimer.elapsedMilliseconds();

        statistics->faceDifferenceMilliseconds +=
            mergeMilliseconds;
        statistics->faceDifferenceMergeMilliseconds +=
            mergeMilliseconds;
        statistics->changedRootCount =
            result.changedRootIndices.size();
    }

    if (!rootChanges.empty())
    {
        // 所有根网格均使用各自准备好的局部颜色构建，失败时当前缓存保持不变。
        buildRootMeshes(shape, rootChanges, m_defaultColor, statistics);
    }

    stageTimer.restart();

    // 提前为新增占用和非空表面Root分配map节点，任一分配失败时回滚全部空占位项。
    std::vector<VoxelCellIndex> insertedOccupancyIndices;
    std::vector<VoxelCellIndex> insertedRootIndices;

    insertedOccupancyIndices.reserve(
        pendingOccupancyChanges.size());
    insertedRootIndices.reserve(
        rootChanges.size());

    try
    {
        for (std::size_t changeIndex = 0;
             changeIndex < pendingOccupancyChanges.size();
             ++changeIndex)
        {
            const PendingRootOccupancyChange& change =
                pendingOccupancyChanges[changeIndex];

            if (!change.hasOccupancy ||
                m_occupancies.find(change.rootIndex) != m_occupancies.end())
            {
                continue;
            }

            const std::pair<VoxelRootOccupancyMap::iterator, bool> inserted =
                m_occupancies.insert(
                    std::make_pair(
                        change.rootIndex,
                        VoxelRootOccupancy()));

            MYVOXEL_ASSERT_MESSAGE(
                inserted.second,
                "Voxel surface cache failed to reserve a new Root occupancy entry.");

            insertedOccupancyIndices.push_back(
                change.rootIndex);
        }

        for (std::size_t changeIndex = 0;
             changeIndex < rootChanges.size();
             ++changeIndex)
        {
            const RootSurfaceChange& change =
                rootChanges[changeIndex];

            if (change.faceMasks.isEmpty() ||
                m_roots.find(change.rootIndex) != m_roots.end())
            {
                continue;
            }

            const std::pair<RootEntryMap::iterator, bool> inserted =
                m_roots.insert(
                    std::make_pair(
                        change.rootIndex,
                        RootEntry()));

            MYVOXEL_ASSERT_MESSAGE(
                inserted.second,
                "Voxel surface cache failed to reserve a new root entry.");

            insertedRootIndices.push_back(
                change.rootIndex);
        }
    }
    catch (...)
    {
        for (std::size_t rootPosition = 0;
             rootPosition < insertedRootIndices.size();
             ++rootPosition)
        {
            m_roots.erase(
                insertedRootIndices[rootPosition]);
        }

        for (std::size_t rootPosition = 0;
             rootPosition < insertedOccupancyIndices.size();
             ++rootPosition)
        {
            m_occupancies.erase(
                insertedOccupancyIndices[rootPosition]);
        }

        throw;
    }

    // 占用和全部网格均成功准备后，先提交所有实际Root的核心占用缓存。
    for (std::size_t changeIndex = 0;
         changeIndex < pendingOccupancyChanges.size();
         ++changeIndex)
    {
        PendingRootOccupancyChange& change =
            pendingOccupancyChanges[changeIndex];

        if (!change.hasOccupancy)
        {
            m_occupancies.erase(change.rootIndex);
            continue;
        }

        VoxelRootOccupancyMap::iterator occupancyIterator =
            m_occupancies.find(change.rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            occupancyIterator != m_occupancies.end(),
            "Voxel surface cache Root occupancy disappeared before submission.");

        occupancyIterator->second =
            std::move(change.occupancy);
    }

    if (rootChanges.empty())
    {
        if (statistics)
        {
            statistics->cacheCommitMilliseconds +=
                stageTimer.elapsedMilliseconds();
            statistics->totalMilliseconds =
                totalTimer.elapsedMilliseconds();
        }

        return result;
    }

    // 网格、面掩码和Root局部颜色均准备完成后统一提交根状态。
    for (std::size_t changeIndex = 0; changeIndex < rootChanges.size(); ++changeIndex)
    {
        RootSurfaceChange& change = rootChanges[changeIndex];

        if (change.faceMasks.isEmpty())
        {
            m_roots.erase(change.rootIndex);
            continue;
        }

        RootEntryMap::iterator entryIterator = m_roots.find(change.rootIndex);

        MYVOXEL_ASSERT_MESSAGE(
            entryIterator != m_roots.end(),
            "Voxel surface cache root disappeared before update submission.");

        entryIterator->second.faceMasks = std::move(change.faceMasks);
        entryIterator->second.colors = std::move(change.colors);
        change.directionMeshPlan.commit(
            entryIterator->second.directionMeshes,
            entryIterator->second.directionMeshVersions);
        entryIterator->second.invalidateCombinedMesh();
        entryIterator->second.localBounds = change.localBounds;
        entryIterator->second.directionQuadCountHints =
            change.directionQuadCountHints;
    }

    invalidateCombinedFaceColors();

    if (statistics)
    {
        statistics->cacheCommitMilliseconds = stageTimer.elapsedMilliseconds();
    }

    stageTimer.restart();
    rebuildLocalBounds();

    if (statistics)
    {
        statistics->boundsUpdateMilliseconds = stageTimer.elapsedMilliseconds();
        statistics->totalMilliseconds = totalTimer.elapsedMilliseconds();
    }

    return result;
}

/// 颜色修改

VoxelSurfaceCache::RootIndexSet VoxelSurfaceCache::setFaceColor(
    const VoxelShape& shape,
    const VoxelFaceSet& faces,
    const Geometry::MeshColor& color)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface cache face coloring requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        isCompatible(shape),
        "Voxel surface cache face coloring requires a cache built from the same VoxelGrid.");

    using RootFaceArrayMap = std::map<VoxelCellIndex, VoxelFaceSet::Container>;

    RootFaceArrayMap rootFaceArrays;

    for (VoxelFaceSet::ConstIterator faceIterator = faces.begin(); faceIterator != faces.end(); ++faceIterator)
    {
        const VoxelCellIndex rootIndex = rootIndexOfFace(m_grid, *faceIterator);
        const RootEntryMap::const_iterator entryIterator = m_roots.find(rootIndex);

        if (entryIterator == m_roots.end() || !entryIterator->second.faceMasks.contains(*faceIterator))
        {
            continue;
        }

        rootFaceArrays[rootIndex].push_back(*faceIterator);
    }

    RootIndexSet changedRootIndices;
    std::vector<RootColorChange> colorChanges;
    colorChanges.reserve(rootFaceArrays.size());

    for (RootFaceArrayMap::iterator groupIterator = rootFaceArrays.begin();
         groupIterator != rootFaceArrays.end();
         ++groupIterator)
    {
        RootEntryMap::const_iterator entryIterator = m_roots.find(groupIterator->first);

        MYVOXEL_ASSERT_MESSAGE(
            entryIterator != m_roots.end(),
            "Voxel surface color group lost its owning root.");

        VoxelFaceSet rootFaces = VoxelFaceSet::fromSortedUnique(std::move(groupIterator->second));

        colorChanges.push_back(
            RootColorChange(
                groupIterator->first,
                &entryIterator->second.faceMasks,
                entryIterator->second.colors,
                entryIterator->second.directionQuadCountHints));

        RootColorChange& change = colorChanges.back();

        if (change.colors.set(rootFaces, color) == 0)
        {
            colorChanges.pop_back();
            continue;
        }

        changedRootIndices.insert(groupIterator->first);
    }

    if (colorChanges.empty())
    {
        return changedRootIndices;
    }

    buildRootMeshes(shape, colorChanges, m_defaultColor, nullptr);
    commitRootColorChanges(m_roots, colorChanges);
    invalidateCombinedFaceColors();

    return changedRootIndices;
}

VoxelSurfaceCache::RootIndexSet VoxelSurfaceCache::eraseFaceColor(
    const VoxelShape& shape,
    const VoxelFaceSet& faces)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface cache face color removal requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        isCompatible(shape),
        "Voxel surface cache face color removal requires a cache built from the same VoxelGrid.");

    using RootFaceArrayMap = std::map<VoxelCellIndex, VoxelFaceSet::Container>;

    RootFaceArrayMap rootFaceArrays;

    for (VoxelFaceSet::ConstIterator faceIterator = faces.begin(); faceIterator != faces.end(); ++faceIterator)
    {
        const VoxelCellIndex rootIndex = rootIndexOfFace(m_grid, *faceIterator);
        const RootEntryMap::const_iterator entryIterator = m_roots.find(rootIndex);

        if (entryIterator == m_roots.end() || !entryIterator->second.colors.contains(*faceIterator))
        {
            continue;
        }

        rootFaceArrays[rootIndex].push_back(*faceIterator);
    }

    RootIndexSet changedRootIndices;
    std::vector<RootColorChange> colorChanges;
    colorChanges.reserve(rootFaceArrays.size());

    for (RootFaceArrayMap::iterator groupIterator = rootFaceArrays.begin();
         groupIterator != rootFaceArrays.end();
         ++groupIterator)
    {
        RootEntryMap::const_iterator entryIterator = m_roots.find(groupIterator->first);

        MYVOXEL_ASSERT_MESSAGE(
            entryIterator != m_roots.end(),
            "Voxel surface color removal group lost its owning root.");

        VoxelFaceSet rootFaces = VoxelFaceSet::fromSortedUnique(std::move(groupIterator->second));

        colorChanges.push_back(
            RootColorChange(
                groupIterator->first,
                &entryIterator->second.faceMasks,
                entryIterator->second.colors,
                entryIterator->second.directionQuadCountHints));

        RootColorChange& change = colorChanges.back();

        if (change.colors.erase(rootFaces) == 0)
        {
            colorChanges.pop_back();
            continue;
        }

        changedRootIndices.insert(groupIterator->first);
    }

    if (colorChanges.empty())
    {
        return changedRootIndices;
    }

    buildRootMeshes(shape, colorChanges, m_defaultColor, nullptr);
    commitRootColorChanges(m_roots, colorChanges);
    invalidateCombinedFaceColors();

    return changedRootIndices;
}

VoxelSurfaceCache::RootIndexSet VoxelSurfaceCache::clearFaceColors(const VoxelShape& shape)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface cache face color clearing requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        isCompatible(shape),
        "Voxel surface cache face color clearing requires a cache built from the same VoxelGrid.");

    RootIndexSet changedRootIndices;
    std::vector<RootColorChange> colorChanges;

    for (RootEntryMap::const_iterator entryIterator = m_roots.begin();
         entryIterator != m_roots.end();
         ++entryIterator)
    {
        if (entryIterator->second.colors.isEmpty())
        {
            continue;
        }

        colorChanges.push_back(
            RootColorChange(
                entryIterator->first,
                &entryIterator->second.faceMasks,
                entryIterator->second.colors,
                entryIterator->second.directionQuadCountHints));

        colorChanges.back().colors.clear();
        changedRootIndices.insert(entryIterator->first);
    }

    if (colorChanges.empty())
    {
        return changedRootIndices;
    }

    buildRootMeshes(shape, colorChanges, m_defaultColor, nullptr);
    commitRootColorChanges(m_roots, colorChanges);
    invalidateCombinedFaceColors();

    return changedRootIndices;
}

VoxelSurfaceCache::RootIndexSet VoxelSurfaceCache::setDefaultColor(
    const VoxelShape& shape,
    const Geometry::MeshColor& color)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Voxel surface cache default coloring requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(
        isCompatible(shape),
        "Voxel surface cache default coloring requires a cache built from the same VoxelGrid.");

    RootIndexSet changedRootIndices;

    if (m_defaultColor == color)
    {
        return changedRootIndices;
    }

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        changedRootIndices.insert(iterator->first);
    }

    std::vector<RootMeshBuildItem> buildItems;
    prepareRootMeshBuildItems(m_roots, changedRootIndices, buildItems);
    buildRootMeshes(shape, buildItems, color, nullptr);

    m_defaultColor = color;
    commitRootMeshBuildItems(m_roots, buildItems);

    return changedRootIndices;
}

/// 缓存状态

bool VoxelSurfaceCache::isInitialized() const
{
    return m_initialized;
}

bool VoxelSurfaceCache::isEmpty() const
{
    return m_roots.empty();
}

std::size_t VoxelSurfaceCache::rootCount() const
{
    return m_roots.size();
}

std::size_t VoxelSurfaceCache::faceCount() const
{
    std::size_t count = 0;

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        count += iterator->second.faceMasks.faceCount();
    }

    return count;
}

std::size_t VoxelSurfaceCache::vertexCount() const
{
    std::size_t count = 0;

    for (RootEntryMap::const_iterator iterator = m_roots.begin();
         iterator != m_roots.end();
         ++iterator)
    {
        for (unsigned int directionValue = 0;
             directionValue < VoxelFaceDirectionCount;
             ++directionValue)
        {
            count +=
                iterator->second.directionMeshes[directionValue].vertexCount();
        }
    }

    return count;
}

std::size_t VoxelSurfaceCache::triangleCount() const
{
    std::size_t count = 0;

    for (RootEntryMap::const_iterator iterator = m_roots.begin();
         iterator != m_roots.end();
         ++iterator)
    {
        for (unsigned int directionValue = 0;
             directionValue < VoxelFaceDirectionCount;
             ++directionValue)
        {
            count +=
                iterator->second.directionMeshes[directionValue].triangleCount();
        }
    }

    return count;
}

const Geometry::MeshColor& VoxelSurfaceCache::defaultColor() const
{
    return m_defaultColor;
}

const Bounds3& VoxelSurfaceCache::localBounds() const
{
    return m_localBounds;
}

/// 根缓存访问

bool VoxelSurfaceCache::containsRoot(const VoxelCellIndex& rootIndex) const
{
    return m_roots.find(rootIndex) != m_roots.end();
}

const VoxelSurfaceCache::RootEntry* VoxelSurfaceCache::rootEntry(const VoxelCellIndex& rootIndex) const
{
    const RootEntryMap::const_iterator iterator = m_roots.find(rootIndex);
    return iterator == m_roots.end() ? nullptr : &iterator->second;
}

VoxelFaceSet VoxelSurfaceCache::rootFaces(const VoxelCellIndex& rootIndex) const
{
    const RootEntry* entry = rootEntry(rootIndex);
    return entry ? entry->faceMasks.toFaceSet() : VoxelFaceSet();
}

const Geometry::Mesh* VoxelSurfaceCache::rootMesh(
    const VoxelCellIndex& rootIndex) const
{
    const RootEntry* entry = rootEntry(rootIndex);
    return entry ? &entry->combinedMesh() : nullptr;
}

const VoxelSurfaceCache::DirectionMeshes*
VoxelSurfaceCache::rootDirectionMeshes(
    const VoxelCellIndex& rootIndex) const
{
    const RootEntry* entry = rootEntry(rootIndex);
    return entry ? &entry->directionMeshes : nullptr;
}

const Geometry::Mesh* VoxelSurfaceCache::rootDirectionMesh(
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel surface cache direction Mesh query received an invalid direction.");

    const RootEntry* entry = rootEntry(rootIndex);

    return entry
        ? &entry->directionMeshes[
            static_cast<std::size_t>(direction)]
        : nullptr;
}

std::uint64_t VoxelSurfaceCache::rootDirectionMeshVersion(
    const VoxelCellIndex& rootIndex,
    VoxelFaceDirection direction) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel surface cache direction Mesh version query received an invalid direction.");

    const RootEntry* entry = rootEntry(rootIndex);

    return entry
        ? entry->directionMeshVersions[
            static_cast<std::size_t>(direction)]
        : static_cast<std::uint64_t>(0);
}

const Bounds3* VoxelSurfaceCache::rootBounds(const VoxelCellIndex& rootIndex) const
{
    const RootEntry* entry = rootEntry(rootIndex);
    return entry ? &entry->localBounds : nullptr;
}

const VoxelSurfaceCache::RootEntryMap& VoxelSurfaceCache::rootEntries() const
{
    return m_roots;
}

/// 完整数据访问

const VoxelFaceColorMap& VoxelSurfaceCache::faceColors() const
{
    rebuildCombinedFaceColors();
    return m_combinedFaceColors;
}

VoxelFaceSet VoxelSurfaceCache::combinedFaces() const
{
    VoxelFaceSet::Container faces;
    faces.reserve(faceCount());

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        iterator->second.faceMasks.appendFaces(faces);
    }

    return VoxelFaceSet(std::move(faces));
}

Geometry::Mesh VoxelSurfaceCache::combinedMesh() const
{
    Geometry::Mesh mesh;
    mesh.reserve(
        vertexCount(),
        triangleCount() * 3);

    for (RootEntryMap::const_iterator iterator = m_roots.begin();
         iterator != m_roots.end();
         ++iterator)
    {
        for (unsigned int directionValue = 0;
             directionValue < VoxelFaceDirectionCount;
             ++directionValue)
        {
            const Geometry::Mesh& directionMesh =
                iterator->second.directionMeshes[directionValue];

            if (!directionMesh.isEmpty())
            {
                mesh.append(directionMesh);
            }
        }
    }

    return mesh;
}

/// 内部辅助

bool VoxelSurfaceCache::isCompatible(const VoxelShape& shape) const
{
    return m_initialized && m_grid.isEqualTo(shape.grid(), 0.0);
}

void VoxelSurfaceCache::rebuildRootMeshes(const VoxelShape& shape, const RootIndexSet& rootIndices)
{
    if (rootIndices.empty())
    {
        return;
    }

    std::vector<RootMeshBuildItem> buildItems;
    buildItems.reserve(rootIndices.size());

    RootIndexSet emptyRootIndices;

    // 准备阶段只读取根缓存，避免并行执行期间修改std::map结构。
    for (RootIndexSet::const_iterator rootIterator = rootIndices.begin();
         rootIterator != rootIndices.end();
         ++rootIterator)
    {
        const RootEntryMap::const_iterator entryIterator = m_roots.find(*rootIterator);

        if (entryIterator == m_roots.end())
        {
            continue;
        }

        if (entryIterator->second.faceMasks.isEmpty())
        {
            emptyRootIndices.insert(*rootIterator);
            continue;
        }

        buildItems.push_back(
            RootMeshBuildItem(
                *rootIterator,
                &entryIterator->second.faceMasks,
                &entryIterator->second.colors,
                entryIterator->second.directionQuadCountHints));
    }

    buildRootMeshes(shape, buildItems, m_defaultColor, nullptr);

    // 只有全部根成功构建后才统一修改根缓存。
    for (RootIndexSet::const_iterator rootIterator = emptyRootIndices.begin();
         rootIterator != emptyRootIndices.end();
         ++rootIterator)
    {
        m_roots.erase(*rootIterator);
    }

    commitRootMeshBuildItems(m_roots, buildItems);
    rebuildLocalBounds();
}

void VoxelSurfaceCache::invalidateCombinedFaceColors()
{
    m_combinedFaceColorsDirty = true;
}

void VoxelSurfaceCache::rebuildCombinedFaceColors() const
{
    if (!m_combinedFaceColorsDirty)
    {
        return;
    }

    std::size_t colorCount = 0;

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        MYVOXEL_ASSERT_MESSAGE(
            iterator->second.colors.size() <= (std::numeric_limits<std::size_t>::max)() - colorCount,
            "Combined voxel face color count overflowed.");

        colorCount += iterator->second.colors.size();
    }

    m_combinedFaceColors.clear();
    m_combinedFaceColors.reserve(colorCount);

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        m_combinedFaceColors.appendSorted(iterator->second.colors);
    }

    m_combinedFaceColorsDirty = false;
}

void VoxelSurfaceCache::rebuildLocalBounds()
{
    Bounds3 bounds;

    for (RootEntryMap::const_iterator iterator = m_roots.begin(); iterator != m_roots.end(); ++iterator)
    {
        bounds = combinedBounds(bounds, iterator->second.localBounds);
    }

    m_localBounds = bounds;
}

}