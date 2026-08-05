#include "VoxelChangeSet.h"

#include <limits>
#include <utility>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const unsigned int BitsPerWord = 64; // 一个std::uint64_t固定保存64个体素变化标记。

// 返回非零64位掩码中最低置位的位索引。
unsigned int firstSetBitIndex(std::uint64_t mask)
{
    MYVOXEL_ASSERT_MESSAGE(mask != 0, "Voxel dirty cell bit scan requires a non-zero mask.");

#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long index = 0;
    _BitScanForward64(&index, mask);
    return static_cast<unsigned int>(index);
#elif defined(_MSC_VER)
    unsigned long index = 0;
    const unsigned long lowWord =
        static_cast<unsigned long>(mask & 0xFFFFFFFFULL);

    if (lowWord != 0)
    {
        _BitScanForward(&index, lowWord);
        return static_cast<unsigned int>(index);
    }

    _BitScanForward(
        &index,
        static_cast<unsigned long>(mask >> 32U));

    return static_cast<unsigned int>(index + 32UL);
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned int>(__builtin_ctzll(mask));
#else
    unsigned int index = 0;

    while ((mask & static_cast<std::uint64_t>(1ULL)) == 0)
    {
        mask >>= 1U;
        ++index;
    }

    return index;
#endif
}

// 返回一个第0层根在指定层级单轴包含的逻辑体素数量。
std::size_t rootAxisCellCount(MyVoxel::VoxelLevel level)
{
    const unsigned int bitCount =
        static_cast<unsigned int>(
            (std::numeric_limits<std::size_t>::digits));

    MYVOXEL_ASSERT_MESSAGE(
        static_cast<unsigned int>(level) < bitCount,
        "Voxel change tracking level exceeds size_t shift range.");

    return static_cast<std::size_t>(1) <<
           static_cast<unsigned int>(level);
}

// 返回保存指定数量连续位所需的64位字数量。
std::size_t wordCount(std::size_t bitCount)
{
    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        bitCount <= maximumSize -
                        static_cast<std::size_t>(BitsPerWord - 1U),
        "Voxel dirty cell word count overflowed.");

    return (bitCount +
            static_cast<std::size_t>(BitsPerWord - 1U)) /
           static_cast<std::size_t>(BitsPerWord);
}

// 返回根内指定层级索引对应的非负局部坐标。
std::size_t localCoordinate(MyVoxel::VoxelIndex globalIndex,
                            MyVoxel::VoxelIndex rootIndex,
                            std::size_t axisCount)
{
    const std::int64_t value =
        static_cast<std::int64_t>(globalIndex) -
        static_cast<std::int64_t>(rootIndex) *
            static_cast<std::int64_t>(axisCount);

    MYVOXEL_ASSERT_MESSAGE(
        value >= 0 &&
            value < static_cast<std::int64_t>(axisCount),
        "Voxel dirty cell address does not belong to the target root.");

    return static_cast<std::size_t>(value);
}

// 返回指定根局部坐标在目标层级中的全局索引。
MyVoxel::VoxelIndex globalCoordinate(MyVoxel::VoxelIndex rootIndex,
                                     std::size_t axisCount,
                                     std::size_t localIndex)
{
    const std::int64_t value =
        static_cast<std::int64_t>(rootIndex) *
            static_cast<std::int64_t>(axisCount) +
        static_cast<std::int64_t>(localIndex);

    const std::int64_t minimum =
        static_cast<std::int64_t>(
            (std::numeric_limits<MyVoxel::VoxelIndex>::min)());

    const std::int64_t maximum =
        static_cast<std::int64_t>(
            (std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(
        value >= minimum && value <= maximum,
        "Voxel dirty cell global index exceeds VoxelIndex range.");

    return static_cast<MyVoxel::VoxelIndex>(value);
}

}

namespace MyVoxel
{

VoxelChangeSet::DirtyCellRegion::DirtyCellRegion(
    const VoxelCellIndex& rootIndexValue,
    VoxelLevel trackingLevelValue)
    : m_rootIndex(rootIndexValue)
    , m_trackingLevel(trackingLevelValue)
    , m_axisCellCount(rootAxisCellCount(trackingLevelValue))
    , m_totalCellCount(0)
    , m_changedCellCount(0)
    , m_fullRoot(false)
{
    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        m_axisCellCount <= maximumSize / m_axisCellCount,
        "Voxel dirty cell plane count overflowed.");

    const std::size_t planeCellCount =
        m_axisCellCount * m_axisCellCount;

    MYVOXEL_ASSERT_MESSAGE(
        planeCellCount <= maximumSize / m_axisCellCount,
        "Voxel dirty cell volume count overflowed.");

    m_totalCellCount = planeCellCount * m_axisCellCount;
}

/// 状态判断

bool VoxelChangeSet::DirtyCellRegion::isFullRoot() const
{
    return m_fullRoot;
}

bool VoxelChangeSet::DirtyCellRegion::containsLocalCell(
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ) const
{
    const std::size_t bitIndex =
        cellBitIndex(localX, localY, localZ);

    if (m_fullRoot)
    {
        return true;
    }

    if (m_changedWords.empty())
    {
        return false;
    }

    const std::size_t wordIndex = bitIndex >> 6U;
    const unsigned int wordBit =
        static_cast<unsigned int>(bitIndex & 63U);

    return (m_changedWords[wordIndex] &
            (static_cast<std::uint64_t>(1ULL) << wordBit)) != 0;
}

/// 区域属性

const VoxelCellIndex& VoxelChangeSet::DirtyCellRegion::rootIndex() const
{
    return m_rootIndex;
}

VoxelLevel VoxelChangeSet::DirtyCellRegion::trackingLevel() const
{
    return m_trackingLevel;
}

std::size_t VoxelChangeSet::DirtyCellRegion::axisCellCount() const
{
    return m_axisCellCount;
}

std::size_t VoxelChangeSet::DirtyCellRegion::changedCellCount() const
{
    return m_changedCellCount;
}

/// 区域访问

void VoxelChangeSet::DirtyCellRegion::appendChangedLocalCellIndices(
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
        std::uint64_t word = m_changedWords[wordIndex];

        while (word != 0)
        {
            const unsigned int wordBit =
                firstSetBitIndex(word);

            const std::size_t bitIndex =
                wordIndex *
                    static_cast<std::size_t>(BitsPerWord) +
                wordBit;

            cellIndices.push_back(bitIndex);
            word &= word - static_cast<std::uint64_t>(1ULL);
        }
    }
}

void VoxelChangeSet::DirtyCellRegion::appendChangedCellIndices(
    std::vector<VoxelCellIndex>& cellIndices) const
{
    cellIndices.reserve(
        cellIndices.size() + m_changedCellCount);

    std::vector<std::size_t> localCellIndices;
    localCellIndices.reserve(m_changedCellCount);

    appendChangedLocalCellIndices(localCellIndices);

    for (std::size_t cellPosition = 0;
         cellPosition < localCellIndices.size();
         ++cellPosition)
    {
        const std::size_t bitIndex =
            localCellIndices[cellPosition];

        const std::size_t localX =
            bitIndex % m_axisCellCount;

        const std::size_t planeIndex =
            bitIndex / m_axisCellCount;

        const std::size_t localY =
            planeIndex % m_axisCellCount;

        const std::size_t localZ =
            planeIndex / m_axisCellCount;

        cellIndices.push_back(
            VoxelCellIndex(
                globalCoordinate(
                    m_rootIndex.x,
                    m_axisCellCount,
                    localX),
                globalCoordinate(
                    m_rootIndex.y,
                    m_axisCellCount,
                    localY),
                globalCoordinate(
                    m_rootIndex.z,
                    m_axisCellCount,
                    localZ)));
    }
}

/// 内部修改

void VoxelChangeSet::DirtyCellRegion::recordMaterialChange(
    const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(
        rootCellAddress(address).index == m_rootIndex,
        "Voxel dirty cell address belongs to another root.");

    if (m_fullRoot)
    {
        return;
    }

    if (address.level <= m_trackingLevel)
    {
        const std::size_t addressAxisCount =
            rootAxisCellCount(address.level);

        const std::size_t descendantScale =
            rootAxisCellCount(
                static_cast<VoxelLevel>(
                    static_cast<unsigned int>(m_trackingLevel) -
                    static_cast<unsigned int>(address.level)));

        const std::size_t localX =
            localCoordinate(
                address.index.x,
                m_rootIndex.x,
                addressAxisCount);

        const std::size_t localY =
            localCoordinate(
                address.index.y,
                m_rootIndex.y,
                addressAxisCount);

        const std::size_t localZ =
            localCoordinate(
                address.index.z,
                m_rootIndex.z,
                addressAxisCount);

        const std::size_t minimumX =
            localX * descendantScale;

        const std::size_t minimumY =
            localY * descendantScale;

        const std::size_t minimumZ =
            localZ * descendantScale;

        markLocalRange(
            minimumX,
            minimumX + descendantScale - 1,
            minimumY,
            minimumY + descendantScale - 1,
            minimumZ,
            minimumZ + descendantScale - 1);

        return;
    }

    const VoxelCellAddress trackedAddress =
        ancestorCellAddress(address, m_trackingLevel);

    const std::size_t localX =
        localCoordinate(
            trackedAddress.index.x,
            m_rootIndex.x,
            m_axisCellCount);

    const std::size_t localY =
        localCoordinate(
            trackedAddress.index.y,
            m_rootIndex.y,
            m_axisCellCount);

    const std::size_t localZ =
        localCoordinate(
            trackedAddress.index.z,
            m_rootIndex.z,
            m_axisCellCount);

    addLocalCell(localX, localY, localZ);
}

void VoxelChangeSet::DirtyCellRegion::markFullRoot()
{
    m_changedWords.clear();
    m_changedCellCount = m_totalCellCount;
    m_fullRoot = true;
}

void VoxelChangeSet::DirtyCellRegion::accumulate(
    const DirtyCellRegion& other)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_rootIndex == other.m_rootIndex &&
            m_trackingLevel == other.m_trackingLevel &&
            m_axisCellCount == other.m_axisCellCount &&
            m_totalCellCount == other.m_totalCellCount,
        "Voxel dirty cell accumulation requires the same root and tracking level.");

    if (m_fullRoot || other.m_changedCellCount == 0)
    {
        return;
    }

    if (other.m_fullRoot)
    {
        markFullRoot();
        return;
    }

    ensureWords();

    for (std::size_t wordIndex = 0;
         wordIndex < other.m_changedWords.size();
         ++wordIndex)
    {
        const std::uint64_t oldWord =
            m_changedWords[wordIndex];

        const std::uint64_t newWord =
            oldWord | other.m_changedWords[wordIndex];

        std::uint64_t addedBits =
            newWord & ~oldWord;

        while (addedBits != 0)
        {
            addedBits &=
                addedBits - static_cast<std::uint64_t>(1ULL);

            ++m_changedCellCount;
        }

        m_changedWords[wordIndex] = newWord;
    }
}

void VoxelChangeSet::DirtyCellRegion::addLocalCell(
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ)
{
    if (m_fullRoot)
    {
        return;
    }

    const std::size_t bitIndex =
        cellBitIndex(localX, localY, localZ);

    ensureWords();

    const std::size_t wordIndex = bitIndex >> 6U;
    const unsigned int wordBit =static_cast<unsigned int>(bitIndex & 63U);

    const std::uint64_t bit =static_cast<std::uint64_t>(1ULL) << wordBit;

    if ((m_changedWords[wordIndex] & bit) == 0)
    {
        m_changedWords[wordIndex] |= bit;
        ++m_changedCellCount;
    }
}

std::size_t VoxelChangeSet::DirtyCellRegion::cellBitIndex(
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ) const
{
    MYVOXEL_ASSERT_MESSAGE(
        localX < m_axisCellCount &&
            localY < m_axisCellCount &&
            localZ < m_axisCellCount,
        "Voxel dirty cell local coordinate exceeds the root range.");

    return (localZ * m_axisCellCount + localY) *
               m_axisCellCount +
           localX;
}

void VoxelChangeSet::DirtyCellRegion::markLocalRange(
    std::size_t minimumX,
    std::size_t maximumX,
    std::size_t minimumY,
    std::size_t maximumY,
    std::size_t minimumZ,
    std::size_t maximumZ)
{
    MYVOXEL_ASSERT_MESSAGE(
        minimumX <= maximumX &&
            maximumX < m_axisCellCount &&
            minimumY <= maximumY &&
            maximumY < m_axisCellCount &&
            minimumZ <= maximumZ &&
            maximumZ < m_axisCellCount,
        "Voxel dirty cell range exceeds the root range.");

    if (minimumX == 0 &&
        maximumX + 1 == m_axisCellCount &&
        minimumY == 0 &&
        maximumY + 1 == m_axisCellCount &&
        minimumZ == 0 &&
        maximumZ + 1 == m_axisCellCount)
    {
        markFullRoot();
        return;
    }

    for (std::size_t localZ = minimumZ;
         localZ <= maximumZ;
         ++localZ)
    {
        for (std::size_t localY = minimumY;
             localY <= maximumY;
             ++localY)
        {
            for (std::size_t localX = minimumX;
                 localX <= maximumX;
                 ++localX)
            {
                addLocalCell(localX, localY, localZ);
            }
        }
    }
}

void VoxelChangeSet::DirtyCellRegion::ensureWords()
{
    if (!m_changedWords.empty())
    {
        return;
    }

    m_changedWords.assign(
        wordCount(m_totalCellCount),
        static_cast<std::uint64_t>(0));
}

/// VoxelChangeSet构造

VoxelChangeSet::VoxelChangeSet(VoxelLevel trackingLevelValue)
    : m_trackingLevel(trackingLevelValue)
{
    rootAxisCellCount(trackingLevelValue);
}

VoxelChangeSet::VoxelChangeSet(VoxelChangeSet&& other)
    : m_trackingLevel(other.m_trackingLevel)
    , m_modifiedRootIndices(std::move(other.m_modifiedRootIndices))
    , m_dirtyRegions(std::move(other.m_dirtyRegions))
{
}

VoxelChangeSet& VoxelChangeSet::operator=(VoxelChangeSet&& other)
{
    if (this != &other)
    {
        m_trackingLevel = other.m_trackingLevel;
        m_modifiedRootIndices = std::move(other.m_modifiedRootIndices);
        m_dirtyRegions = std::move(other.m_dirtyRegions);
    }

    return *this;
}

/// 状态判断

bool VoxelChangeSet::hasChanges() const
{
    return !m_modifiedRootIndices.empty();
}

bool VoxelChangeSet::containsModifiedRoot(const VoxelCellIndex& rootIndex) const
{
    return m_modifiedRootIndices.find(rootIndex) !=
           m_modifiedRootIndices.end();
}

bool VoxelChangeSet::containsDirtyRegion(const VoxelCellIndex& rootIndex) const
{
    return m_dirtyRegions.find(rootIndex) !=
           m_dirtyRegions.end();
}

/// 记录属性

VoxelLevel VoxelChangeSet::trackingLevel() const
{
    return m_trackingLevel;
}

std::size_t VoxelChangeSet::modifiedRootCount() const
{
    return m_modifiedRootIndices.size();
}

std::size_t VoxelChangeSet::dirtyRegionCount() const
{
    return m_dirtyRegions.size();
}

/// 记录访问

const VoxelChangeSet::RootIndexSet& VoxelChangeSet::modifiedRootIndices() const
{
    return m_modifiedRootIndices;
}

const VoxelChangeSet::DirtyCellRegion* VoxelChangeSet::dirtyRegion(
    const VoxelCellIndex& rootIndex) const
{
    const DirtyCellRegionMap::const_iterator iterator =
        m_dirtyRegions.find(rootIndex);

    return iterator == m_dirtyRegions.end()
               ? nullptr
               : &iterator->second;
}

const VoxelChangeSet::DirtyCellRegionMap& VoxelChangeSet::dirtyRegions() const
{
    return m_dirtyRegions;
}

/// 内部记录

void VoxelChangeSet::recordStructureChange(
    const VoxelCellAddress& address)
{
    m_modifiedRootIndices.insert(
        rootCellAddress(address).index);
}

void VoxelChangeSet::recordMaterialChange(const VoxelCellAddress& address)
{
    const VoxelCellIndex rootIndex =rootCellAddress(address).index;

    m_modifiedRootIndices.insert(rootIndex);
    ensureDirtyRegion(rootIndex).recordMaterialChange(address);
}

void VoxelChangeSet::recordFullRootChange(const VoxelCellIndex& rootIndex)
{
    m_modifiedRootIndices.insert(rootIndex);
    ensureDirtyRegion(rootIndex).markFullRoot();
}

void VoxelChangeSet::recordDirtyRegion(const DirtyCellRegion& dirtyRegion)
{
    MYVOXEL_ASSERT_MESSAGE(
        dirtyRegion.trackingLevel() == m_trackingLevel,
        "Voxel dirty region tracking level does not match the change set.");

    MYVOXEL_ASSERT_MESSAGE(
        dirtyRegion.changedCellCount() > 0,
        "Cannot record an empty voxel dirty region.");

    const VoxelCellIndex& rootIndex =
        dirtyRegion.rootIndex();

    m_modifiedRootIndices.insert(rootIndex);

    DirtyCellRegionMap::iterator iterator =
        m_dirtyRegions.find(rootIndex);

    if (iterator == m_dirtyRegions.end())
    {
        m_dirtyRegions.insert(
            std::make_pair(
                rootIndex,
                dirtyRegion));

        return;
    }

    iterator->second.accumulate(dirtyRegion);
}

void VoxelChangeSet::accumulate(const VoxelChangeSet& other)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_trackingLevel == other.m_trackingLevel,
        "VoxelChangeSet accumulation requires the same tracking level.");

    m_modifiedRootIndices.insert(
        other.m_modifiedRootIndices.begin(),
        other.m_modifiedRootIndices.end());

    for (DirtyCellRegionMap::const_iterator iterator =
             other.m_dirtyRegions.begin();
         iterator != other.m_dirtyRegions.end();
         ++iterator)
    {
        DirtyCellRegionMap::iterator current =
            m_dirtyRegions.find(iterator->first);

        if (current == m_dirtyRegions.end())
        {
            m_dirtyRegions.insert(
                std::make_pair(
                    iterator->first,
                    iterator->second));

            continue;
        }

        current->second.accumulate(iterator->second);
    }
}

void VoxelChangeSet::clear()
{
    m_modifiedRootIndices.clear();
    m_dirtyRegions.clear();
}

void VoxelChangeSet::swap(VoxelChangeSet& other)
{
    MYVOXEL_ASSERT_MESSAGE(
        m_trackingLevel == other.m_trackingLevel,
        "VoxelChangeSet swap requires the same tracking level.");

    m_modifiedRootIndices.swap(other.m_modifiedRootIndices);
    m_dirtyRegions.swap(other.m_dirtyRegions);
}

VoxelChangeSet::DirtyCellRegion& VoxelChangeSet::ensureDirtyRegion(
    const VoxelCellIndex& rootIndex)
{
    DirtyCellRegionMap::iterator iterator =
        m_dirtyRegions.find(rootIndex);

    if (iterator != m_dirtyRegions.end())
    {
        return iterator->second;
    }

    iterator = m_dirtyRegions.insert(
        std::make_pair(
            rootIndex,
            DirtyCellRegion(
                rootIndex,
                m_trackingLevel))).first;

    return iterator->second;
}

}