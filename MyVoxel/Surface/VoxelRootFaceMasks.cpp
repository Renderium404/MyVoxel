#include "VoxelRootFaceMasks.h"

#include <limits>
#include <utility>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const unsigned int BitsPerWord = 64; // 一个std::uint64_t固定保存64个连续平面U方向单位面。

// 返回保存指定数量连续位所需的64位字数量。
std::size_t wordCount(std::size_t bitCount)
{
    const std::size_t maximumSize = (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(bitCount > 0, "Voxel root face mask axis size must be positive.");
    MYVOXEL_ASSERT_MESSAGE(
        bitCount <= maximumSize - static_cast<std::size_t>(BitsPerWord - 1U),
        "Voxel root face mask word count overflowed.");

    return (bitCount + static_cast<std::size_t>(BitsPerWord - 1U)) /
           static_cast<std::size_t>(BitsPerWord);
}

// 返回非零64位掩码中最低置位的位索引。
unsigned int firstSetBitIndex(std::uint64_t mask)
{
    MYVOXEL_ASSERT_MESSAGE(mask != 0, "Cannot query the first set bit of an empty face mask.");

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

    MYVOXEL_ASSERT_MESSAGE(found != 0, "Cannot find a set bit in a non-empty face mask.");
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

// 返回最低置位索引并从原掩码中删除该位。
unsigned int takeFirstSetBitIndex(std::uint64_t& mask)
{
    const unsigned int bitIndex = firstSetBitIndex(mask);
    mask &= mask - static_cast<std::uint64_t>(1ULL);
    return bitIndex;
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

// 将64位整数转换为VoxelIndex。
MyVoxel::VoxelIndex voxelIndex(std::int64_t value)
{
    const std::int64_t minimum =
        static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum =
        static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(
        value >= minimum && value <= maximum,
        "Voxel root face address exceeds VoxelIndex range.");

    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回第0层根索引在最高层网格中的最小体素坐标。
std::int64_t rootMinimumCoordinate(MyVoxel::VoxelIndex rootIndex, std::size_t axisCellCount)
{
    MYVOXEL_ASSERT_MESSAGE(
        axisCellCount <= static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)()),
        "Voxel root face mask resolution exceeds int64 range.");

    const std::int64_t scale = static_cast<std::int64_t>(axisCellCount);
    const std::int64_t value = static_cast<std::int64_t>(rootIndex);
    const std::int64_t minimum = (std::numeric_limits<std::int64_t>::min)();
    const std::int64_t maximum = (std::numeric_limits<std::int64_t>::max)();

    if (value > 0)
    {
        MYVOXEL_ASSERT_MESSAGE(
            value <= maximum / scale,
            "Voxel root face minimum coordinate multiplication overflowed.");
    }
    else if (value < 0)
    {
        MYVOXEL_ASSERT_MESSAGE(
            value >= minimum / scale,
            "Voxel root face minimum coordinate multiplication underflowed.");
    }

    return value * scale;
}

}

namespace MyVoxel
{

VoxelRootFaceMasks::VoxelRootFaceMasks()
    : m_axisCellCount(0)
    , m_wordsPerPlaneRow(0)
    , m_faceCount(0)
{
}

VoxelRootFaceMasks::VoxelRootFaceMasks(
    const VoxelCellIndex& rootIndexValue,
    std::size_t axisCellCountValue)
    : m_rootIndex(rootIndexValue)
    , m_axisCellCount(axisCellCountValue)
    , m_wordsPerPlaneRow(wordCount(axisCellCountValue))
    , m_faceCount(0)
{
    // 提前验证完整位图尺寸，但空Root不立即分配存储。
    storageWordCount();
}

VoxelRootFaceMasks::VoxelRootFaceMasks(VoxelRootFaceMasks&& other)
    : m_rootIndex(other.m_rootIndex)
    , m_axisCellCount(other.m_axisCellCount)
    , m_wordsPerPlaneRow(other.m_wordsPerPlaneRow)
    , m_faceCount(other.m_faceCount)
    , m_words(std::move(other.m_words))
{
    other.m_axisCellCount = 0;
    other.m_wordsPerPlaneRow = 0;
    other.m_faceCount = 0;
}

VoxelRootFaceMasks& VoxelRootFaceMasks::operator=(VoxelRootFaceMasks&& other)
{
    if (this != &other)
    {
        m_rootIndex = other.m_rootIndex;
        m_axisCellCount = other.m_axisCellCount;
        m_wordsPerPlaneRow = other.m_wordsPerPlaneRow;
        m_faceCount = other.m_faceCount;
        m_words = std::move(other.m_words);

        other.m_axisCellCount = 0;
        other.m_wordsPerPlaneRow = 0;
        other.m_faceCount = 0;
    }

    return *this;
}

/// 掩码状态

bool VoxelRootFaceMasks::isInitialized() const
{
    return m_axisCellCount > 0;
}

bool VoxelRootFaceMasks::isEmpty() const
{
    return m_faceCount == 0;
}

const VoxelCellIndex& VoxelRootFaceMasks::rootIndex() const
{
    MYVOXEL_ASSERT_MESSAGE(isInitialized(), "Voxel root face masks are not initialized.");
    return m_rootIndex;
}

std::size_t VoxelRootFaceMasks::axisCellCount() const
{
    MYVOXEL_ASSERT_MESSAGE(isInitialized(), "Voxel root face masks are not initialized.");
    return m_axisCellCount;
}

std::size_t VoxelRootFaceMasks::wordsPerPlaneRow() const
{
    MYVOXEL_ASSERT_MESSAGE(isInitialized(), "Voxel root face masks are not initialized.");
    return m_wordsPerPlaneRow;
}

std::size_t VoxelRootFaceMasks::faceCount() const
{
    return m_faceCount;
}

/// 掩码读取

std::uint64_t VoxelRootFaceMasks::planeWord(
    VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t v,
    std::size_t wordIndex) const
{
    const std::size_t index =
        planeWordIndex(direction, slice, v, wordIndex);

    return m_words.empty()
        ? static_cast<std::uint64_t>(0)
        : m_words[index];
}

bool VoxelRootFaceMasks::contains(const VoxelFaceAddress& face) const
{
    if (!isInitialized() || !isValidVoxelFaceDirection(face.direction))
    {
        return false;
    }

    const std::int64_t rootMinimumX =
        rootMinimumCoordinate(m_rootIndex.x, m_axisCellCount);
    const std::int64_t rootMinimumY =
        rootMinimumCoordinate(m_rootIndex.y, m_axisCellCount);
    const std::int64_t rootMinimumZ =
        rootMinimumCoordinate(m_rootIndex.z, m_axisCellCount);
    const std::int64_t localX =
        static_cast<std::int64_t>(face.cellIndex.x) - rootMinimumX;
    const std::int64_t localY =
        static_cast<std::int64_t>(face.cellIndex.y) - rootMinimumY;
    const std::int64_t localZ =
        static_cast<std::int64_t>(face.cellIndex.z) - rootMinimumZ;

    if (localX < 0 || localY < 0 || localZ < 0 ||
        localX >= static_cast<std::int64_t>(m_axisCellCount) ||
        localY >= static_cast<std::int64_t>(m_axisCellCount) ||
        localZ >= static_cast<std::int64_t>(m_axisCellCount))
    {
        return false;
    }

    std::size_t slice = 0;
    std::size_t u = 0;
    std::size_t v = 0;

    planeCoordinates(
        face.direction,
        static_cast<std::size_t>(localX),
        static_cast<std::size_t>(localY),
        static_cast<std::size_t>(localZ),
        slice,
        u,
        v);

    const std::size_t wordIndex = u >> 6U;
    const unsigned int bitIndex = static_cast<unsigned int>(u & 63U);

    return (planeWord(face.direction, slice, v, wordIndex) &
            (static_cast<std::uint64_t>(1ULL) << bitIndex)) != 0;
}

bool VoxelRootFaceMasks::directionIsEmpty(VoxelFaceDirection direction) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Voxel root face direction query requires initialized masks.");
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel root face direction is invalid.");

    if (m_words.empty())
    {
        return true;
    }

    const std::size_t directionWordCount =
        m_axisCellCount *
        m_axisCellCount *
        m_wordsPerPlaneRow;
    const std::size_t directionOffset =
        static_cast<std::size_t>(direction) *
        directionWordCount;

    for (std::size_t wordIndex = 0;
         wordIndex < directionWordCount;
         ++wordIndex)
    {
        if (m_words[directionOffset + wordIndex] != 0)
        {
            return false;
        }
    }

    return true;
}

bool VoxelRootFaceMasks::directionEquals(
    const VoxelRootFaceMasks& other,
    VoxelFaceDirection direction) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized() && other.isInitialized(),
        "Voxel root face direction comparison requires initialized masks.");
    MYVOXEL_ASSERT_MESSAGE(
        m_rootIndex == other.m_rootIndex &&
        m_axisCellCount == other.m_axisCellCount,
        "Voxel root face direction comparison requires the same root and resolution.");
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel root face direction is invalid.");

    if (m_words.empty() && other.m_words.empty())
    {
        return true;
    }

    const std::size_t directionWordCount =
        m_axisCellCount *
        m_axisCellCount *
        m_wordsPerPlaneRow;
    const std::size_t directionOffset =
        static_cast<std::size_t>(direction) *
        directionWordCount;

    for (std::size_t wordIndex = 0;
         wordIndex < directionWordCount;
         ++wordIndex)
    {
        const std::uint64_t firstWord =
            m_words.empty()
                ? static_cast<std::uint64_t>(0)
                : m_words[directionOffset + wordIndex];
        const std::uint64_t secondWord =
            other.m_words.empty()
                ? static_cast<std::uint64_t>(0)
                : other.m_words[directionOffset + wordIndex];

        if (firstWord != secondWord)
        {
            return false;
        }
    }

    return true;
}

/// 掩码构建

void VoxelRootFaceMasks::setPlaneWord(
    VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t v,
    std::size_t wordIndex,
    std::uint64_t word)
{
    const std::size_t index =
        planeWordIndex(direction, slice, v, wordIndex);
    const std::uint64_t validMask =
        validWordMask(wordIndex);

    MYVOXEL_ASSERT_MESSAGE(
        (word & ~validMask) == 0,
        "Voxel root face mask word contains bits outside the plane resolution.");

    if (word == 0)
    {
        return;
    }

    ensureStorage();

    MYVOXEL_ASSERT_MESSAGE(
        m_words[index] == 0,
        "Voxel root face mask word was written more than once.");

    m_words[index] = word;
    m_faceCount += bitCount(word);
}

void VoxelRootFaceMasks::setFace(
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ,
    VoxelFaceDirection direction)
{
    MYVOXEL_ASSERT_MESSAGE(isInitialized(), "Voxel root face masks are not initialized.");
    MYVOXEL_ASSERT_MESSAGE(
        localX < m_axisCellCount &&
        localY < m_axisCellCount &&
        localZ < m_axisCellCount,
        "Voxel root face local coordinate exceeds the root resolution.");

    std::size_t slice = 0;
    std::size_t u = 0;
    std::size_t v = 0;

    planeCoordinates(direction, localX, localY, localZ, slice, u, v);

    const std::size_t wordIndex = u >> 6U;
    const unsigned int bitIndex = static_cast<unsigned int>(u & 63U);
    const std::size_t index =
        planeWordIndex(direction, slice, v, wordIndex);
    const std::uint64_t bit =
        static_cast<std::uint64_t>(1ULL) << bitIndex;

    ensureStorage();

    if ((m_words[index] & bit) == 0)
    {
        m_words[index] |= bit;
        ++m_faceCount;
    }
}

void VoxelRootFaceMasks::replacePlaneWordMasked(
    VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t v,
    std::size_t wordIndex,
    std::uint64_t calculatedWord,
    std::uint64_t replaceMask)
{
    const std::size_t index =
        planeWordIndex(
            direction,
            slice,
            v,
            wordIndex);
    const std::uint64_t validMask =
        validWordMask(wordIndex);

    replaceMask &= validMask;
    calculatedWord &= validMask;

    if (replaceMask == 0)
    {
        return;
    }

    const std::uint64_t oldWord =
        m_words.empty()
            ? static_cast<std::uint64_t>(0)
            : m_words[index];
    const std::uint64_t newWord =
        (oldWord & ~replaceMask) |
        (calculatedWord & replaceMask);

    if (oldWord == newWord)
    {
        return;
    }

    const std::size_t oldCount =
        bitCount(oldWord & replaceMask);
    const std::size_t newCount =
        bitCount(newWord & replaceMask);

    if (newWord != 0)
    {
        ensureStorage();
    }

    if (newCount >= oldCount)
    {
        m_faceCount +=
            newCount - oldCount;
    }
    else
    {
        MYVOXEL_ASSERT_MESSAGE(
            m_faceCount >= oldCount - newCount,
            "Voxel root face count underflowed.");

        m_faceCount -=
            oldCount - newCount;
    }

    m_words[index] = newWord;
}

void VoxelRootFaceMasks::releaseEmptyStorage()
{
    if (m_faceCount == 0)
    {
        std::vector<std::uint64_t>().swap(m_words);
    }
}

/// 面地址转换

VoxelFaceSet VoxelRootFaceMasks::toFaceSet() const
{
    if (isEmpty())
    {
        return VoxelFaceSet();
    }

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        m_axisCellCount <= maximumSize / m_axisCellCount,
        "Voxel root face cell direction array size overflowed.");

    const std::size_t cellPlaneCount =
        m_axisCellCount * m_axisCellCount;

    MYVOXEL_ASSERT_MESSAGE(
        cellPlaneCount <= maximumSize / m_axisCellCount,
        "Voxel root face cell direction array size overflowed.");

    const std::size_t cellCount =
        cellPlaneCount * m_axisCellCount;
    std::vector<std::uint8_t> cellDirections(
        cellCount,
        static_cast<std::uint8_t>(0));

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        const VoxelFaceDirection direction =
            static_cast<VoxelFaceDirection>(directionValue);
        const std::uint8_t directionBit =
            static_cast<std::uint8_t>(1U << directionValue);

        for (std::size_t slice = 0;
             slice < m_axisCellCount;
             ++slice)
        {
            for (std::size_t v = 0;
                 v < m_axisCellCount;
                 ++v)
            {
                for (std::size_t wordIndex = 0;
                     wordIndex < m_wordsPerPlaneRow;
                     ++wordIndex)
                {
                    std::uint64_t word =
                        planeWord(
                            direction,
                            slice,
                            v,
                            wordIndex);

                    while (word != 0)
                    {
                        const unsigned int bitIndex =
                            takeFirstSetBitIndex(word);
                        const std::size_t u =
                            wordIndex *
                                static_cast<std::size_t>(BitsPerWord) +
                            bitIndex;

                        MYVOXEL_ASSERT_MESSAGE(
                            u < m_axisCellCount,
                            "Voxel root face mask contains an invalid U coordinate.");

                        std::size_t localX = 0;
                        std::size_t localY = 0;
                        std::size_t localZ = 0;

                        switch (direction)
                        {
                        case VoxelFaceDirection::NegativeX:
                        case VoxelFaceDirection::PositiveX:
                            localX = slice;
                            localY = u;
                            localZ = v;
                            break;

                        case VoxelFaceDirection::NegativeY:
                        case VoxelFaceDirection::PositiveY:
                            localX = u;
                            localY = slice;
                            localZ = v;
                            break;

                        case VoxelFaceDirection::NegativeZ:
                        case VoxelFaceDirection::PositiveZ:
                            localX = u;
                            localY = v;
                            localZ = slice;
                            break;
                        }

                        const std::size_t cellIndex =
                            (localX * m_axisCellCount + localY) *
                                m_axisCellCount +
                            localZ;

                        cellDirections[cellIndex] |= directionBit;
                    }
                }
            }
        }
    }

    const std::int64_t rootMinimumX =
        rootMinimumCoordinate(m_rootIndex.x, m_axisCellCount);
    const std::int64_t rootMinimumY =
        rootMinimumCoordinate(m_rootIndex.y, m_axisCellCount);
    const std::int64_t rootMinimumZ =
        rootMinimumCoordinate(m_rootIndex.z, m_axisCellCount);
    VoxelFaceSet::Container faces;
    faces.reserve(m_faceCount);

    for (std::size_t localX = 0;
         localX < m_axisCellCount;
         ++localX)
    {
        const VoxelIndex cellX =
            voxelIndex(
                rootMinimumX +
                static_cast<std::int64_t>(localX));

        for (std::size_t localY = 0;
             localY < m_axisCellCount;
             ++localY)
        {
            const VoxelIndex cellY =
                voxelIndex(
                    rootMinimumY +
                    static_cast<std::int64_t>(localY));

            for (std::size_t localZ = 0;
                 localZ < m_axisCellCount;
                 ++localZ)
            {
                const std::size_t cellIndex =
                    (localX * m_axisCellCount + localY) *
                        m_axisCellCount +
                    localZ;
                const std::uint8_t directions =
                    cellDirections[cellIndex];

                if (directions == 0)
                {
                    continue;
                }

                const VoxelCellIndex ownerIndex(
                    cellX,
                    cellY,
                    voxelIndex(
                        rootMinimumZ +
                        static_cast<std::int64_t>(localZ)));

                for (unsigned int directionValue = 0;
                     directionValue < VoxelFaceDirectionCount;
                     ++directionValue)
                {
                    if ((directions &
                         static_cast<std::uint8_t>(
                             1U << directionValue)) == 0)
                    {
                        continue;
                    }

                    faces.push_back(
                        VoxelFaceAddress(
                            ownerIndex,
                            static_cast<VoxelFaceDirection>(
                                directionValue)));
                }
            }
        }
    }

    MYVOXEL_ASSERT_MESSAGE(
        faces.size() == m_faceCount,
        "Voxel root face set size does not match the mask face count.");

    return VoxelFaceSet::fromSortedUnique(std::move(faces));
}

void VoxelRootFaceMasks::appendFaces(VoxelFaceSet::Container& faces) const
{
    VoxelRootFaceMasks emptyMasks(m_rootIndex, m_axisCellCount);
    appendDifferenceFaces(emptyMasks, faces);
}

void VoxelRootFaceMasks::appendDifferenceFaces(
    const VoxelRootFaceMasks& other,
    VoxelFaceSet::Container& faces) const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized() && other.isInitialized(),
        "Voxel root face mask difference requires initialized masks.");
    MYVOXEL_ASSERT_MESSAGE(
        m_rootIndex == other.m_rootIndex &&
        m_axisCellCount == other.m_axisCellCount,
        "Voxel root face mask difference requires the same root and resolution.");

    for (unsigned int directionValue = 0;
         directionValue < VoxelFaceDirectionCount;
         ++directionValue)
    {
        const VoxelFaceDirection direction =
            static_cast<VoxelFaceDirection>(directionValue);

        for (std::size_t slice = 0; slice < m_axisCellCount; ++slice)
        {
            for (std::size_t v = 0; v < m_axisCellCount; ++v)
            {
                for (std::size_t wordIndex = 0;
                     wordIndex < m_wordsPerPlaneRow;
                     ++wordIndex)
                {
                    std::uint64_t difference =
                        planeWord(direction, slice, v, wordIndex) &
                        ~other.planeWord(direction, slice, v, wordIndex);

                    while (difference != 0)
                    {
                        const unsigned int bitIndex =
                            takeFirstSetBitIndex(difference);
                        const std::size_t u =
                            wordIndex * static_cast<std::size_t>(BitsPerWord) +
                            bitIndex;

                        MYVOXEL_ASSERT_MESSAGE(
                            u < m_axisCellCount,
                            "Voxel root face mask emitted an invalid plane coordinate.");

                        faces.push_back(faceAddress(direction, slice, u, v));
                    }
                }
            }
        }
    }
}

bool VoxelRootFaceMasks::operator==(const VoxelRootFaceMasks& other) const
{
    if (m_rootIndex != other.m_rootIndex ||
        m_axisCellCount != other.m_axisCellCount ||
        m_faceCount != other.m_faceCount)
    {
        return false;
    }

    if (m_faceCount == 0)
    {
        return true;
    }

    return m_words == other.m_words;
}

bool VoxelRootFaceMasks::operator!=(const VoxelRootFaceMasks& other) const
{
    return !(*this == other);
}

std::size_t VoxelRootFaceMasks::planeWordIndex(
    VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t v,
    std::size_t wordIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(isInitialized(), "Voxel root face masks are not initialized.");
    MYVOXEL_ASSERT_MESSAGE(
        isValidVoxelFaceDirection(direction),
        "Voxel root face direction is invalid.");
    MYVOXEL_ASSERT_MESSAGE(
        slice < m_axisCellCount,
        "Voxel root face slice exceeds the root resolution.");
    MYVOXEL_ASSERT_MESSAGE(
        v < m_axisCellCount,
        "Voxel root face V coordinate exceeds the root resolution.");
    MYVOXEL_ASSERT_MESSAGE(
        wordIndex < m_wordsPerPlaneRow,
        "Voxel root face word index exceeds one plane row.");

    return (((static_cast<std::size_t>(direction) * m_axisCellCount + slice) *
             m_axisCellCount + v) *
            m_wordsPerPlaneRow) +
           wordIndex;
}

std::uint64_t VoxelRootFaceMasks::validWordMask(std::size_t wordIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(
        wordIndex < m_wordsPerPlaneRow,
        "Voxel root face word index exceeds one plane row.");

    if (wordIndex + 1 < m_wordsPerPlaneRow)
    {
        return ~static_cast<std::uint64_t>(0);
    }

    const unsigned int usedBitCount =
        static_cast<unsigned int>(
            m_axisCellCount -
            wordIndex * static_cast<std::size_t>(BitsPerWord));

    MYVOXEL_ASSERT_MESSAGE(
        usedBitCount > 0 && usedBitCount <= BitsPerWord,
        "Voxel root face final word bit count is invalid.");

    if (usedBitCount == BitsPerWord)
    {
        return ~static_cast<std::uint64_t>(0);
    }

    return (static_cast<std::uint64_t>(1ULL) << usedBitCount) -
           static_cast<std::uint64_t>(1ULL);
}

std::size_t VoxelRootFaceMasks::storageWordCount() const
{
    MYVOXEL_ASSERT_MESSAGE(
        isInitialized(),
        "Voxel root face masks are not initialized.");

    const std::size_t maximumSize =
        (std::numeric_limits<std::size_t>::max)();

    MYVOXEL_ASSERT_MESSAGE(
        m_axisCellCount <= maximumSize / m_axisCellCount,
        "Voxel root face mask plane row count overflowed.");

    const std::size_t planeRowCount =
        m_axisCellCount * m_axisCellCount;

    MYVOXEL_ASSERT_MESSAGE(
        planeRowCount <= maximumSize / m_wordsPerPlaneRow,
        "Voxel root face mask direction word count overflowed.");

    const std::size_t directionWordCount =
        planeRowCount * m_wordsPerPlaneRow;

    MYVOXEL_ASSERT_MESSAGE(
        directionWordCount <=
            maximumSize / VoxelFaceDirectionCount,
        "Voxel root face mask total word count overflowed.");

    return directionWordCount * VoxelFaceDirectionCount;
}

void VoxelRootFaceMasks::ensureStorage()
{
    if (!m_words.empty())
    {
        return;
    }

    m_words.assign(
        storageWordCount(),
        static_cast<std::uint64_t>(0));
}

void VoxelRootFaceMasks::planeCoordinates(
    VoxelFaceDirection direction,
    std::size_t localX,
    std::size_t localY,
    std::size_t localZ,
    std::size_t& slice,
    std::size_t& u,
    std::size_t& v)
{
    switch (direction)
    {
    case VoxelFaceDirection::NegativeX:
    case VoxelFaceDirection::PositiveX:
        slice = localX;
        u = localY;
        v = localZ;
        return;

    case VoxelFaceDirection::NegativeY:
    case VoxelFaceDirection::PositiveY:
        slice = localY;
        u = localX;
        v = localZ;
        return;

    case VoxelFaceDirection::NegativeZ:
    case VoxelFaceDirection::PositiveZ:
        slice = localZ;
        u = localX;
        v = localY;
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxel root face direction is unknown.");
    slice = 0;
    u = 0;
    v = 0;
}

VoxelFaceAddress VoxelRootFaceMasks::faceAddress(
    VoxelFaceDirection direction,
    std::size_t slice,
    std::size_t u,
    std::size_t v) const
{
    MYVOXEL_ASSERT_MESSAGE(
        slice < m_axisCellCount &&
        u < m_axisCellCount &&
        v < m_axisCellCount,
        "Voxel root face plane coordinate exceeds the root resolution.");

    std::size_t localX = 0;
    std::size_t localY = 0;
    std::size_t localZ = 0;

    switch (direction)
    {
    case VoxelFaceDirection::NegativeX:
    case VoxelFaceDirection::PositiveX:
        localX = slice;
        localY = u;
        localZ = v;
        break;

    case VoxelFaceDirection::NegativeY:
    case VoxelFaceDirection::PositiveY:
        localX = u;
        localY = slice;
        localZ = v;
        break;

    case VoxelFaceDirection::NegativeZ:
    case VoxelFaceDirection::PositiveZ:
        localX = u;
        localY = v;
        localZ = slice;
        break;

    default:
        MYVOXEL_ASSERT_MESSAGE(false, "Voxel root face direction is unknown.");
        break;
    }

    const std::int64_t rootMinimumX =
        rootMinimumCoordinate(m_rootIndex.x, m_axisCellCount);
    const std::int64_t rootMinimumY =
        rootMinimumCoordinate(m_rootIndex.y, m_axisCellCount);
    const std::int64_t rootMinimumZ =
        rootMinimumCoordinate(m_rootIndex.z, m_axisCellCount);

    return VoxelFaceAddress(
        VoxelCellIndex(
            voxelIndex(rootMinimumX + static_cast<std::int64_t>(localX)),
            voxelIndex(rootMinimumY + static_cast<std::int64_t>(localY)),
            voxelIndex(rootMinimumZ + static_cast<std::int64_t>(localZ))),
        direction);
}

}
