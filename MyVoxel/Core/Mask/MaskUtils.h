#ifndef MYVOXEL_CORE_MASK_MASKUTILS_H
#define MYVOXEL_CORE_MASK_MASKUTILS_H

#include <cassert>
#include <cstdint>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

/// 八位掩码

// 返回指定角点对应的八位掩码位。
inline std::uint8_t cornerBit(VoxelCorner corner)
{
    const unsigned int cornerIndex = static_cast<unsigned int>(corner);
    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint8_t>(1U << cornerIndex);
}

// 返回八位掩码中指定角点是否置位。
inline bool maskBit(std::uint8_t mask, VoxelCorner corner)
{
    return (mask & cornerBit(corner)) != 0;
}

// 设置八位掩码中指定角点。
inline void setMaskBit(std::uint8_t& mask, VoxelCorner corner, bool value)
{
    const std::uint8_t bit = cornerBit(corner);
    mask = value ? static_cast<std::uint8_t>(mask | bit) : static_cast<std::uint8_t>(mask & static_cast<std::uint8_t>(~bit));
}

// 返回八位掩码中的置位数量。
inline unsigned int maskBitCount(std::uint8_t mask)
{
    unsigned int value = static_cast<unsigned int>(mask);
    value = value - ((value >> 1U) & 0x55U);
    value = (value & 0x33U) + ((value >> 2U) & 0x33U);
    return (value + (value >> 4U)) & 0x0FU;
}

// 返回非零八位掩码中最低置位索引。
inline unsigned int firstMaskBitIndex(std::uint8_t mask)
{
    assert(mask != 0);

#if defined(_MSC_VER)
    unsigned long bitIndex = 0;
    _BitScanForward(&bitIndex, static_cast<unsigned long>(mask));
    return static_cast<unsigned int>(bitIndex);
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned int>(__builtin_ctz(static_cast<unsigned int>(mask)));
#else
    unsigned int bitIndex = 0;

    while ((mask & static_cast<std::uint8_t>(1U)) == 0)
    {
        mask = static_cast<std::uint8_t>(mask >> 1U);
        ++bitIndex;
    }

    return bitIndex;
#endif
}

// 返回最低置位索引并从原八位掩码中删除该位。
inline unsigned int takeFirstMaskBitIndex(std::uint8_t& mask)
{
    const unsigned int bitIndex = firstMaskBitIndex(mask);
    mask = static_cast<std::uint8_t>(mask & static_cast<std::uint8_t>(mask - 1U));
    return bitIndex;
}

/// 六十四位掩码

// 返回指定0~63样本索引对应的64位掩码位。
inline std::uint64_t maskBitValue(unsigned int bitIndex)
{
    assert(bitIndex < 64U);
    return static_cast<std::uint64_t>(1ULL << bitIndex);
}

// 返回64位掩码中指定样本是否置位。
inline bool maskBit(std::uint64_t mask, unsigned int bitIndex)
{
    return (mask & maskBitValue(bitIndex)) != 0;
}

// 设置64位掩码中指定样本。
inline void setMaskBit(std::uint64_t& mask, unsigned int bitIndex, bool value)
{
    const std::uint64_t bit = maskBitValue(bitIndex);
    mask = value ? mask | bit : mask & ~bit;
}

// 返回64位掩码中的置位数量。
inline unsigned int maskBitCount(std::uint64_t mask)
{
    const std::uint64_t pairMask = 0x5555555555555555ULL; // 64位并行置位计数相邻位掩码。
    const std::uint64_t twoBitMask = 0x3333333333333333ULL; // 64位并行置位计数两位组掩码。
    const std::uint64_t nibbleMask = 0x0F0F0F0F0F0F0F0FULL; // 64位并行置位计数半字节掩码。
    const std::uint64_t byteMultiplier = 0x0101010101010101ULL; // 将局部计数累加到最高字节。

    mask = mask - ((mask >> 1U) & pairMask);
    mask = (mask & twoBitMask) + ((mask >> 2U) & twoBitMask);
    mask = (mask + (mask >> 4U)) & nibbleMask;
    return static_cast<unsigned int>((mask * byteMultiplier) >> 56U);
}

// 返回非零64位掩码中最低置位索引。
inline unsigned int firstMaskBitIndex(std::uint64_t mask)
{
    assert(mask != 0);

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

    assert(found != 0);
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

// 返回最低置位索引并从原64位掩码中删除该位。
inline unsigned int takeFirstMaskBitIndex(std::uint64_t& mask)
{
    const unsigned int bitIndex = firstMaskBitIndex(mask);
    mask &= mask - static_cast<std::uint64_t>(1ULL);
    return bitIndex;
}

}

#endif // MYVOXEL_CORE_MASK_MASKUTILS_H