#include "VoxelFaceColorMap.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 按体素面地址比较颜色记录与查询面。
struct ColorEntryFaceLess
{
    bool operator()(const MyVoxel::VoxelFaceColorMap::Entry& entry,
                    const MyVoxel::VoxelFaceAddress& face) const
    {
        return entry.first < face;
    }
};

// 返回指定面在连续颜色映射中的插入位置。
MyVoxel::VoxelFaceColorMap::Container::iterator lowerBound(
    MyVoxel::VoxelFaceColorMap::Container& colors,
    const MyVoxel::VoxelFaceAddress& face)
{
    return std::lower_bound(colors.begin(), colors.end(), face, ColorEntryFaceLess());
}

// 返回指定面在连续颜色映射中的只读插入位置。
MyVoxel::VoxelFaceColorMap::Container::const_iterator lowerBound(
    const MyVoxel::VoxelFaceColorMap::Container& colors,
    const MyVoxel::VoxelFaceAddress& face)
{
    return std::lower_bound(colors.begin(), colors.end(), face, ColorEntryFaceLess());
}

}

namespace MyVoxel
{

VoxelFaceColorMap::Entry::Entry(const VoxelFaceAddress& face, const Display_Color& color)
    : first(face)
    , second(color)
{
}

VoxelFaceColorMap::VoxelFaceColorMap()
{
}

VoxelFaceColorMap::VoxelFaceColorMap(VoxelFaceColorMap&& other)
    : m_colors(std::move(other.m_colors))
{
}

VoxelFaceColorMap& VoxelFaceColorMap::operator=(VoxelFaceColorMap&& other)
{
    if (this != &other)
    {
        m_colors = std::move(other.m_colors);
    }

    return *this;
}

/// 映射状态

bool VoxelFaceColorMap::isEmpty() const
{
    return m_colors.empty();
}

std::size_t VoxelFaceColorMap::size() const
{
    return m_colors.size();
}

void VoxelFaceColorMap::clear()
{
    m_colors.clear();
}

void VoxelFaceColorMap::reserve(std::size_t capacity)
{
    m_colors.reserve(capacity);
}

void VoxelFaceColorMap::appendSorted(const VoxelFaceColorMap& other)
{
    if (other.m_colors.empty())
    {
        return;
    }

    if (!m_colors.empty())
    {
        MYVOXEL_ASSERT_MESSAGE(
            m_colors.back().first < other.m_colors.front().first,
            "Appended voxel face colors must follow the current address range.");
    }

    if (other.m_colors.size() > m_colors.max_size() - m_colors.size())
    {
        throw std::length_error("Combined voxel face color map exceeds vector capacity.");
    }

    m_colors.reserve(m_colors.size() + other.m_colors.size());

    for (Container::const_iterator iterator = other.m_colors.begin();
         iterator != other.m_colors.end();
         ++iterator)
    {
        m_colors.push_back(*iterator);
    }
}

/// 颜色查询

bool VoxelFaceColorMap::contains(const VoxelFaceAddress& face) const
{
    return find(face) != nullptr;
}

const Display_Color* VoxelFaceColorMap::find(const VoxelFaceAddress& face) const
{
    const Container::const_iterator iterator = lowerBound(m_colors, face);

    if (iterator == m_colors.end() || face < iterator->first)
    {
        return nullptr;
    }

    return &iterator->second;
}

const Display_Color& VoxelFaceColorMap::colorOrDefault(
    const VoxelFaceAddress& face,
    const Display_Color& defaultColor) const
{
    const Display_Color* color = find(face);
    return color ? *color : defaultColor;
}

VoxelFaceColorMap::ConstIterator VoxelFaceColorMap::begin() const
{
    return m_colors.begin();
}

VoxelFaceColorMap::ConstIterator VoxelFaceColorMap::end() const
{
    return m_colors.end();
}

/// 单面修改

bool VoxelFaceColorMap::set(const VoxelFaceAddress& face, const Display_Color& color)
{
    Container::iterator iterator = lowerBound(m_colors, face);

    if (iterator != m_colors.end() && !(face < iterator->first))
    {
        if (iterator->second == color)
        {
            return false;
        }

        iterator->second = color;
        return true;
    }

    m_colors.insert(iterator, Entry(face, color));
    return true;
}

bool VoxelFaceColorMap::erase(const VoxelFaceAddress& face)
{
    Container::iterator iterator = lowerBound(m_colors, face);

    if (iterator == m_colors.end() || face < iterator->first)
    {
        return false;
    }

    m_colors.erase(iterator);
    return true;
}

/// 批量修改

std::size_t VoxelFaceColorMap::set(const VoxelFaceSet& faces, const Display_Color& color)
{
    if (faces.isEmpty())
    {
        return 0;
    }

    Container updatedColors;

    if (faces.size() > updatedColors.max_size() - m_colors.size())
    {
        throw std::length_error("Voxel face color map size exceeds vector capacity.");
    }

    updatedColors.reserve(m_colors.size() + faces.size());

    Container::const_iterator colorIterator = m_colors.begin();
    VoxelFaceSet::ConstIterator faceIterator = faces.begin();
    std::size_t changedCount = 0;

    while (colorIterator != m_colors.end() && faceIterator != faces.end())
    {
        if (colorIterator->first < *faceIterator)
        {
            updatedColors.push_back(*colorIterator);
            ++colorIterator;
            continue;
        }

        if (*faceIterator < colorIterator->first)
        {
            updatedColors.push_back(Entry(*faceIterator, color));
            ++faceIterator;
            ++changedCount;
            continue;
        }

        updatedColors.push_back(Entry(*faceIterator, color));

        if (colorIterator->second != color)
        {
            ++changedCount;
        }

        ++colorIterator;
        ++faceIterator;
    }

    for (; colorIterator != m_colors.end(); ++colorIterator)
    {
        updatedColors.push_back(*colorIterator);
    }

    for (; faceIterator != faces.end(); ++faceIterator)
    {
        updatedColors.push_back(Entry(*faceIterator, color));
        ++changedCount;
    }

    m_colors.swap(updatedColors);
    return changedCount;
}

std::size_t VoxelFaceColorMap::erase(const VoxelFaceSet& faces)
{
    if (m_colors.empty() || faces.isEmpty())
    {
        return 0;
    }

    Container updatedColors;
    updatedColors.reserve(m_colors.size());

    Container::const_iterator colorIterator = m_colors.begin();
    VoxelFaceSet::ConstIterator faceIterator = faces.begin();
    std::size_t erasedCount = 0;

    while (colorIterator != m_colors.end())
    {
        while (faceIterator != faces.end() && *faceIterator < colorIterator->first)
        {
            ++faceIterator;
        }

        if (faceIterator != faces.end() && colorIterator->first == *faceIterator)
        {
            ++colorIterator;
            ++faceIterator;
            ++erasedCount;
            continue;
        }

        updatedColors.push_back(*colorIterator);
        ++colorIterator;
    }

    m_colors.swap(updatedColors);
    return erasedCount;
}

std::size_t VoxelFaceColorMap::retain(const VoxelFaceSet& existingFaces)
{
    if (m_colors.empty())
    {
        return 0;
    }

    Container retainedColors;
    retainedColors.reserve((std::min)(m_colors.size(), existingFaces.size()));

    Container::const_iterator colorIterator = m_colors.begin();
    VoxelFaceSet::ConstIterator faceIterator = existingFaces.begin();

    while (colorIterator != m_colors.end() && faceIterator != existingFaces.end())
    {
        if (colorIterator->first < *faceIterator)
        {
            ++colorIterator;
            continue;
        }

        if (*faceIterator < colorIterator->first)
        {
            ++faceIterator;
            continue;
        }

        retainedColors.push_back(*colorIterator);
        ++colorIterator;
        ++faceIterator;
    }

    const std::size_t erasedCount = m_colors.size() - retainedColors.size();
    m_colors.swap(retainedColors);
    return erasedCount;
}

}
