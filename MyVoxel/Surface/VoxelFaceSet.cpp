#include "VoxelFaceSet.h"

#include <algorithm>
#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断指定体素面数组是否严格升序排列。
bool isSortedUnique(const MyVoxel::VoxelFaceSet::Container& faces)
{
    for (std::size_t faceIndex = 1; faceIndex < faces.size(); ++faceIndex)
    {
        if (!(faces[faceIndex - 1] < faces[faceIndex]))
        {
            return false;
        }
    }

    return true;
}

}

namespace MyVoxel
{

VoxelFaceSet::VoxelFaceSet()
{
}

VoxelFaceSet::VoxelFaceSet(Container faces)
{
    assign(std::move(faces));
}
VoxelFaceSet::VoxelFaceSet(VoxelFaceSet&& other)
    : m_faces(std::move(other.m_faces))
{
}

VoxelFaceSet& VoxelFaceSet::operator=(VoxelFaceSet&& other)
{
    if (this != &other)
    {
        m_faces = std::move(other.m_faces);
    }

    return *this;
}

VoxelFaceSet VoxelFaceSet::fromSortedUnique(Container faces)
{
    return VoxelFaceSet(std::move(faces), SortedUniqueTag());
}
VoxelFaceSet::VoxelFaceSet(Container&& faces, SortedUniqueTag)
    : m_faces(std::move(faces))
{
    MYVOXEL_ASSERT_MESSAGE(isSortedUnique(m_faces), "VoxelFaceSet internal faces must be sorted and unique.");
}

/// 集合状态

bool VoxelFaceSet::isEmpty() const
{
    return m_faces.empty();
}

std::size_t VoxelFaceSet::size() const
{
    return m_faces.size();
}

void VoxelFaceSet::clear()
{
    m_faces.clear();
}

void VoxelFaceSet::reserve(std::size_t faceCount)
{
    m_faces.reserve(faceCount);
}

/// 集合查询

bool VoxelFaceSet::contains(const VoxelFaceAddress& face) const
{
    return std::binary_search(m_faces.begin(), m_faces.end(), face);
}

const VoxelFaceSet::Container& VoxelFaceSet::faces() const
{
    return m_faces;
}

VoxelFaceSet::ConstIterator VoxelFaceSet::begin() const
{
    return m_faces.begin();
}

VoxelFaceSet::ConstIterator VoxelFaceSet::end() const
{
    return m_faces.end();
}

/// 单面修改

bool VoxelFaceSet::insert(const VoxelFaceAddress& face)
{
    Container::iterator iterator = std::lower_bound(m_faces.begin(), m_faces.end(), face);

    if (iterator != m_faces.end() && *iterator == face)
    {
        return false;
    }

    m_faces.insert(iterator, face);
    return true;
}

bool VoxelFaceSet::erase(const VoxelFaceAddress& face)
{
    Container::iterator iterator = std::lower_bound(m_faces.begin(), m_faces.end(), face);

    if (iterator == m_faces.end() || *iterator != face)
    {
        return false;
    }

    m_faces.erase(iterator);
    return true;
}

/// 批量修改

void VoxelFaceSet::assign(Container faces)
{
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    m_faces.swap(faces);
}

void VoxelFaceSet::swap(VoxelFaceSet& other)
{
    m_faces.swap(other.m_faces);
}

bool VoxelFaceSet::operator==(const VoxelFaceSet& other) const
{
    return m_faces == other.m_faces;
}

bool VoxelFaceSet::operator!=(const VoxelFaceSet& other) const
{
    return !(*this == other);
}

}