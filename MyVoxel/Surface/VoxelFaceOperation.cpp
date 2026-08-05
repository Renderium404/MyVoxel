
#include "VoxelFaceOperation.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace
{

// 根据两个输入集合容量上限创建结果连续数组。
MyVoxel::VoxelFaceSet::Container createResultContainer(std::size_t capacity)
{
    MyVoxel::VoxelFaceSet::Container result;
    result.reserve(capacity);
    return result;
}

}

namespace MyVoxel
{

VoxelFaceSet VoxelFaceOperation::unite(const VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet::Container result = createResultContainer(left.size() + right.size());

    std::set_union(
        left.begin(),
        left.end(),
        right.begin(),
        right.end(),
        std::back_inserter(result));

    return VoxelFaceSet(std::move(result), VoxelFaceSet::SortedUniqueTag());
}

VoxelFaceSet VoxelFaceOperation::intersect(const VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet::Container result = createResultContainer((std::min)(left.size(), right.size()));

    std::set_intersection(
        left.begin(),
        left.end(),
        right.begin(),
        right.end(),
        std::back_inserter(result));

    return VoxelFaceSet(std::move(result), VoxelFaceSet::SortedUniqueTag());
}

VoxelFaceSet VoxelFaceOperation::subtract(const VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet::Container result = createResultContainer(left.size());

    std::set_difference(
        left.begin(),
        left.end(),
        right.begin(),
        right.end(),
        std::back_inserter(result));

    return VoxelFaceSet(std::move(result), VoxelFaceSet::SortedUniqueTag());
}

bool VoxelFaceOperation::uniteInPlace(VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet result = unite(left, right);

    if (result == left)
    {
        return false;
    }

    left.swap(result);
    return true;
}

bool VoxelFaceOperation::intersectInPlace(VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet result = intersect(left, right);

    if (result == left)
    {
        return false;
    }

    left.swap(result);
    return true;
}

bool VoxelFaceOperation::subtractInPlace(VoxelFaceSet& left, const VoxelFaceSet& right)
{
    VoxelFaceSet result = subtract(left, right);

    if (result == left)
    {
        return false;
    }

    left.swap(result);
    return true;
}

}