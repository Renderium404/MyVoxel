#ifndef MYVOXEL_SURFACE_VOXELFACESET_H
#define MYVOXEL_SURFACE_VOXELFACESET_H

#include <cstddef>
#include <vector>

#include "VoxelFaceAddress.h"

namespace MyVoxel
{

class VoxelFaceOperation;

// 保存一组按照VoxelFaceAddress升序排列且不包含重复项的体素面地址。
class VoxelFaceSet
{
public:
    using Container = std::vector<VoxelFaceAddress>;
    using ConstIterator = Container::const_iterator;
    VoxelFaceSet();
    // 使用一组顺序不限且允许重复的面地址构造集合。
    explicit VoxelFaceSet(Container faces);
    VoxelFaceSet(const VoxelFaceSet&) = default;
    VoxelFaceSet& operator=(const VoxelFaceSet&) = default;

    // 移动构造和移动赋值手动实现，以兼容Visual Studio 2013。
    VoxelFaceSet(VoxelFaceSet&& other);
    VoxelFaceSet& operator=(VoxelFaceSet&& other);

    // 使用已经升序排列且不含重复项的面地址数组构造集合。
    static VoxelFaceSet fromSortedUnique(Container faces);

    /// 集合状态
    // 判断当前集合是否不包含任何体素面。
    bool isEmpty() const;
    // 返回当前集合包含的唯一体素面数量。
    std::size_t size() const;
    // 清空全部体素面。
    void clear();
    // 预留至少能够保存指定数量体素面的连续空间。
    void reserve(std::size_t faceCount);

    /// 集合查询
    // 判断当前集合是否包含指定体素面。
    bool contains(const VoxelFaceAddress& face) const;
    // 返回当前排序面地址数组。
    const Container& faces() const;

    // 返回集合起始只读迭代器。
    ConstIterator begin() const;
    // 返回集合末尾只读迭代器。
    ConstIterator end() const;

    /// 单面修改
    // 插入指定体素面，已存在时不修改集合并返回false。
    bool insert(const VoxelFaceAddress& face);
    // 删除指定体素面，不存在时不修改集合并返回false。
    bool erase(const VoxelFaceAddress& face);

    /// 批量修改
    // 使用一组顺序不限且允许重复的面地址替换当前集合。
    void assign(Container faces);
    // 与另一个面集合交换内部连续存储。
    void swap(VoxelFaceSet& other);

    bool operator==(const VoxelFaceSet& other) const;
    bool operator!=(const VoxelFaceSet& other) const;

private:
    struct SortedUniqueTag
    {
    };

    // 使用已经升序排列并且不含重复项的面地址数组构造集合。
    VoxelFaceSet(Container&& faces, SortedUniqueTag);

    friend class VoxelFaceOperation;

private:
    Container m_faces; // 按VoxelFaceAddress升序排列且不含重复项的面地址。
};

}

#endif // MYVOXEL_SURFACE_VOXELFACESET_H