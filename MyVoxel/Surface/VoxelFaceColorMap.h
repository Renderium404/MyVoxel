#ifndef MYVOXEL_SURFACE_VOXELFACECOLORMAP_H
#define MYVOXEL_SURFACE_VOXELFACECOLORMAP_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Display/Base/Display_Color.h"

#include "VoxelFaceAddress.h"
#include "VoxelFaceSet.h"

namespace MyVoxel
{

// 稀疏保存需要单独上色的体素面及其颜色。
//
// 没有保存在当前映射中的面由网格构建器使用调用者指定的默认颜色。
// 颜色记录按照VoxelFaceAddress升序连续保存，以降低批量复制、提交和查询成本。
class VoxelFaceColorMap
{
public:
    // 保存一个体素面地址及其单独颜色。
    struct Entry
    {
        Entry(const VoxelFaceAddress& face, const Display_Color& color);

        VoxelFaceAddress first; // 单独上色的体素面地址。
        Display_Color second; // 当前体素面的单独颜色。
    };

    using Container = std::vector<Entry>;
    using ConstIterator = Container::const_iterator;

    VoxelFaceColorMap();

    VoxelFaceColorMap(const VoxelFaceColorMap&) = default;
    VoxelFaceColorMap& operator=(const VoxelFaceColorMap&) = default;

    // 移动构造和移动赋值手动实现，以兼容Visual Studio 2013。
    VoxelFaceColorMap(VoxelFaceColorMap&& other);
    VoxelFaceColorMap& operator=(VoxelFaceColorMap&& other);

    /// 映射状态

    // 判断当前映射是否没有任何单独上色的体素面。
    bool isEmpty() const;

    // 返回当前单独上色的体素面数量。
    std::size_t size() const;

    // 清空全部体素面颜色。
    void clear();

    // 预留至少capacity条颜色记录的连续存储空间。
    void reserve(std::size_t capacity);

    // 将地址严格大于当前末项的有序颜色映射追加到当前映射。
    void appendSorted(const VoxelFaceColorMap& other);

    /// 颜色查询

    // 判断指定体素面是否具有单独颜色。
    bool contains(const VoxelFaceAddress& face) const;

    // 返回指定体素面的颜色，不存在单独颜色时返回空指针。
    const Display_Color* find(const VoxelFaceAddress& face) const;

    // 返回指定体素面的单独颜色，不存在时返回defaultColor。
    const Display_Color& colorOrDefault(
        const VoxelFaceAddress& face,
        const Display_Color& defaultColor) const;

    // 返回颜色映射起始只读迭代器。
    ConstIterator begin() const;

    // 返回颜色映射末尾只读迭代器。
    ConstIterator end() const;

    /// 单面修改

    // 设置指定体素面的颜色，新增或颜色发生变化时返回true。
    bool set(const VoxelFaceAddress& face, const Display_Color& color);

    // 删除指定体素面的单独颜色，不存在时返回false。
    bool erase(const VoxelFaceAddress& face);

    /// 批量修改

    // 为指定面集合设置相同颜色，返回新增或颜色发生变化的面数量。
    std::size_t set(const VoxelFaceSet& faces, const Display_Color& color);

    // 删除指定面集合中的全部单独颜色，返回实际删除数量。
    std::size_t erase(const VoxelFaceSet& faces);

    // 仅保留existingFaces中仍然存在的颜色记录，返回被删除的颜色数量。
    std::size_t retain(const VoxelFaceSet& existingFaces);

private:
    Container m_colors; // 按VoxelFaceAddress升序连续保存的稀疏面颜色映射。
};

}

#endif // MYVOXEL_SURFACE_VOXELFACECOLORMAP_H
