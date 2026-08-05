#ifndef MYVOXEL_SURFACE_VOXELFACEOPERATION_H
#define MYVOXEL_SURFACE_VOXELFACEOPERATION_H

#include "VoxelFaceSet.h"

namespace MyVoxel
{

// 提供两个体素面集合之间的并集、交集和差集运算。
//
// 这些函数只处理面地址集合，不执行体积布尔运算，也不改变VoxelShape。
class VoxelFaceOperation
{
public:
    /// 返回新集合

    // 返回left和right包含的全部体素面。
    static VoxelFaceSet unite(const VoxelFaceSet& left, const VoxelFaceSet& right);

    // 返回left和right同时包含的体素面。
    static VoxelFaceSet intersect(const VoxelFaceSet& left, const VoxelFaceSet& right);

    // 返回left中存在但right中不存在的体素面。
    static VoxelFaceSet subtract(const VoxelFaceSet& left, const VoxelFaceSet& right);

    /// 原地集合运算

    // 将right合并到left，返回left是否发生变化。
    static bool uniteInPlace(VoxelFaceSet& left, const VoxelFaceSet& right);

    // 仅保留left和right同时包含的体素面，返回left是否发生变化。
    static bool intersectInPlace(VoxelFaceSet& left, const VoxelFaceSet& right);

    // 从left中删除right包含的体素面，返回left是否发生变化。
    static bool subtractInPlace(VoxelFaceSet& left, const VoxelFaceSet& right);

private:
    VoxelFaceOperation() = delete;
};

}

#endif // MYVOXEL_SURFACE_VOXELFACEOPERATION_H