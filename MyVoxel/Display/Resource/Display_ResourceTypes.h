#ifndef MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCETYPES_H
#define MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCETYPES_H

#include <cstdint>

namespace MyVoxel
{

typedef std::uint64_t Display_ResourceId;

// 标识不可变CPU显示资源的数据类型。
enum class Display_ResourceKind
{
    Mesh,       //三角网格
    Line        //线段
};

}

#endif // MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCETYPES_H