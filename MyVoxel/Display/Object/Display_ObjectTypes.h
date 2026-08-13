#ifndef MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTTYPES_H
#define MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTTYPES_H

#include <cstdint>

namespace MyVoxel
{

typedef std::uint64_t Display_ObjectId;
typedef std::uint64_t Display_ObjectPartId;

// 标识显示对象资源在GPU侧的预期更新频率。
enum class Display_ObjectUsage
{
    Static, // 对象资源创建后很少替换。
    Dynamic // 对象资源会持续执行整体或分片替换。
};

}

#endif // MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTTYPES_H