#ifndef MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCE_H
#define MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCE_H

#include <cstddef>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Resource/Display_ResourceTypes.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 作为全部不可变CPU显示资源的共享生命周期基类，只定义资源类型、局部范围和内存统计。
//
// Display_Resource不保存资源标识、对象变换、可见状态、GPU句柄或显示对象关系。
class Display_Resource : public Foundation::ReferenceCounted
{
public:
    /// 资源属性

    // 返回当前显示资源的标准类型。
    virtual Display_ResourceKind kind() const = 0;
    // 判断当前显示资源是否包含完整有效数据。
    virtual bool isValid() const = 0;
    // 返回当前显示资源在自身局部坐标系中的轴对齐包围盒。
    virtual const Bounds3& localBounds() const = 0;
    // 返回当前资源实际连续数据占用的字节数，不包含容器预留空间和对象自身开销。
    virtual std::size_t memoryByteSize() const = 0;

protected:
    // 构造引用计数为零的显示资源。
    Display_Resource();
    // 通过最终共享引用释放具体显示资源。
    ~Display_Resource() override;
};

}

#endif // MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCE_H