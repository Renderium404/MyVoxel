#ifndef MYVOXEL_DISPLAY_BUILDER_DISPLAY_SHAPEBUILDER_H
#define MYVOXEL_DISPLAY_BUILDER_DISPLAY_SHAPEBUILDER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Object/Display_ObjectManager.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"
#include "MyVoxel/Mesh/ShapeMesher.h"

namespace MyVoxel
{

class Shape;
class Topology_Shape;

// 将Topology_Shape转换为不可变Mesh显示资源，并将Shape实例绑定为Mesh Display_Object。
class Display_ShapeBuilder
{
public:
    /// 支持判断

    // 判断指定Topology_Shape是否具有当前标准ShapeMesher实现。
    static bool supports(const Topology_Shape& topology);

    /// 资源创建

    // 将局部Topology_Shape转换为可渲染Mesh资源并注册到资源管理器，失败返回零。
    static Display_ResourceId createResource(const Topology_Shape& topology, Display_ResourceManager& resourceManager,
                                             const Display_Color& color,
                                             const ShapeMeshingOptions& options = ShapeMeshingOptions());

    /// 对象创建

    // 使用已经存在的对应Mesh资源创建Shape显示对象，不创建或接管资源生命周期。
    static Display_ObjectId createObject(const Shape& shape, Display_ResourceId resourceId, Display_ObjectManager& objectManager,
                                         Display_ObjectUsage usage = Display_ObjectUsage::Static);

private:
    Display_ShapeBuilder() = delete;
};

}

#endif // MYVOXEL_DISPLAY_BUILDER_DISPLAY_SHAPEBUILDER_H