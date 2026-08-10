#ifndef MYVOXEL_DISPLAY_BUILDER_DISPLAY_WIREBUILDER_H
#define MYVOXEL_DISPLAY_BUILDER_DISPLAY_WIREBUILDER_H

#include <vector>

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Builder/Display_CurveBuildOptions.h"
#include "MyVoxel/Display/Object/Display_ObjectManager.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"

namespace MyVoxel
{

class Topology_Wire;
class Wire;

// 将Topology_Wire的有向Edge序列转换为Line资源集合，并将Wire实例绑定为多Part Line Display_Object。
class Display_WireBuilder
{
public:
    /// 支持判断

    // 判断Wire全部Edge是否能够转换为标准线显示资源。
    static bool supports(const Topology_Wire& wire);

    /// 资源创建

    // 按Wire当前Edge遍历顺序创建一一对应Line资源，任一创建失败时回收本次已经创建的资源并返回空数组。
    static std::vector<Display_ResourceId> createResources(const Topology_Wire& wire, Display_ResourceManager& resourceManager,
                                                           const Display_Color& color,
                                                           const Display_CurveBuildOptions& options = Display_CurveBuildOptions());

    /// 对象创建

    // 使用与Wire Edge顺序一一对应的已有Line资源创建Wire显示对象，不创建或接管资源生命周期。
    static Display_ObjectId createObject(const Wire& wire, const std::vector<Display_ResourceId>& resourceIds,
                                         Display_ObjectManager& objectManager,
                                         Display_ObjectUsage usage = Display_ObjectUsage::Static, double lineWidth = 1.0);

private:
    Display_WireBuilder() = delete;
};

}

#endif // MYVOXEL_DISPLAY_BUILDER_DISPLAY_WIREBUILDER_H