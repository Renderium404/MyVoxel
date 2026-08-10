#ifndef MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDER_H
#define MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Builder/Display_CurveBuildOptions.h"
#include "MyVoxel/Display/Object/Display_ObjectManager.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"

namespace MyVoxel
{

class Curve;
class Topology_Edge;

// 将Topology_Edge转换为不可变线显示资源，并将Curve实例绑定为Line Display_Object。
class Display_CurveBuilder
{
public:
    /// 支持判断

    // 判断当前Edge几何是否能够转换为标准线显示资源。
    static bool supports(const Topology_Edge& edge);

    /// 资源创建

    // 将指定局部Topology_Edge离散为不可变Line资源并注册到资源管理器，失败返回零。
    static Display_ResourceId createResource(const Topology_Edge& edge, Display_ResourceManager& resourceManager,
                                             const Display_Color& color,
                                             const Display_CurveBuildOptions& options = Display_CurveBuildOptions());

    /// 对象创建

    // 使用已经存在的对应Line资源创建Curve显示对象，不创建或接管资源生命周期。
    static Display_ObjectId createObject(const Curve& curve, Display_ResourceId resourceId, Display_ObjectManager& objectManager,
                                         Display_ObjectUsage usage = Display_ObjectUsage::Static, double lineWidth = 1.0);

private:
    Display_CurveBuilder() = delete;
};

}

#endif // MYVOXEL_DISPLAY_BUILDER_DISPLAY_CURVEBUILDER_H