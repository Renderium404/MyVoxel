#ifndef MYVOXEL_DISPLAY_ADAPTER_DISPLAY_WIREADAPTER_H
#define MYVOXEL_DISPLAY_ADAPTER_DISPLAY_WIREADAPTER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineSnapshot.h"
#include "MyVoxel/Tool/Discretization/CurveDiscretizer.h"
#include "MyVoxel/Topology/Wire.h"

namespace MyVoxel
{

// 将Wire空间实例转换为通用Display线对象快照，每条Topology_Curve对应一个稳定分片。
class Display_WireAdapter
{
public:
    // 根据Wire实例建立多分片静态线对象快照，partId固定使用curveIndex+1。
    static Display_LineObjectSnapshot buildSnapshot(Display_LineObjectId objectId, const Wire& wire, const Display_Color& color,
                                                    float width = 1.0f,
                                                    const CurveDiscretizationOptions& options = CurveDiscretizationOptions());

private:
    Display_WireAdapter() = delete;
};

}

#endif // MYVOXEL_DISPLAY_ADAPTER_DISPLAY_WIREADAPTER_H
