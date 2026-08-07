#ifndef MYVOXEL_DISPLAY_ADAPTER_DISPLAY_CURVEADAPTER_H
#define MYVOXEL_DISPLAY_ADAPTER_DISPLAY_CURVEADAPTER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineSnapshot.h"
#include "MyVoxel/Tool/Discretization/CurveDiscretizer.h"
#include "MyVoxel/Topology/Curve.h"

namespace MyVoxel
{

// 将Curve空间实例转换为通用Display线对象快照。
class Display_CurveAdapter
{
public:
    // 根据Curve实例建立单分片静态线对象快照，曲线局部变换直接作为对象localToWorld。
    static Display_LineObjectSnapshot buildSnapshot(Display_LineObjectId objectId, const Curve& curve, const Display_Color& color,
                                                    float width = 1.0f,
                                                    const CurveDiscretizationOptions& options = CurveDiscretizationOptions());

private:
    Display_CurveAdapter() = delete;
};

}

#endif // MYVOXEL_DISPLAY_ADAPTER_DISPLAY_CURVEADAPTER_H
