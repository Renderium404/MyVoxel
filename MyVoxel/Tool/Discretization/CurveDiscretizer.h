#ifndef MYVOXEL_TOOL_DISCRETIZATION_CURVEDISCRETIZER_H
#define MYVOXEL_TOOL_DISCRETIZATION_CURVEDISCRETIZER_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Topology/Topology_Curve.h"

namespace MyVoxel
{

// 保存曲线显示离散使用的固定角度分段参数。
struct CurveDiscretizationOptions
{
    CurveDiscretizationOptions();

    // 判断当前离散参数是否有效。
    bool isValid() const;

    double maximumArcAngle; // 一个圆弧显示线段允许覆盖的最大绝对圆心角，单位为弧度。
    std::size_t maximumArcSegmentCount; // 单条圆弧允许生成的最大显示线段数量。
};

// 将Topology_Curve离散为按GL_LINES端点对排列的局部空间点序列。
class CurveDiscretizer
{
public:
    // 返回当前曲线按指定参数需要生成的线段数量。
    static std::size_t segmentCount(const Topology_Curve& curve, const CurveDiscretizationOptions& options = CurveDiscretizationOptions());
    // 将曲线离散线段追加到points，每两个连续点形成一个独立线段。
    static void appendSegments(const Topology_Curve& curve, std::vector<MyMath::Vector3>& points,
                               const CurveDiscretizationOptions& options = CurveDiscretizationOptions());
    // 返回当前曲线完整的GL_LINES端点对序列。
    static std::vector<MyMath::Vector3> buildSegments(const Topology_Curve& curve,
                                                      const CurveDiscretizationOptions& options = CurveDiscretizationOptions());

private:
    CurveDiscretizer() = delete;
};

}

#endif // MYVOXEL_TOOL_DISCRETIZATION_CURVEDISCRETIZER_H
