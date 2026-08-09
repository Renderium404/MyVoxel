#ifndef MYVOXEL_GEOMETRY_CURVE_CURVEKIND_H
#define MYVOXEL_GEOMETRY_CURVE_CURVEKIND_H

namespace MyVoxel
{

// 标识有限有向曲线几何的标准类型。
enum class CurveKind
{
    Unknown, // 无法识别的曲线类型。
    Line, // 有限有向直线段。
    Arc // 有限有向圆弧段。
};

}

#endif // MYVOXEL_GEOMETRY_CURVE_CURVEKIND_H
