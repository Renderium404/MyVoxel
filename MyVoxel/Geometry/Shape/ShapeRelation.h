#ifndef MYVOXEL_GEOMETRY_SHAPERELATION_H
#define MYVOXEL_GEOMETRY_SHAPERELATION_H

namespace MyVoxel
{


// 表示一个空间范围与连续几何体之间的保守空间关系。
enum class ShapeRelation
{
    Outside,       // 空间范围完全位于几何体外部。
    Inside,        // 空间范围完全位于几何体内部。
    Intersecting   // 空间范围与几何体边界相交，或无法保守确定内外关系。
};

}

#endif // MYVOXEL_GEOMETRY_SHAPERELATION_H