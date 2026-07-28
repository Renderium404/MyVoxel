#ifndef MYVOXEL_SHAPEREGIONRELATION_H
#define MYVOXEL_SHAPEREGIONRELATION_H

namespace MyVoxel
{

// 表示局部空间区域与连续Shape材料之间的保守关系。
enum class ShapeRegionRelation
{
    Outside, // 查询区域与Shape材料完全不相交。
    Intersecting, // 查询区域与Shape材料相交或当前无法确定完整关系。
    Inside // 查询区域完全位于Shape材料内部。
};

}

#endif // MYVOXEL_SHAPEREGIONRELATION_H