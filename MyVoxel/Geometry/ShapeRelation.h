#ifndef MYVOXEL_GEOMETRY_SHAPERELATION_H
#define MYVOXEL_GEOMETRY_SHAPERELATION_H

namespace MyVoxel
{
namespace Geometry
{

// 表示一个空间范围与连续几何体之间的保守空间关系。
enum class ShapeRelation
{
    Outside,                //外部
    Inside,                 //包含
    Intersecting            //相交
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPERELATION_H