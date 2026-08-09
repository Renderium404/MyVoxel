#ifndef MYVOXEL_GEOMETRY_GEOMETRY_POINT_H
#define MYVOXEL_GEOMETRY_GEOMETRY_POINT_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Geometry/Geometry_Object.h"

namespace MyVoxel
{

// 表示三维几何空间中的不可变零维几何对象，不具有拓扑身份、空间放置或显示状态。
class Geometry_Point : public Geometry_Object
{
public:
    // 使用有限三维坐标构造几何点。
    explicit Geometry_Point(const MyMath::Vector3& position);

    /// 几何数据

    // 返回几何点在自身局部几何空间中的三维坐标。
    const MyMath::Vector3& position() const;

private:
    MyMath::Vector3 m_position; // 几何点在自身局部几何空间中的三维坐标。
};

}

#endif // MYVOXEL_GEOMETRY_GEOMETRY_POINT_H