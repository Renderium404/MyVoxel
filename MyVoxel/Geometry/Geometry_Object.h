#ifndef MYVOXEL_GEOMETRY_GEOMETRY_OBJECT_H
#define MYVOXEL_GEOMETRY_GEOMETRY_OBJECT_H

#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 作为全部连续几何对象的共享生命周期基类，不定义具体几何类型、维度或空间放置。
class Geometry_Object : public Foundation::ReferenceCounted
{
protected:
    // 构造引用计数为零的连续几何对象。
    Geometry_Object();
    // 通过Foundation::RefPtr释放最终引用时销毁具体几何对象。
    ~Geometry_Object() override;
};

}

#endif // MYVOXEL_GEOMETRY_GEOMETRY_OBJECT_H