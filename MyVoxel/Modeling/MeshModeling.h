#ifndef MYVOXEL_MODELING_MESHMODELING_H
#define MYVOXEL_MODELING_MESHMODELING_H

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Shape.h"

namespace MyVoxel
{
namespace Modeling
{

/// 三角网格几何创建

// 复制有效、非空且封闭的内部三角网格并创建连续Shape。
Geometry::Shape makeMesh(const Geometry::Mesh& mesh);

// 移动有效、非空且封闭的内部三角网格并创建连续Shape。
Geometry::Shape makeMesh(Geometry::Mesh&& mesh);

}
}

#endif // MYVOXEL_MODELING_MESHMODELING_H
