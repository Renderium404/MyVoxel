#ifndef MYVOXEL_MODELING_SHAPE_MESHMODELING_H
#define MYVOXEL_MODELING_SHAPE_MESHMODELING_H

#include "MyMath/Matrix4.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

// 复制有效、非空且由调用者确认封闭的内部三角网格并创建Topology_Shape。
Topology_Shape createMesh(const Mesh& mesh);
// 移动有效、非空且由调用者确认封闭的内部三角网格并创建Topology_Shape。
Topology_Shape createMesh(Mesh&& mesh);

/// 空间Shape实例创建

// 复制内部三角网格并使用单位变换创建Shape实例。
Shape makeMesh(const Mesh& mesh);
// 移动内部三角网格并使用单位变换创建Shape实例。
Shape makeMesh(Mesh&& mesh);
// 复制内部三角网格并使用指定可逆仿射变换创建Shape实例。
Shape makeMesh(const Mesh& mesh, const MyMath::Matrix4& localToWorld);
// 移动内部三角网格并使用指定可逆仿射变换创建Shape实例。
Shape makeMesh(Mesh&& mesh, const MyMath::Matrix4& localToWorld);

}
}

#endif // MYVOXEL_MODELING_SHAPE_MESHMODELING_H
