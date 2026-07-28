#ifndef MYVOXEL_SHAPETRANSFORM_H
#define MYVOXEL_SHAPETRANSFORM_H

#include "MyMath/Matrix4.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector3.h"

#include "../Core/VoxelShape.h"
#include "../Shape/Shape.h"

namespace MyVoxel
{

/// 通用变换

// 在世界坐标中对连续Shape应用指定仿射变换。
Shape applyTransform(const Shape& shape, const MyMath::Matrix4& transform);

// 在连续Shape局部坐标中应用指定仿射变换。
Shape applyLocalTransform(const Shape& shape, const MyMath::Matrix4& transform);

// 返回变换恢复为单位矩阵的连续Shape。
Shape resetTransform(const Shape& shape);

// 在世界坐标中对体素Shape应用指定仿射变换。
VoxelShape applyTransform(const VoxelShape& shape, const MyMath::Matrix4& transform);

// 在体素Shape局部坐标中应用指定仿射变换。
VoxelShape applyLocalTransform(const VoxelShape& shape, const MyMath::Matrix4& transform);

// 返回变换恢复为单位矩阵的体素Shape。
VoxelShape resetTransform(const VoxelShape& shape);

/// 平移变换

// 在世界坐标中平移连续Shape。
Shape translate(const Shape& shape, const MyMath::Vector3& offset);

// 在连续Shape局部坐标中平移。
Shape translateLocal(const Shape& shape, const MyMath::Vector3& offset);

// 在世界坐标中平移体素Shape。
VoxelShape translate(const VoxelShape& shape, const MyMath::Vector3& offset);

// 在体素Shape局部坐标中平移。
VoxelShape translateLocal(const VoxelShape& shape, const MyMath::Vector3& offset);

/// 旋转变换

// 在世界坐标中绕原点对连续Shape应用指定旋转。
Shape rotate(const Shape& shape, const MyMath::Quaternion& rotation);

// 在世界坐标中绕指定中心对连续Shape应用指定旋转。
Shape rotate(const Shape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center);

// 在世界坐标中绕原点和指定轴旋转连续Shape，angle使用弧度。
Shape rotate(const Shape& shape, const MyMath::Vector3& axis, double angle);

// 在世界坐标中绕指定中心和指定轴旋转连续Shape，angle使用弧度。
Shape rotate(const Shape& shape, const MyMath::Vector3& axis, double angle, const MyMath::Vector3& center);

// 在连续Shape局部坐标中绕局部原点应用指定旋转。
Shape rotateLocal(const Shape& shape, const MyMath::Quaternion& rotation);

// 在连续Shape局部坐标中绕指定局部中心应用指定旋转。
Shape rotateLocal(const Shape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center);

// 在世界坐标中绕原点对体素Shape应用指定旋转。
VoxelShape rotate(const VoxelShape& shape, const MyMath::Quaternion& rotation);

// 在世界坐标中绕指定中心对体素Shape应用指定旋转。
VoxelShape rotate(const VoxelShape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center);

// 在世界坐标中绕原点和指定轴旋转体素Shape，angle使用弧度。
VoxelShape rotate(const VoxelShape& shape, const MyMath::Vector3& axis, double angle);

// 在世界坐标中绕指定中心和指定轴旋转体素Shape，angle使用弧度。
VoxelShape rotate(const VoxelShape& shape, const MyMath::Vector3& axis, double angle, const MyMath::Vector3& center);

// 在体素Shape局部坐标中绕局部原点应用指定旋转。
VoxelShape rotateLocal(const VoxelShape& shape, const MyMath::Quaternion& rotation);

// 在体素Shape局部坐标中绕指定局部中心应用指定旋转。
VoxelShape rotateLocal(const VoxelShape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center);

/// 缩放变换

// 在世界坐标中以原点为中心对连续Shape进行等比缩放。
Shape scale(const Shape& shape, double factor);

// 在世界坐标中以指定中心对连续Shape进行等比缩放。
Shape scale(const Shape& shape, double factor, const MyMath::Vector3& center);

// 在世界坐标中以原点为中心对连续Shape进行分轴缩放。
Shape scale(const Shape& shape, const MyMath::Vector3& factors);

// 在世界坐标中以指定中心对连续Shape进行分轴缩放。
Shape scale(const Shape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center);

// 在连续Shape局部坐标中以局部原点为中心进行等比缩放。
Shape scaleLocal(const Shape& shape, double factor);

// 在连续Shape局部坐标中以指定局部中心进行等比缩放。
Shape scaleLocal(const Shape& shape, double factor, const MyMath::Vector3& center);

// 在连续Shape局部坐标中以局部原点为中心进行分轴缩放。
Shape scaleLocal(const Shape& shape, const MyMath::Vector3& factors);

// 在连续Shape局部坐标中以指定局部中心进行分轴缩放。
Shape scaleLocal(const Shape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center);

// 在世界坐标中以原点为中心对体素Shape进行等比缩放。
VoxelShape scale(const VoxelShape& shape, double factor);

// 在世界坐标中以指定中心对体素Shape进行等比缩放。
VoxelShape scale(const VoxelShape& shape, double factor, const MyMath::Vector3& center);

// 在世界坐标中以原点为中心对体素Shape进行分轴缩放。
VoxelShape scale(const VoxelShape& shape, const MyMath::Vector3& factors);

// 在世界坐标中以指定中心对体素Shape进行分轴缩放。
VoxelShape scale(const VoxelShape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center);

// 在体素Shape局部坐标中以局部原点为中心进行等比缩放。
VoxelShape scaleLocal(const VoxelShape& shape, double factor);

// 在体素Shape局部坐标中以指定局部中心进行等比缩放。
VoxelShape scaleLocal(const VoxelShape& shape, double factor, const MyMath::Vector3& center);

// 在体素Shape局部坐标中以局部原点为中心进行分轴缩放。
VoxelShape scaleLocal(const VoxelShape& shape, const MyMath::Vector3& factors);

// 在体素Shape局部坐标中以指定局部中心进行分轴缩放。
VoxelShape scaleLocal(const VoxelShape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center);

}

#endif // MYVOXEL_SHAPETRANSFORM_H