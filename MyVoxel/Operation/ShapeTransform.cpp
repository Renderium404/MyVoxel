#include "ShapeTransform.h"

#include <cassert>
#include <cmath>

namespace
{

// 创建以指定点为中心应用operation的变换矩阵。
MyMath::Matrix4 aroundPoint(const MyMath::Matrix4& operation, const MyMath::Vector3& center)
{
    assert(operation.isAffine());
    assert(center.isFinite());

    const MyMath::Matrix4 moveToCenter = MyMath::Matrix4::fromTranslation(center);
    const MyMath::Matrix4 moveToOrigin = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-center.x(), -center.y(), -center.z()));
    return moveToCenter * operation * moveToOrigin;
}

// 创建指定三个方向缩放比例的仿射矩阵。
MyMath::Matrix4 makeScaleMatrix(const MyMath::Vector3& factors)
{
    assert(factors.isFinite());
    assert(factors.x() > 0.0 && factors.y() > 0.0 && factors.z() > 0.0);

    MyMath::Matrix4 matrix = MyMath::Matrix4::identity();
    matrix(0, 0) = factors.x();
    matrix(1, 1) = factors.y();
    matrix(2, 2) = factors.z();
    return matrix;
}

// 将单位四元数转换为旋转矩阵。
MyMath::Matrix4 makeRotationMatrix(const MyMath::Quaternion& rotation)
{
    assert(rotation.isUnit());
    return rotation.toRotationMatrix();
}

// 根据旋转轴和弧度角创建旋转矩阵。
MyMath::Matrix4 makeRotationMatrix(const MyMath::Vector3& axis, double angle)
{
    assert(axis.isVector());
    assert(std::isfinite(angle));

    const MyMath::Quaternion rotation = MyMath::Quaternion::fromAxisAngle(axis, angle);

    assert(rotation.isUnit());
    return rotation.toRotationMatrix();
}

}

namespace MyVoxel
{

/// 连续Shape通用变换

Shape applyTransform(const Shape& shape, const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());

    Shape result = shape;
    result.setTransform(transform * shape.transform());
    return result;
}

Shape applyLocalTransform(const Shape& shape, const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());

    Shape result = shape;
    result.setTransform(shape.transform() * transform);
    return result;
}

Shape resetTransform(const Shape& shape)
{
    Shape result = shape;
    result.resetTransform();
    return result;
}

/// 体素Shape通用变换

VoxelShape applyTransform(const VoxelShape& shape, const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());

    VoxelShape result = shape;
    result.setTransform(transform * shape.transform());
    return result;
}

VoxelShape applyLocalTransform(const VoxelShape& shape, const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());

    VoxelShape result = shape;
    result.setTransform(shape.transform() * transform);
    return result;
}

VoxelShape resetTransform(const VoxelShape& shape)
{
    VoxelShape result = shape;
    result.resetTransform();
    return result;
}

/// 连续Shape平移

Shape translate(const Shape& shape, const MyMath::Vector3& offset)
{
    assert(offset.isFinite());
    return applyTransform(shape, MyMath::Matrix4::fromTranslation(offset));
}

Shape translateLocal(const Shape& shape, const MyMath::Vector3& offset)
{
    assert(offset.isFinite());
    return applyLocalTransform(shape, MyMath::Matrix4::fromTranslation(offset));
}

/// 体素Shape平移

VoxelShape translate(const VoxelShape& shape, const MyMath::Vector3& offset)
{
    assert(offset.isFinite());
    return applyTransform(shape, MyMath::Matrix4::fromTranslation(offset));
}

VoxelShape translateLocal(const VoxelShape& shape, const MyMath::Vector3& offset)
{
    assert(offset.isFinite());
    return applyLocalTransform(shape, MyMath::Matrix4::fromTranslation(offset));
}

/// 连续Shape旋转

Shape rotate(const Shape& shape, const MyMath::Quaternion& rotation)
{
    return applyTransform(shape, makeRotationMatrix(rotation));
}

Shape rotate(const Shape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeRotationMatrix(rotation), center));
}

Shape rotate(const Shape& shape, const MyMath::Vector3& axis, double angle)
{
    return applyTransform(shape, makeRotationMatrix(axis, angle));
}

Shape rotate(const Shape& shape, const MyMath::Vector3& axis, double angle, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeRotationMatrix(axis, angle), center));
}

Shape rotateLocal(const Shape& shape, const MyMath::Quaternion& rotation)
{
    return applyLocalTransform(shape, makeRotationMatrix(rotation));
}

Shape rotateLocal(const Shape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center)
{
    return applyLocalTransform(shape, aroundPoint(makeRotationMatrix(rotation), center));
}

/// 体素Shape旋转

VoxelShape rotate(const VoxelShape& shape, const MyMath::Quaternion& rotation)
{
    return applyTransform(shape, makeRotationMatrix(rotation));
}

VoxelShape rotate(const VoxelShape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeRotationMatrix(rotation), center));
}

VoxelShape rotate(const VoxelShape& shape, const MyMath::Vector3& axis, double angle)
{
    return applyTransform(shape, makeRotationMatrix(axis, angle));
}

VoxelShape rotate(const VoxelShape& shape, const MyMath::Vector3& axis, double angle, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeRotationMatrix(axis, angle), center));
}

VoxelShape rotateLocal(const VoxelShape& shape, const MyMath::Quaternion& rotation)
{
    return applyLocalTransform(shape, makeRotationMatrix(rotation));
}

VoxelShape rotateLocal(const VoxelShape& shape, const MyMath::Quaternion& rotation, const MyMath::Vector3& center)
{
    return applyLocalTransform(shape, aroundPoint(makeRotationMatrix(rotation), center));
}

/// 连续Shape缩放

Shape scale(const Shape& shape, double factor)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scale(shape, MyMath::Vector3(factor, factor, factor));
}

Shape scale(const Shape& shape, double factor, const MyMath::Vector3& center)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scale(shape, MyMath::Vector3(factor, factor, factor), center);
}

Shape scale(const Shape& shape, const MyMath::Vector3& factors)
{
    return applyTransform(shape, makeScaleMatrix(factors));
}

Shape scale(const Shape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeScaleMatrix(factors), center));
}

Shape scaleLocal(const Shape& shape, double factor)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scaleLocal(shape, MyMath::Vector3(factor, factor, factor));
}

Shape scaleLocal(const Shape& shape, double factor, const MyMath::Vector3& center)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scaleLocal(shape, MyMath::Vector3(factor, factor, factor), center);
}

Shape scaleLocal(const Shape& shape, const MyMath::Vector3& factors)
{
    return applyLocalTransform(shape, makeScaleMatrix(factors));
}

Shape scaleLocal(const Shape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center)
{
    return applyLocalTransform(shape, aroundPoint(makeScaleMatrix(factors), center));
}

/// 体素Shape缩放

VoxelShape scale(const VoxelShape& shape, double factor)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scale(shape, MyMath::Vector3(factor, factor, factor));
}

VoxelShape scale(const VoxelShape& shape, double factor, const MyMath::Vector3& center)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scale(shape, MyMath::Vector3(factor, factor, factor), center);
}

VoxelShape scale(const VoxelShape& shape, const MyMath::Vector3& factors)
{
    return applyTransform(shape, makeScaleMatrix(factors));
}

VoxelShape scale(const VoxelShape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center)
{
    return applyTransform(shape, aroundPoint(makeScaleMatrix(factors), center));
}

VoxelShape scaleLocal(const VoxelShape& shape, double factor)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scaleLocal(shape, MyMath::Vector3(factor, factor, factor));
}

VoxelShape scaleLocal(const VoxelShape& shape, double factor, const MyMath::Vector3& center)
{
    assert(std::isfinite(factor) && factor > 0.0);
    return scaleLocal(shape, MyMath::Vector3(factor, factor, factor), center);
}

VoxelShape scaleLocal(const VoxelShape& shape, const MyMath::Vector3& factors)
{
    return applyLocalTransform(shape, makeScaleMatrix(factors));
}

VoxelShape scaleLocal(const VoxelShape& shape, const MyMath::Vector3& factors, const MyMath::Vector3& center)
{
    return applyLocalTransform(shape, aroundPoint(makeScaleMatrix(factors), center));
}

}