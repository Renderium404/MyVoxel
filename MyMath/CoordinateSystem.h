#ifndef MYMATH_COORDINATESYSTEM_H
#define MYMATH_COORDINATESYSTEM_H

#include "Matrix4.h"
#include "Quaternion.h"
#include "Vector3.h"

namespace MyMath
{

// 三维正交坐标系，直接继承4×4矩阵并补充坐标系语义。
class CoordinateSystem : public Matrix4
{
public:
    static const double DefaultEpsilon;

public:
    // 构造与世界坐标系重合的单位坐标系。
    CoordinateSystem();

    /// 坐标系创建

    // 创建与世界坐标系重合的单位坐标系。
    static CoordinateSystem identity();

    /// 坐标系创建

    // 使用原点和三个正交单位轴创建坐标系，调用者必须保证坐标轴有效。
    static CoordinateSystem fromAxes(
        const Vector3& origin,
        const Vector3& xAxis,
        const Vector3& yAxis,
        const Vector3& zAxis,
        double epsilon = DefaultEpsilon);

    // 使用原点、X方向和Y参考方向创建右手坐标系，调用者必须保证两个方向不平行。
    static CoordinateSystem fromXY(
        const Vector3& origin,
        const Vector3& xDirection,
        const Vector3& yReference,
        double epsilon = DefaultEpsilon);

    // 使用原点、Y方向和Z参考方向创建右手坐标系，调用者必须保证两个方向不平行。
    static CoordinateSystem fromYZ(
        const Vector3& origin,
        const Vector3& yDirection,
        const Vector3& zReference,
        double epsilon = DefaultEpsilon);

    // 使用原点、Z方向和X参考方向创建右手坐标系，调用者必须保证两个方向不平行。
    static CoordinateSystem fromZX(
        const Vector3& origin,
        const Vector3& zDirection,
        const Vector3& xReference,
        double epsilon = DefaultEpsilon);

    // 使用原点和单位旋转四元数创建右手坐标系，调用者必须保证输入有效。
    static CoordinateSystem fromQuaternion(
        const Vector3& origin,
        const Quaternion& orientation,
        double epsilon = DefaultEpsilon);

    // 使用正交刚体变换矩阵创建坐标系，调用者必须保证矩阵有效。
    static CoordinateSystem fromMatrix(
        const Matrix4& matrix,
        double epsilon = DefaultEpsilon);

    /// 坐标系属性

    // 获取坐标系原点。
    Vector3 origin() const;

    // 获取坐标系X轴。
    Vector3 xAxis() const;

    // 获取坐标系Y轴。
    Vector3 yAxis() const;

    // 获取坐标系Z轴。
    Vector3 zAxis() const;

    // 设置坐标系原点，不改变坐标轴方向。
    bool setOrigin(const Vector3& origin);

    // 判断当前矩阵是否为有效正交坐标系。
    bool isValid(double epsilon = DefaultEpsilon) const;

    // 判断当前坐标系是否为左手坐标系。
    bool isLeftHanded(double epsilon = DefaultEpsilon) const;

    // 获取右手坐标系的旋转四元数，左手坐标系返回false。
    bool orientation(Quaternion& result, double epsilon = DefaultEpsilon) const;

    /// 点、向量和相对坐标系变换

    // 将局部点转换到世界坐标系。
    Vector3 toGlobal(const Vector3& localPoint) const;

    // 将世界坐标点转换到当前局部坐标系。
    Vector3 toLocal(const Vector3& globalPoint) const;

    // 将局部向量映射到世界坐标系，不包含平移。
    Vector3 mapVector(const Vector3& localVector) const;

    // 将世界坐标向量映射到当前局部坐标系。
    Vector3 unmapVector(const Vector3& globalVector) const;

    // 将相对当前坐标系定义的坐标系转换到世界坐标系。
    CoordinateSystem toGlobalFromRelative(const CoordinateSystem& relativeSystem) const;

    // 保留Matrix4的通用逆矩阵计算接口。
    using Matrix4::inverted;

    // 返回当前正交坐标系的快速逆变换。
    CoordinateSystem inverted() const;

    /// 坐标系运动

    // 沿当前局部坐标轴平移坐标系。
    CoordinateSystem& translate(const Vector3& localOffset);

    // 沿世界坐标系方向平移坐标系。
    CoordinateSystem& translateGlobal(const Vector3& globalOffset);

    // 绕当前坐标系中的局部轴自转，角度单位为弧度。
    bool rotate(const Vector3& localAxis, double angle, double epsilon = DefaultEpsilon);

    // 绕当前局部X轴自转，角度单位为弧度。
    bool rotateX(double angle, double epsilon = DefaultEpsilon);

    // 绕当前局部Y轴自转，角度单位为弧度。
    bool rotateY(double angle, double epsilon = DefaultEpsilon);

    // 绕当前局部Z轴自转，角度单位为弧度。
    bool rotateZ(double angle, double epsilon = DefaultEpsilon);

    // 绕世界坐标系中的任意直线公转，角度单位为弧度。
    bool revolve(const Vector3& globalPoint, const Vector3& globalAxis, double angle,
                 double epsilon = DefaultEpsilon);

    // 翻转局部Z轴，切换坐标系手性。
    CoordinateSystem& mirror();

private:
    // 检查三个轴是否构成正交单位坐标系，允许左手或右手。
    static bool validateAxes(const Vector3& xAxis, const Vector3& yAxis,
                             const Vector3& zAxis, double epsilon);

    // 使用已经验证的原点和坐标轴写入矩阵。
    void assign(const Vector3& origin, const Vector3& xAxis,
                const Vector3& yAxis, const Vector3& zAxis);

    // 使用矩阵覆盖当前坐标系。
    void assign(const Matrix4& matrix);
};

}

#endif