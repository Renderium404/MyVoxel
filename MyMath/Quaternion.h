#ifndef MYMATH_QUATERNION_H
#define MYMATH_QUATERNION_H

#include "Matrix3.h"
#include "Vector3.h"

namespace MyMath
{

// 表示三维旋转的双精度四元数，分量顺序为w、x、y、z。
class Quaternion
{
public:
    static const double DefaultEpsilon;

    // 构造单位四元数。
    Quaternion();

    // 使用实部和三个虚部分量构造四元数。
    Quaternion(double w, double x, double y, double z);

    /// 四元数创建

    // 创建零四元数。
    static Quaternion zero();

    // 创建单位四元数。
    static Quaternion identity();

    // 根据旋转轴和弧度角创建单位四元数，旋转轴或角度无效时返回零四元数。
    static Quaternion fromAxisAngle(const Vector3& axis, double angle, double epsilon = DefaultEpsilon);

    // 根据右手正交旋转矩阵创建单位四元数，矩阵无效时返回零四元数。
    static Quaternion fromRotationMatrix(const Matrix3& matrix, double epsilon = DefaultEpsilon);

    /// 分量访问

    // 返回实部分量。
    double w() const;

    // 返回X虚部分量。
    double x() const;

    // 返回Y虚部分量。
    double y() const;

    // 返回Z虚部分量。
    double z() const;

    // 设置实部分量。
    void setW(double w);

    // 设置X虚部分量。
    void setX(double x);

    // 设置Y虚部分量。
    void setY(double y);

    // 设置Z虚部分量。
    void setZ(double z);

    // 同时设置全部四个分量。
    void set(double w, double x, double y, double z);

    /// 状态判断

    // 判断所有分量是否为有限值。
    bool isFinite() const;

    // 判断四元数模长是否不大于指定误差。
    bool isZero(double epsilon = DefaultEpsilon) const;

    // 判断四元数是否表示单位旋转，正负单位四元数均视为单位旋转。
    bool isIdentity(double epsilon = DefaultEpsilon) const;

    // 判断四元数模长是否近似为1。
    bool isUnit(double epsilon = DefaultEpsilon) const;

    // 判断两个四元数的四维距离是否不大于指定误差。
    bool isEqualTo(const Quaternion& other, double epsilon = DefaultEpsilon) const;

    // 判断两个单位四元数是否表示相同旋转。
    bool isSameRotation(const Quaternion& other, double epsilon = DefaultEpsilon) const;

    /// 长度与归一化

    // 返回四元数模长平方，结果可能因数值范围而溢出。
    double lengthSquared() const;

    // 返回使用缩放计算的四元数模长。
    double length() const;

    // 返回单位四元数，当前四元数不能归一化时返回零四元数。
    Quaternion normalized(double epsilon = DefaultEpsilon) const;

    // 将当前四元数归一化，当前四元数不能归一化时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);

    /// 四元数变换

    // 返回四元数共轭。
    Quaternion conjugated() const;

    // 返回四元数逆，当前四元数不可逆时返回零四元数。
    Quaternion inverted(double epsilon = DefaultEpsilon) const;

    // 将单位四元数转换为旋转轴和弧度角，调用者必须保证当前四元数为单位四元数。
    void toAxisAngle(Vector3& axis, double& angle, double epsilon = DefaultEpsilon) const;

    // 将单位四元数转换为三阶旋转矩阵，当前四元数不是单位四元数时返回零矩阵。
    Matrix3 toRotationMatrix(double epsilon = DefaultEpsilon) const;

    // 使用单位四元数旋转三维数据，调用者必须保证四元数为单位四元数且输入有限。
    Vector3 rotateVector(const Vector3& vector, double epsilon = DefaultEpsilon) const;

    /// 插值计算

    // 返回两个单位四元数之间的球面线性插值结果，输入无效时返回零四元数。
    static Quaternion slerp(const Quaternion& from, const Quaternion& to, double factor, double epsilon = DefaultEpsilon);

    /// 算术运算

    // 计算两个四元数的点积。
    static double dot(const Quaternion& first, const Quaternion& second);

    // 返回两个四元数的逐分量加法结果。
    Quaternion operator+(const Quaternion& other) const;

    // 返回两个四元数的逐分量减法结果。
    Quaternion operator-(const Quaternion& other) const;

    // 返回当前四元数的逐分量相反数。
    Quaternion operator-() const;

    // 返回两个四元数的Hamilton乘积，右侧四元数表示的旋转先应用。
    Quaternion operator*(const Quaternion& other) const;

    // 返回当前四元数与标量的乘积。
    Quaternion operator*(double scalar) const;

    // 返回当前四元数除以非零标量的结果。
    Quaternion operator/(double scalar) const;

    // 将另一个四元数逐分量加到当前四元数。
    Quaternion& operator+=(const Quaternion& other);

    // 将另一个四元数逐分量从当前四元数中减去。
    Quaternion& operator-=(const Quaternion& other);

    // 将当前四元数右乘另一个四元数。
    Quaternion& operator*=(const Quaternion& other);

    // 将当前四元数乘以标量。
    Quaternion& operator*=(double scalar);

    // 将当前四元数除以非零标量。
    Quaternion& operator/=(double scalar);

private:
    double m_w; // 实部分量。
    double m_x; // X虚部分量。
    double m_y; // Y虚部分量。
    double m_z; // Z虚部分量。
};

// 计算标量与四元数的乘积。
Quaternion operator*(double scalar, const Quaternion& quaternion);

}

#endif // MYMATH_QUATERNION_H