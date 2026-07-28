#ifndef MYMATH_QUATERNION_H
#define MYMATH_QUATERNION_H

#include "Matrix4.h"
#include "Vector3.h"

namespace MyMath
{

// 表示三维旋转的双精度四元数，分量顺序为w、x、y、z。
class Quaternion
{
public:
    static const double DefaultEpsilon;

public:
    // 构造单位四元数。
    Quaternion();

    // 使用实部和三个虚部分量构造四元数。
    Quaternion(double w, double x, double y, double z);

    /// 四元数创建

    // 创建零四元数。
    static Quaternion zero();

    // 创建单位四元数。
    static Quaternion identity();

    // 根据旋转轴和弧度角创建单位四元数，旋转轴无效时返回零四元数。
    static Quaternion fromAxisAngle(const Vector3& axis, double angle,
                                    double epsilon = DefaultEpsilon);

    // 根据纯右手正交旋转矩阵创建单位四元数，矩阵无效时返回零四元数。
    static Quaternion fromRotationMatrix(const Matrix4& matrix,
                                         double epsilon = DefaultEpsilon);
    /// 分量访问
    double w() const;
    double x() const;
    double y() const;
    double z() const;
    void setW(double w);
    void setX(double x);
    void setY(double y);
    void setZ(double z);
    void set(double w, double x, double y, double z);
    /// 状态判断

    // 判断所有分量是否为有限值。
    bool isFinite() const;
    // 判断四元数模长是否不大于指定误差。
    bool isZero(double epsilon = DefaultEpsilon) const;
    // 判断四元数是否表示单位旋转。
    bool isIdentity(double epsilon = DefaultEpsilon) const;
    // 判断四元数模长是否近似为1。
    bool isUnit(double epsilon = DefaultEpsilon) const;
    // 判断两个四元数的四维距离是否不大于指定误差。
    bool isEqualTo(const Quaternion& other, double epsilon = DefaultEpsilon) const;
    // 判断两个单位四元数是否表示相同旋转。
    bool isSameRotation(const Quaternion& other, double epsilon = DefaultEpsilon) const;
    
    /// 长度与归一化

    // 返回四元数模长平方。
    double lengthSquared() const;

    // 返回四元数模长。
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

    // 将单位四元数转换为纯旋转矩阵，当前四元数不是单位四元数时返回零矩阵。
    Matrix4 toRotationMatrix(double epsilon = DefaultEpsilon) const;

    // 使用单位四元数旋转三维数据，调用者必须保证当前四元数为单位四元数。
    Vector3 rotateVector(const Vector3& vector, double epsilon = DefaultEpsilon) const;

    /// 插值计算

    // 返回两个单位四元数之间的球面线性插值结果，输入无效时返回零四元数。
    static Quaternion slerp(const Quaternion& from, const Quaternion& to, double factor,
                            double epsilon = DefaultEpsilon);

    /// 四元数运算

    // 计算两个四元数的点积。
    static double dot(const Quaternion& first, const Quaternion& second);

    Quaternion operator+(const Quaternion& other) const;
    Quaternion operator-(const Quaternion& other) const;
    Quaternion operator-() const;

    // 组合两个旋转，右侧四元数先应用。
    Quaternion operator*(const Quaternion& other) const;

    Quaternion operator*(double scalar) const;
    Quaternion operator/(double scalar) const;

    Quaternion& operator+=(const Quaternion& other);
    Quaternion& operator-=(const Quaternion& other);
    Quaternion& operator*=(const Quaternion& other);
    Quaternion& operator*=(double scalar);
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

#endif