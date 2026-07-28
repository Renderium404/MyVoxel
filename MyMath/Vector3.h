#ifndef MYMATH_VECTOR3_H
#define MYMATH_VECTOR3_H

namespace MyMath
{

// 表示三维点或三维向量的双精度数据。
class Vector3
{
public:
    static const double DefaultEpsilon;

public:
    // 构造零数据。
    Vector3();

    // 使用三个分量构造三维数据。
    Vector3(double x, double y, double z);

    /// 常用数据

    // 创建零数据。
    static Vector3 zero();

    // 创建X轴单位向量。
    static Vector3 unitX();

    // 创建Y轴单位向量。
    static Vector3 unitY();

    // 创建Z轴单位向量。
    static Vector3 unitZ();

    /// 分量访问

    double x() const;
    double y() const;
    double z() const;

    void setX(double x);
    void setY(double y);
    void setZ(double z);
    void set(double x, double y, double z);

    /// 状态判断

    // 判断所有分量是否为有限值。
    bool isFinite() const;

    // 判断当前数据是否能够作为非零向量参与向量运算。
    bool isVector(double epsilon = DefaultEpsilon) const;

    // 判断当前数据是否近似为零。
    bool isZero(double epsilon = DefaultEpsilon) const;

    // 判断当前数据是否近似为单位向量。
    bool isUnit(double epsilon = DefaultEpsilon) const;

    // 判断两个三维数据之间的距离是否不大于指定误差。
    bool isEqualTo(const Vector3& other, double epsilon = DefaultEpsilon) const;

    /// 长度与距离

    // 返回向量长度平方。
    double lengthSquared() const;

    // 返回向量长度。
    double length() const;

    // 返回当前数据与目标数据之间的距离平方。
    double distanceSquaredTo(const Vector3& other) const;

    // 返回当前数据与目标数据之间的距离。
    double distanceTo(const Vector3& other) const;

    /// 向量计算

    // 返回单位向量，当前数据不能作为向量时返回零向量。
    Vector3 normalized(double epsilon = DefaultEpsilon) const;

    // 将当前数据归一化，当前数据不能作为向量时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);

    // 计算两个向量的点积。
    static double dot(const Vector3& first, const Vector3& second);

    // 计算两个向量的叉积。
    static Vector3 cross(const Vector3& first, const Vector3& second);

    /// 算术运算

    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator-() const;

    Vector3 operator*(double scalar) const;
    Vector3 operator/(double scalar) const;

    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(double scalar);
    Vector3& operator/=(double scalar);

private:
    double m_x; // X分量。
    double m_y; // Y分量。
    double m_z; // Z分量。
};

// 计算标量与向量的乘积。
Vector3 operator*(double scalar, const Vector3& vector);

}

#endif