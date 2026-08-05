#ifndef MYMATH_VECTOR3_H
#define MYMATH_VECTOR3_H

namespace MyMath
{

class Matrix3;
class Quaternion;
class CoordinateSystem;

// 表示三维点或三维向量的双精度数据。
class Vector3
{
    friend class Matrix3;
    friend class Matrix4;
    friend class Quaternion;
    friend class CoordinateSystem;

public:
    static const double DefaultEpsilon;

    // 构造零数据。
    Vector3();

    // 使用三个分量构造三维数据。
    Vector3(double x, double y, double z);

    /// 数据创建

    // 创建零数据。
    static Vector3 zero();

    // 创建X轴单位向量。
    static Vector3 unitX();

    // 创建Y轴单位向量。
    static Vector3 unitY();

    // 创建Z轴单位向量。
    static Vector3 unitZ();

    /// 分量访问

    // 返回X分量。
    double x() const;

    // 返回Y分量。
    double y() const;

    // 返回Z分量。
    double z() const;

    // 设置X分量。
    void setX(double x);

    // 设置Y分量。
    void setY(double y);

    // 设置Z分量。
    void setZ(double z);

    // 同时设置三个分量。
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

    // 返回向量长度平方，结果可能因数值范围而溢出。
    double lengthSquared() const;

    // 返回使用缩放计算的向量长度。
    double length() const;

    // 返回当前数据与目标数据之间的距离平方，结果可能因数值范围而溢出。
    double distanceSquaredTo(const Vector3& other) const;

    // 返回当前数据与目标数据之间使用缩放计算的距离。
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

    // 返回两个三维数据的逐分量加法结果。
    Vector3 operator+(const Vector3& other) const;

    // 返回两个三维数据的逐分量减法结果。
    Vector3 operator-(const Vector3& other) const;

    // 返回当前三维数据的相反数。
    Vector3 operator-() const;

    // 返回当前三维数据与标量的乘积。
    Vector3 operator*(double scalar) const;

    // 返回当前三维数据除以非零标量的结果。
    Vector3 operator/(double scalar) const;

    // 将另一个三维数据逐分量加到当前数据。
    Vector3& operator+=(const Vector3& other);

    // 将另一个三维数据逐分量从当前数据中减去。
    Vector3& operator-=(const Vector3& other);

    // 将当前三维数据乘以标量。
    Vector3& operator*=(double scalar);

    // 将当前三维数据除以非零标量。
    Vector3& operator/=(double scalar);

private:
    double m_x; // X分量。
    double m_y; // Y分量。
    double m_z; // Z分量。
};

// 计算标量与三维数据的乘积。
Vector3 operator*(double scalar, const Vector3& vector);

}

#endif // MYMATH_VECTOR3_H