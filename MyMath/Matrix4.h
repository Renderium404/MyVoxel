#ifndef MYMATH_MATRIX4_H
#define MYMATH_MATRIX4_H

#include <array>

#include "Vector3.h"

namespace MyMath
{

// 4×4双精度矩阵，内部采用行优先存储。
class Matrix4
{
public:
    enum
    {
        Size = 4,
        ElementCount = Size * Size
    };

    static const double DefaultEpsilon;

public:
    // 构造零矩阵。
    Matrix4();

    // 使用行优先数组构造矩阵。
    explicit Matrix4(const std::array<double, ElementCount>& values);

    /// 矩阵创建

    // 创建零矩阵。
    static Matrix4 zero();

    // 创建单位矩阵。
    static Matrix4 identity();

    // 创建平移矩阵。
    static Matrix4 fromTranslation(const Vector3& translation);

    /// 元素访问

    // 获取指定行列的元素。
    double value(int row, int column) const;

    // 设置指定行列的元素。
    void setValue(int row, int column, double value);

    // 获取指定行列元素的可修改引用。
    double& operator()(int row, int column);

    // 获取指定行列元素的只读引用。
    const double& operator()(int row, int column) const;

    // 获取内部行优先数据。
    double* data();

    // 获取内部行优先只读数据。
    const double* data() const;

    /// 状态设置

    // 将矩阵设置为零矩阵。
    void setToZero();

    // 将矩阵设置为单位矩阵。
    void setToIdentity();

    // 设置矩阵第四列中的平移分量。
    void setTranslation(const Vector3& translation);

    /// 状态判断

    // 判断矩阵中所有元素是否为有限值。
    bool isFinite() const;

    // 判断矩阵是否近似为零矩阵。
    bool isZero(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否近似为单位矩阵。
    bool isIdentity(double epsilon = DefaultEpsilon) const;

    // 判断矩阵最后一行是否符合仿射变换形式。
    bool isAffine(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否为不包含平移的右手正交旋转矩阵。
    bool isRotationMatrix(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否为右手正交刚体变换矩阵。
    bool isRigidTransform(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否可逆。
    bool isInvertible(double epsilon = DefaultEpsilon) const;

    // 判断两个矩阵的对应元素是否近似相等。
    bool isEqualTo(const Matrix4& other, double epsilon = DefaultEpsilon) const;

    /// 变换分量

    // 获取矩阵第四列中的平移分量。
    Vector3 translation() const;

    /// 点和向量变换

    // 使用矩阵前三行变换点，包含平移分量。
    Vector3 transformPoint(const Vector3& point) const;

    // 使用矩阵左上角3×3部分变换向量，不包含平移分量。
    Vector3 transformVector(const Vector3& vector) const;

    /// 基础运算

    // 返回矩阵转置结果。
    Matrix4 transposed() const;

    // 计算矩阵行列式，矩阵包含非有限值时返回NaN。
    double determinant() const;

    // 计算矩阵逆矩阵，矩阵不可逆时保持result不变并返回false。
    bool inverted(Matrix4& result, double epsilon = DefaultEpsilon) const;

    // 将当前矩阵替换为逆矩阵，矩阵不可逆时保持不变并返回false。
    bool invert(double epsilon = DefaultEpsilon);

    /// 算术运算

    Matrix4 operator+(const Matrix4& other) const;
    Matrix4 operator-(const Matrix4& other) const;
    Matrix4 operator-() const;
    Matrix4 operator*(const Matrix4& other) const;
    Matrix4 operator*(double scalar) const;
    Matrix4 operator/(double scalar) const;

    Matrix4& operator+=(const Matrix4& other);
    Matrix4& operator-=(const Matrix4& other);
    Matrix4& operator*=(const Matrix4& other);
    Matrix4& operator*=(double scalar);
    Matrix4& operator/=(double scalar);

private:
    // 检查矩阵下标。
    static void checkIndex(int row, int column);

    // 将二维下标转换为行优先数组下标。
    static int index(int row, int column);

private:
    std::array<double, ElementCount> m_values; // 行优先矩阵数据。
};

// 计算标量与矩阵的乘积。
Matrix4 operator*(double scalar, const Matrix4& matrix);

}

#endif