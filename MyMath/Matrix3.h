#ifndef MYMATH_MATRIX3_H
#define MYMATH_MATRIX3_H

#include <array>

#include "Vector3.h"

namespace MyMath
{

class Quaternion;
class CoordinateSystem;

// 3×3双精度矩阵，内部采用行优先存储，使用列向量乘法约定。
class Matrix3
{
    friend class Quaternion;
    friend class CoordinateSystem;

public:
    enum
    {
        Size = 3,
        ElementCount = Size * Size
    };

    static const double DefaultEpsilon;

    // 构造零矩阵。
    Matrix3();

    // 使用行优先数组构造矩阵。
    explicit Matrix3(const std::array<double, ElementCount>& values);

    // 使用九个元素按行优先顺序构造矩阵。
    Matrix3(double m00, double m01, double m02,
            double m10, double m11, double m12,
            double m20, double m21, double m22);

    /// 矩阵创建

    // 创建零矩阵。
    static Matrix3 zero();

    // 创建单位矩阵。
    static Matrix3 identity();

    // 使用三个行向量创建矩阵。
    static Matrix3 fromRows(const Vector3& first, const Vector3& second, const Vector3& third);

    // 使用三个列向量创建矩阵。
    static Matrix3 fromColumns(const Vector3& first, const Vector3& second, const Vector3& third);

    // 使用三个对角元素创建对角矩阵。
    static Matrix3 fromDiagonal(const Vector3& diagonal);

    // 创建first与second转置的外积矩阵，即first * second^T。
    static Matrix3 fromOuterProduct(const Vector3& first, const Vector3& second);

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

    /// 行列访问

    // 获取指定行向量。
    Vector3 row(int index) const;

    // 获取指定列向量。
    Vector3 column(int index) const;

    // 获取矩阵对角元素。
    Vector3 diagonal() const;

    // 设置指定行向量。
    void setRow(int index, const Vector3& value);

    // 设置指定列向量。
    void setColumn(int index, const Vector3& value);

    // 设置三个行向量。
    void setRows(const Vector3& first, const Vector3& second, const Vector3& third);

    // 设置三个列向量。
    void setColumns(const Vector3& first, const Vector3& second, const Vector3& third);

    // 设置矩阵对角元素，不改变非对角元素。
    void setDiagonal(const Vector3& diagonal);

    /// 状态设置

    // 将矩阵设置为零矩阵。
    void setToZero();

    // 将矩阵设置为单位矩阵。
    void setToIdentity();

    /// 状态判断

    // 判断矩阵中所有元素是否为有限值。
    bool isFinite() const;

    // 判断矩阵是否近似为零矩阵。
    bool isZero(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否近似为单位矩阵。
    bool isIdentity(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否近似为对角矩阵。
    bool isDiagonal(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否近似为对称矩阵。
    bool isSymmetric(double epsilon = DefaultEpsilon) const;

    // 判断矩阵的列向量是否构成正交单位基，允许旋转或反射。
    bool isOrthogonal(double epsilon = DefaultEpsilon) const;

    // 判断矩阵是否为右手正交旋转矩阵。
    bool isRotationMatrix(double epsilon = DefaultEpsilon) const;

    // 判断矩阵在指定误差下是否可逆。
    bool isInvertible(double epsilon = DefaultEpsilon) const;

    // 判断两个矩阵的对应元素是否近似相等。
    bool isEqualTo(const Matrix3& other, double epsilon = DefaultEpsilon) const;

    /// 矩阵特征

    // 返回矩阵迹，矩阵包含非有限值时返回NaN。
    double trace() const;

    // 返回所有元素绝对值的最大值，矩阵包含非有限值时返回NaN。
    double absoluteMaximum() const;

    // 返回矩阵1范数，即各列绝对值之和的最大值，矩阵包含非有限值时返回NaN。
    double oneNorm() const;

    // 返回矩阵无穷范数，即各行绝对值之和的最大值，矩阵包含非有限值时返回NaN。
    double infinityNorm() const;

    // 计算矩阵行列式，矩阵包含非有限值时返回NaN。
    double determinant() const;

    /// 向量变换

    // 使用当前矩阵左乘列向量。
    Vector3 transformVector(const Vector3& vector) const;

    /// 基础运算

    // 返回矩阵转置结果。
    Matrix3 transposed() const;

    // 计算矩阵逆矩阵，矩阵不可逆时保持result不变并返回false。
    bool inverted(Matrix3& result, double epsilon = DefaultEpsilon) const;

    // 将当前矩阵替换为逆矩阵，矩阵不可逆时保持不变并返回false。
    bool invert(double epsilon = DefaultEpsilon);

    /// 算术运算

    // 返回两个矩阵的逐元素加法结果。
    Matrix3 operator+(const Matrix3& other) const;

    // 返回两个矩阵的逐元素减法结果。
    Matrix3 operator-(const Matrix3& other) const;

    // 返回当前矩阵的逐元素相反数。
    Matrix3 operator-() const;

    // 返回当前矩阵与另一个矩阵的乘积。
    Matrix3 operator*(const Matrix3& other) const;

    // 返回当前矩阵与标量的乘积。
    Matrix3 operator*(double scalar) const;

    // 返回当前矩阵除以非零标量的结果。
    Matrix3 operator/(double scalar) const;

    // 将另一个矩阵逐元素加到当前矩阵。
    Matrix3& operator+=(const Matrix3& other);

    // 将另一个矩阵逐元素从当前矩阵中减去。
    Matrix3& operator-=(const Matrix3& other);

    // 将当前矩阵右乘另一个矩阵。
    Matrix3& operator*=(const Matrix3& other);

    // 将当前矩阵乘以标量。
    Matrix3& operator*=(double scalar);

    // 将当前矩阵除以非零标量。
    Matrix3& operator/=(double scalar);

private:
    // 检查矩阵行列下标。
    static void checkIndex(int row, int column);

    // 检查行或列下标。
    static void checkVectorIndex(int index);

    // 将二维下标转换为行优先数组下标。
    static int index(int row, int column);

private:
    std::array<double, ElementCount> m_values; // 行优先矩阵数据。
};

// 计算标量与矩阵的乘积。
Matrix3 operator*(double scalar, const Matrix3& matrix);

}

#endif // MYMATH_MATRIX3_H