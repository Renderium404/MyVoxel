#ifndef MYMATH_MATRIX4_H
#define MYMATH_MATRIX4_H

#include <array>
#include "Vector3.h"
namespace MyMath
{
class Matrix3;
// 4×4双精度矩阵，内部采用行优先存储，使用列向量乘法约定。
class Matrix4
{
public:
    enum
    {
        Size = 4,
        ElementCount = Size * Size
    };

    static const double DefaultEpsilon;
    // 构造零矩阵。
    Matrix4();
    // 使用行优先数组构造矩阵。
    explicit Matrix4(const std::array<double, ElementCount>& values);
    // 使用十六个元素按行优先顺序构造矩阵。
    Matrix4(double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33);

    /// 矩阵创建
    // 创建零矩阵。
    static Matrix4 zero();
    // 创建单位矩阵。
    static Matrix4 identity();
    /// 仿射矩阵创建
    // 创建三维平移矩阵，调用者必须保证translation为有限值。
    static Matrix4 fromTranslation(const Vector3& translation);
    // 创建三维统一缩放矩阵，调用者必须保证scale为有限值。
    static Matrix4 fromScale(double scale);
    // 创建三维非统一缩放矩阵，调用者必须保证scale各分量为有限值。
    static Matrix4 fromScale(const Vector3& scale);
    // 根据右手正交旋转矩阵创建三维旋转矩阵，调用者必须保证rotation有效。
    static Matrix4 fromRotation(const Matrix3& rotation, double epsilon = DefaultEpsilon);
    // 根据三阶线性变换和平移创建三维仿射矩阵，调用者必须保证输入为有限值。
    static Matrix4 fromAffine(const Matrix3& linearTransform, const Vector3& translation);

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

    /// 状态判断
    // 判断矩阵中所有元素是否为有限值。
    bool isFinite() const;
    // 判断矩阵是否近似为零矩阵。
    bool isZero(double epsilon = DefaultEpsilon) const;
    // 判断矩阵是否近似为单位矩阵。
    bool isIdentity(double epsilon = DefaultEpsilon) const;
    // 判断矩阵在指定误差下是否可逆。
    bool isInvertible(double epsilon = DefaultEpsilon) const;
    // 判断两个矩阵的对应元素是否近似相等。
    bool isEqualTo(const Matrix4& other, double epsilon = DefaultEpsilon) const;
    // 判断矩阵是否满足三维仿射齐次矩阵的最后一行约束。
    bool isAffine(double epsilon = DefaultEpsilon) const;
    
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

    /// 向量运算
    // 使用当前矩阵左乘由四个分量构成的四维列向量。
    std::array<double, Size> transformVector(double x, double y, double z, double w) const;
    // 使用当前仿射矩阵变换三维点，包含平移；调用者必须保证当前矩阵为仿射矩阵。
    Vector3 transformPoint(const Vector3& point) const;
    // 使用当前仿射矩阵变换三维向量，不包含平移；调用者必须保证当前矩阵为仿射矩阵。
    Vector3 transformVector(const Vector3& vector) const;

    /// 基础运算
    // 返回矩阵转置结果。
    Matrix4 transposed() const;

    // 计算矩阵逆矩阵，矩阵不可逆时保持result不变并返回false。
    bool inverted(Matrix4& result, double epsilon = DefaultEpsilon) const;
    // 将当前矩阵替换为逆矩阵，矩阵不可逆时保持不变并返回false。
    bool invert(double epsilon = DefaultEpsilon);

    /// 仿射矩阵属性
    // 返回三维仿射矩阵的平移分量，调用者必须保证当前矩阵为仿射矩阵。
    Vector3 translation() const;
    
    /// 算术运算
    // 返回两个矩阵的逐元素加法结果。
    Matrix4 operator+(const Matrix4& other) const;
    // 返回两个矩阵的逐元素减法结果。
    Matrix4 operator-(const Matrix4& other) const;
    // 返回当前矩阵的逐元素相反数。
    Matrix4 operator-() const;
    // 返回当前矩阵与另一个矩阵的乘积。
    Matrix4 operator*(const Matrix4& other) const;
    // 返回当前矩阵与标量的乘积。
    Matrix4 operator*(double scalar) const;
    // 返回当前矩阵除以非零标量的结果。
    Matrix4 operator/(double scalar) const;
    // 将另一个矩阵逐元素加到当前矩阵。
    Matrix4& operator+=(const Matrix4& other);
    // 将另一个矩阵逐元素从当前矩阵中减去。
    Matrix4& operator-=(const Matrix4& other);
    // 将当前矩阵右乘另一个矩阵。
    Matrix4& operator*=(const Matrix4& other);
    // 将当前矩阵乘以标量。
    Matrix4& operator*=(double scalar);
    // 将当前矩阵除以非零标量。
    Matrix4& operator/=(double scalar);

private:
    // 检查矩阵行列下标。
    static void checkIndex(int row, int column);
    // 将二维下标转换为行优先数组下标。
    static int index(int row, int column);

private:
    std::array<double, ElementCount> m_values; // 行优先矩阵数据。
};

// 计算标量与矩阵的乘积。
Matrix4 operator*(double scalar, const Matrix4& matrix);

}

#endif // MYMATH_MATRIX4_H