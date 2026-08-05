#include "Matrix4.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include "Matrix3.h"
#include "MathUtils.h"


namespace MyMath
{

const double Matrix4::DefaultEpsilon = 1.0e-12; // 默认浮点比较和奇异性判断误差。

Matrix4::Matrix4()
{
    setToZero();
}

Matrix4::Matrix4(const std::array<double, ElementCount>& values)
    : m_values(values)
{
}

Matrix4::Matrix4(double m00, double m01, double m02, double m03,
                 double m10, double m11, double m12, double m13,
                 double m20, double m21, double m22, double m23,
                 double m30, double m31, double m32, double m33)
{
    m_values[0] = m00;
    m_values[1] = m01;
    m_values[2] = m02;
    m_values[3] = m03;

    m_values[4] = m10;
    m_values[5] = m11;
    m_values[6] = m12;
    m_values[7] = m13;

    m_values[8] = m20;
    m_values[9] = m21;
    m_values[10] = m22;
    m_values[11] = m23;

    m_values[12] = m30;
    m_values[13] = m31;
    m_values[14] = m32;
    m_values[15] = m33;
}

/// 矩阵创建

Matrix4 Matrix4::zero()
{
    return Matrix4();
}

Matrix4 Matrix4::identity()
{
    Matrix4 matrix;
    matrix.setToIdentity();
    return matrix;
}
/// 仿射矩阵创建

Matrix4 Matrix4::fromTranslation(const Vector3& translation)
{
    assert(translation.isFinite());

    return fromAffine(Matrix3::identity(), translation);
}

Matrix4 Matrix4::fromScale(double scale)
{
    assert(MyMath::isFinite(scale));

    return fromScale(Vector3(scale, scale, scale));
}

Matrix4 Matrix4::fromScale(const Vector3& scale)
{
    assert(scale.isFinite());

    return Matrix4(
        scale.x(), 0.0, 0.0, 0.0,
        0.0, scale.y(), 0.0, 0.0,
        0.0, 0.0, scale.z(), 0.0,
        0.0, 0.0, 0.0, 1.0);
}

Matrix4 Matrix4::fromRotation(const Matrix3& rotation, double epsilon)
{
    assert(rotation.isRotationMatrix(epsilon));

    return fromAffine(rotation, Vector3::zero());
}

Matrix4 Matrix4::fromAffine(const Matrix3& linearTransform, const Vector3& translation)
{
    assert(linearTransform.isFinite());
    assert(translation.isFinite());

    return Matrix4(
        linearTransform(0, 0), linearTransform(0, 1), linearTransform(0, 2), translation.x(),
        linearTransform(1, 0), linearTransform(1, 1), linearTransform(1, 2), translation.y(),
        linearTransform(2, 0), linearTransform(2, 1), linearTransform(2, 2), translation.z(),
        0.0, 0.0, 0.0, 1.0);
}
/// 元素访问

double Matrix4::value(int row, int column) const
{
    return m_values[index(row, column)];
}

void Matrix4::setValue(int row, int column, double value)
{
    m_values[index(row, column)] = value;
}

double& Matrix4::operator()(int row, int column)
{
    return m_values[index(row, column)];
}

const double& Matrix4::operator()(int row, int column) const
{
    return m_values[index(row, column)];
}

double* Matrix4::data()
{
    return m_values.data();
}

const double* Matrix4::data() const
{
    return m_values.data();
}

/// 状态设置

void Matrix4::setToZero()
{
    m_values.fill(0.0);
}

void Matrix4::setToIdentity()
{
    setToZero();

    m_values[0] = 1.0;
    m_values[5] = 1.0;
    m_values[10] = 1.0;
    m_values[15] = 1.0;
}

/// 状态判断

bool Matrix4::isFinite() const
{
    for (int i = 0; i < ElementCount; ++i)
    {
        if (!MyMath::isFinite(m_values[i]))
        {
            return false;
        }
    }

    return true;
}

bool Matrix4::isZero(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    for (int i = 0; i < ElementCount; ++i)
    {
        if (std::fabs(m_values[i]) > epsilon)
        {
            return false;
        }
    }

    return true;
}

bool Matrix4::isIdentity(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            const double expectedValue = row == column ? 1.0 : 0.0;

            if (std::fabs(value(row, column) - expectedValue) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

bool Matrix4::isInvertible(double epsilon) const
{
    Matrix4 result;
    return inverted(result, epsilon);
}

bool Matrix4::isEqualTo(const Matrix4& other, double epsilon) const
{
    if (!isFinite() || !other.isFinite())
    {
        return false;
    }

    for (int i = 0; i < ElementCount; ++i)
    {
        if (std::fabs(m_values[i] - other.m_values[i]) > epsilon)
        {
            return false;
        }
    }

    return true;
}
bool Matrix4::isAffine(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    return std::fabs(m_values[12]) <= epsilon &&
           std::fabs(m_values[13]) <= epsilon &&
           std::fabs(m_values[14]) <= epsilon &&
           std::fabs(m_values[15] - 1.0) <= epsilon;
}
/// 矩阵特征

double Matrix4::trace() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    return m_values[0] + m_values[5] + m_values[10] + m_values[15];
}

double Matrix4::absoluteMaximum() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    double result = 0.0;

    for (int i = 0; i < ElementCount; ++i)
    {
        result = (std::max)(result, std::fabs(m_values[i]));
    }

    return result;
}

double Matrix4::oneNorm() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    double result = 0.0;

    for (int column = 0; column < Size; ++column)
    {
        double sum = 0.0;

        for (int row = 0; row < Size; ++row)
        {
            sum += std::fabs(value(row, column));
        }

        result = (std::max)(result, sum);
    }

    return result;
}

double Matrix4::infinityNorm() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    double result = 0.0;

    for (int row = 0; row < Size; ++row)
    {
        double sum = 0.0;

        for (int column = 0; column < Size; ++column)
        {
            sum += std::fabs(value(row, column));
        }

        result = (std::max)(result, sum);
    }

    return result;
}

double Matrix4::determinant() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    const double scale = absoluteMaximum();

    if (scale == 0.0)
    {
        return 0.0;
    }

    double values[Size][Size];

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            values[row][column] = value(row, column) / scale;
        }
    }

    double normalizedDeterminant = 1.0;
    int sign = 1;

    for (int column = 0; column < Size; ++column)
    {
        int pivotRow = column;
        double pivotAbsolute = std::fabs(values[column][column]);

        for (int row = column + 1; row < Size; ++row)
        {
            const double currentAbsolute = std::fabs(values[row][column]);

            if (currentAbsolute > pivotAbsolute)
            {
                pivotAbsolute = currentAbsolute;
                pivotRow = row;
            }
        }

        if (pivotAbsolute == 0.0)
        {
            return 0.0;
        }

        if (pivotRow != column)
        {
            for (int currentColumn = 0; currentColumn < Size; ++currentColumn)
            {
                std::swap(values[column][currentColumn], values[pivotRow][currentColumn]);
            }

            sign = -sign;
        }

        const double pivotValue = values[column][column];
        normalizedDeterminant *= pivotValue;

        for (int row = column + 1; row < Size; ++row)
        {
            const double factor = values[row][column] / pivotValue;

            for (int currentColumn = column + 1; currentColumn < Size; ++currentColumn)
            {
                values[row][currentColumn] -= factor * values[column][currentColumn];
            }
        }
    }

    const double scaleSquared = scale * scale;

    return static_cast<double>(sign) * normalizedDeterminant * scaleSquared * scaleSquared;
}

/// 向量运算

std::array<double, Matrix4::Size> Matrix4::transformVector(double x, double y, double z, double w) const
{
    std::array<double, Size> result;

    result[0] = m_values[0] * x + m_values[1] * y + m_values[2] * z + m_values[3] * w;
    result[1] = m_values[4] * x + m_values[5] * y + m_values[6] * z + m_values[7] * w;
    result[2] = m_values[8] * x + m_values[9] * y + m_values[10] * z + m_values[11] * w;
    result[3] = m_values[12] * x + m_values[13] * y + m_values[14] * z + m_values[15] * w;

    return result;
}
Vector3 Matrix4::transformPoint(const Vector3& point) const
{
    return Vector3(
        m_values[0] * point.m_x + m_values[1] * point.m_y + m_values[2] * point.m_z + m_values[3],
        m_values[4] * point.m_x + m_values[5] * point.m_y + m_values[6] * point.m_z + m_values[7],
        m_values[8] * point.m_x + m_values[9] * point.m_y + m_values[10] * point.m_z + m_values[11]);
}

Vector3 Matrix4::transformVector(const Vector3& vector) const
{
    return Vector3(
        m_values[0] * vector.m_x + m_values[1] * vector.m_y + m_values[2] * vector.m_z,
        m_values[4] * vector.m_x + m_values[5] * vector.m_y + m_values[6] * vector.m_z,
        m_values[8] * vector.m_x + m_values[9] * vector.m_y + m_values[10] * vector.m_z);
}
/// 基础运算

Matrix4 Matrix4::transposed() const
{
    return Matrix4(m_values[0], m_values[4], m_values[8], m_values[12],
                   m_values[1], m_values[5], m_values[9], m_values[13],
                   m_values[2], m_values[6], m_values[10], m_values[14],
                   m_values[3], m_values[7], m_values[11], m_values[15]);
}

bool Matrix4::inverted(Matrix4& result, double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    const double scale = absoluteMaximum();

    if (scale == 0.0)
    {
        return false;
    }

    double augmented[Size][Size * 2];

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            augmented[row][column] = value(row, column) / scale;
            augmented[row][column + Size] = row == column ? 1.0 : 0.0;
        }
    }

    for (int column = 0; column < Size; ++column)
    {
        int pivotRow = column;
        double pivotAbsolute = std::fabs(augmented[column][column]);

        for (int row = column + 1; row < Size; ++row)
        {
            const double currentAbsolute = std::fabs(augmented[row][column]);

            if (currentAbsolute > pivotAbsolute)
            {
                pivotAbsolute = currentAbsolute;
                pivotRow = row;
            }
        }

        if (pivotAbsolute <= epsilon)
        {
            return false;
        }

        if (pivotRow != column)
        {
            for (int currentColumn = 0; currentColumn < Size * 2; ++currentColumn)
            {
                std::swap(augmented[column][currentColumn], augmented[pivotRow][currentColumn]);
            }
        }

        const double pivotValue = augmented[column][column];

        for (int currentColumn = 0; currentColumn < Size * 2; ++currentColumn)
        {
            augmented[column][currentColumn] /= pivotValue;
        }

        for (int row = 0; row < Size; ++row)
        {
            if (row == column)
            {
                continue;
            }

            const double factor = augmented[row][column];

            for (int currentColumn = 0; currentColumn < Size * 2; ++currentColumn)
            {
                augmented[row][currentColumn] -= factor * augmented[column][currentColumn];
            }
        }
    }

    Matrix4 invertedMatrix;

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            invertedMatrix(row, column) = augmented[row][column + Size] / scale;
        }
    }

    if (!invertedMatrix.isFinite())
    {
        return false;
    }

    result = invertedMatrix;

    return true;
}

bool Matrix4::invert(double epsilon)
{
    Matrix4 invertedMatrix;

    if (!inverted(invertedMatrix, epsilon))
    {
        return false;
    }

    *this = invertedMatrix;

    return true;
}
/// 仿射矩阵属性

Vector3 Matrix4::translation() const
{
    assert(isAffine());

    return Vector3(
        m_values[3],
        m_values[7],
        m_values[11]);
}
/// 算术运算

Matrix4 Matrix4::operator+(const Matrix4& other) const
{
    Matrix4 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] + other.m_values[i];
    }

    return result;
}

Matrix4 Matrix4::operator-(const Matrix4& other) const
{
    Matrix4 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] - other.m_values[i];
    }

    return result;
}

Matrix4 Matrix4::operator-() const
{
    Matrix4 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = -m_values[i];
    }

    return result;
}

Matrix4 Matrix4::operator*(const Matrix4& other) const
{
    Matrix4 result;

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            double sum = 0.0;

            for (int i = 0; i < Size; ++i)
            {
                sum += value(row, i) * other.value(i, column);
            }

            result(row, column) = sum;
        }
    }

    return result;
}

Matrix4 Matrix4::operator*(double scalar) const
{
    Matrix4 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] * scalar;
    }

    return result;
}

Matrix4 Matrix4::operator/(double scalar) const
{
    assert(scalar != 0.0);

    Matrix4 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] / scalar;
    }

    return result;
}

Matrix4& Matrix4::operator+=(const Matrix4& other)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] += other.m_values[i];
    }

    return *this;
}

Matrix4& Matrix4::operator-=(const Matrix4& other)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] -= other.m_values[i];
    }

    return *this;
}

Matrix4& Matrix4::operator*=(const Matrix4& other)
{
    *this = *this * other;

    return *this;
}

Matrix4& Matrix4::operator*=(double scalar)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] *= scalar;
    }

    return *this;
}

Matrix4& Matrix4::operator/=(double scalar)
{
    assert(scalar != 0.0);

    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] /= scalar;
    }

    return *this;
}

/// 内部辅助

void Matrix4::checkIndex(int row, int column)
{
    assert(row >= 0 && row < Size);
    assert(column >= 0 && column < Size);
}

int Matrix4::index(int row, int column)
{
    checkIndex(row, column);

    return row * Size + column;
}

Matrix4 operator*(double scalar, const Matrix4& matrix)
{
    return matrix * scalar;
}

}