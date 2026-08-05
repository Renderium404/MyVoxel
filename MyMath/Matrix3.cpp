#include "Matrix3.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "MathUtils.h"

namespace MyMath
{

const double Matrix3::DefaultEpsilon = 1.0e-12; // 默认浮点比较和奇异性判断误差。

Matrix3::Matrix3()
{
    setToZero();
}

Matrix3::Matrix3(const std::array<double, ElementCount>& values)
    : m_values(values)
{
}

Matrix3::Matrix3(double m00, double m01, double m02,
                 double m10, double m11, double m12,
                 double m20, double m21, double m22)
{
    m_values[0] = m00;
    m_values[1] = m01;
    m_values[2] = m02;
    m_values[3] = m10;
    m_values[4] = m11;
    m_values[5] = m12;
    m_values[6] = m20;
    m_values[7] = m21;
    m_values[8] = m22;
}

/// 矩阵创建

Matrix3 Matrix3::zero()
{
    return Matrix3();
}

Matrix3 Matrix3::identity()
{
    Matrix3 matrix;
    matrix.setToIdentity();
    return matrix;
}

Matrix3 Matrix3::fromRows(const Vector3& first, const Vector3& second, const Vector3& third)
{
    return Matrix3(first.m_x, first.m_y, first.m_z,
                   second.m_x, second.m_y, second.m_z,
                   third.m_x, third.m_y, third.m_z);
}

Matrix3 Matrix3::fromColumns(const Vector3& first, const Vector3& second, const Vector3& third)
{
    return Matrix3(first.m_x, second.m_x, third.m_x,
                   first.m_y, second.m_y, third.m_y,
                   first.m_z, second.m_z, third.m_z);
}

Matrix3 Matrix3::fromDiagonal(const Vector3& diagonal)
{
    return Matrix3(diagonal.m_x, 0.0, 0.0,
                   0.0, diagonal.m_y, 0.0,
                   0.0, 0.0, diagonal.m_z);
}

Matrix3 Matrix3::fromOuterProduct(const Vector3& first, const Vector3& second)
{
    return Matrix3(first.m_x * second.m_x, first.m_x * second.m_y, first.m_x * second.m_z,
                   first.m_y * second.m_x, first.m_y * second.m_y, first.m_y * second.m_z,
                   first.m_z * second.m_x, first.m_z * second.m_y, first.m_z * second.m_z);
}

/// 元素访问

double Matrix3::value(int row, int column) const
{
    return m_values[index(row, column)];
}

void Matrix3::setValue(int row, int column, double value)
{
    m_values[index(row, column)] = value;
}

double& Matrix3::operator()(int row, int column)
{
    return m_values[index(row, column)];
}

const double& Matrix3::operator()(int row, int column) const
{
    return m_values[index(row, column)];
}

double* Matrix3::data()
{
    return m_values.data();
}

const double* Matrix3::data() const
{
    return m_values.data();
}

/// 行列访问

Vector3 Matrix3::row(int indexValue) const
{
    checkVectorIndex(indexValue);

    const int offset = indexValue * Size;

    return Vector3(m_values[offset], m_values[offset + 1], m_values[offset + 2]);
}

Vector3 Matrix3::column(int indexValue) const
{
    checkVectorIndex(indexValue);

    return Vector3(m_values[indexValue],
                   m_values[Size + indexValue],
                   m_values[2 * Size + indexValue]);
}

Vector3 Matrix3::diagonal() const
{
    return Vector3(m_values[0], m_values[4], m_values[8]);
}

void Matrix3::setRow(int indexValue, const Vector3& value)
{
    checkVectorIndex(indexValue);

    const int offset = indexValue * Size;

    m_values[offset] = value.m_x;
    m_values[offset + 1] = value.m_y;
    m_values[offset + 2] = value.m_z;
}

void Matrix3::setColumn(int indexValue, const Vector3& value)
{
    checkVectorIndex(indexValue);

    m_values[indexValue] = value.m_x;
    m_values[Size + indexValue] = value.m_y;
    m_values[2 * Size + indexValue] = value.m_z;
}

void Matrix3::setRows(const Vector3& first, const Vector3& second, const Vector3& third)
{
    m_values[0] = first.m_x;
    m_values[1] = first.m_y;
    m_values[2] = first.m_z;

    m_values[3] = second.m_x;
    m_values[4] = second.m_y;
    m_values[5] = second.m_z;

    m_values[6] = third.m_x;
    m_values[7] = third.m_y;
    m_values[8] = third.m_z;
}

void Matrix3::setColumns(const Vector3& first, const Vector3& second, const Vector3& third)
{
    m_values[0] = first.m_x;
    m_values[1] = second.m_x;
    m_values[2] = third.m_x;

    m_values[3] = first.m_y;
    m_values[4] = second.m_y;
    m_values[5] = third.m_y;

    m_values[6] = first.m_z;
    m_values[7] = second.m_z;
    m_values[8] = third.m_z;
}

void Matrix3::setDiagonal(const Vector3& diagonal)
{
    m_values[0] = diagonal.m_x;
    m_values[4] = diagonal.m_y;
    m_values[8] = diagonal.m_z;
}

/// 状态设置

void Matrix3::setToZero()
{
    m_values.fill(0.0);
}

void Matrix3::setToIdentity()
{
    setToZero();
    m_values[0] = 1.0;
    m_values[4] = 1.0;
    m_values[8] = 1.0;
}

/// 状态判断

bool Matrix3::isFinite() const
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

bool Matrix3::isZero(double epsilon) const
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

bool Matrix3::isIdentity(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    for (int rowIndex = 0; rowIndex < Size; ++rowIndex)
    {
        for (int columnIndex = 0; columnIndex < Size; ++columnIndex)
        {
            const double expectedValue = rowIndex == columnIndex ? 1.0 : 0.0;

            if (std::fabs(value(rowIndex, columnIndex) - expectedValue) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

bool Matrix3::isDiagonal(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    for (int rowIndex = 0; rowIndex < Size; ++rowIndex)
    {
        for (int columnIndex = 0; columnIndex < Size; ++columnIndex)
        {
            if (rowIndex != columnIndex && std::fabs(value(rowIndex, columnIndex)) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

bool Matrix3::isSymmetric(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    return std::fabs(m_values[1] - m_values[3]) <= epsilon &&
           std::fabs(m_values[2] - m_values[6]) <= epsilon &&
           std::fabs(m_values[5] - m_values[7]) <= epsilon;
}

bool Matrix3::isOrthogonal(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    const double firstLength = norm(m_values[0], m_values[3], m_values[6]);
    const double secondLength = norm(m_values[1], m_values[4], m_values[7]);
    const double thirdLength = norm(m_values[2], m_values[5], m_values[8]);

    if (std::fabs(firstLength - 1.0) > epsilon ||
        std::fabs(secondLength - 1.0) > epsilon ||
        std::fabs(thirdLength - 1.0) > epsilon)
    {
        return false;
    }

    const double firstSecondDot = m_values[0] * m_values[1] + m_values[3] * m_values[4] + m_values[6] * m_values[7];
    const double firstThirdDot = m_values[0] * m_values[2] + m_values[3] * m_values[5] + m_values[6] * m_values[8];
    const double secondThirdDot = m_values[1] * m_values[2] + m_values[4] * m_values[5] + m_values[7] * m_values[8];

    return std::fabs(firstSecondDot) <= epsilon &&
           std::fabs(firstThirdDot) <= epsilon &&
           std::fabs(secondThirdDot) <= epsilon;
}

bool Matrix3::isRotationMatrix(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    const double firstLength = norm(m_values[0], m_values[3], m_values[6]);
    const double secondLength = norm(m_values[1], m_values[4], m_values[7]);
    const double thirdLength = norm(m_values[2], m_values[5], m_values[8]);

    if (std::fabs(firstLength - 1.0) > epsilon ||
        std::fabs(secondLength - 1.0) > epsilon ||
        std::fabs(thirdLength - 1.0) > epsilon)
    {
        return false;
    }

    const double firstSecondDot = m_values[0] * m_values[1] + m_values[3] * m_values[4] + m_values[6] * m_values[7];
    const double firstThirdDot = m_values[0] * m_values[2] + m_values[3] * m_values[5] + m_values[6] * m_values[8];
    const double secondThirdDot = m_values[1] * m_values[2] + m_values[4] * m_values[5] + m_values[7] * m_values[8];

    if (std::fabs(firstSecondDot) > epsilon ||
        std::fabs(firstThirdDot) > epsilon ||
        std::fabs(secondThirdDot) > epsilon)
    {
        return false;
    }

    const double crossX = m_values[3] * m_values[7] - m_values[6] * m_values[4];
    const double crossY = m_values[6] * m_values[1] - m_values[0] * m_values[7];
    const double crossZ = m_values[0] * m_values[4] - m_values[3] * m_values[1];
    const double handedness = crossX * m_values[2] + crossY * m_values[5] + crossZ * m_values[8];

    return std::fabs(handedness - 1.0) <= epsilon;
}

bool Matrix3::isInvertible(double epsilon) const
{
    Matrix3 result;
    return inverted(result, epsilon);
}

bool Matrix3::isEqualTo(const Matrix3& other, double epsilon) const
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

/// 矩阵特征

double Matrix3::trace() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    return m_values[0] + m_values[4] + m_values[8];
}

double Matrix3::absoluteMaximum() const
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

double Matrix3::oneNorm() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    const double firstColumn = std::fabs(m_values[0]) + std::fabs(m_values[3]) + std::fabs(m_values[6]);
    const double secondColumn = std::fabs(m_values[1]) + std::fabs(m_values[4]) + std::fabs(m_values[7]);
    const double thirdColumn = std::fabs(m_values[2]) + std::fabs(m_values[5]) + std::fabs(m_values[8]);

    return (std::max)(firstColumn, (std::max)(secondColumn, thirdColumn));
}

double Matrix3::infinityNorm() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    const double firstRow = std::fabs(m_values[0]) + std::fabs(m_values[1]) + std::fabs(m_values[2]);
    const double secondRow = std::fabs(m_values[3]) + std::fabs(m_values[4]) + std::fabs(m_values[5]);
    const double thirdRow = std::fabs(m_values[6]) + std::fabs(m_values[7]) + std::fabs(m_values[8]);

    return (std::max)(firstRow, (std::max)(secondRow, thirdRow));
}

double Matrix3::determinant() const
{
    if (!isFinite())
    {
        return quietNaN();
    }

    const double directDeterminant =
        m_values[0] * (m_values[4] * m_values[8] - m_values[5] * m_values[7]) -
        m_values[1] * (m_values[3] * m_values[8] - m_values[5] * m_values[6]) +
        m_values[2] * (m_values[3] * m_values[7] - m_values[4] * m_values[6]);

    if (MyMath::isFinite(directDeterminant))
    {
        return directDeterminant;
    }

    const double firstScale = maximumAbsolute(m_values[0], m_values[1], m_values[2]);
    const double secondScale = maximumAbsolute(m_values[3], m_values[4], m_values[5]);
    const double thirdScale = maximumAbsolute(m_values[6], m_values[7], m_values[8]);

    if (firstScale == 0.0 || secondScale == 0.0 || thirdScale == 0.0)
    {
        return 0.0;
    }

    const double m00 = m_values[0] / firstScale;
    const double m01 = m_values[1] / firstScale;
    const double m02 = m_values[2] / firstScale;
    const double m10 = m_values[3] / secondScale;
    const double m11 = m_values[4] / secondScale;
    const double m12 = m_values[5] / secondScale;
    const double m20 = m_values[6] / thirdScale;
    const double m21 = m_values[7] / thirdScale;
    const double m22 = m_values[8] / thirdScale;

    const double normalizedDeterminant =
        m00 * (m11 * m22 - m12 * m21) -
        m01 * (m10 * m22 - m12 * m20) +
        m02 * (m10 * m21 - m11 * m20);

    if (normalizedDeterminant == 0.0)
    {
        return 0.0;
    }

    int determinantExponent = 0;
    int firstExponent = 0;
    int secondExponent = 0;
    int thirdExponent = 0;

    const double determinantMantissa = std::frexp(normalizedDeterminant, &determinantExponent);
    const double firstMantissa = std::frexp(firstScale, &firstExponent);
    const double secondMantissa = std::frexp(secondScale, &secondExponent);
    const double thirdMantissa = std::frexp(thirdScale, &thirdExponent);
    const double combinedMantissa = determinantMantissa * firstMantissa * secondMantissa * thirdMantissa;

    return std::ldexp(combinedMantissa,
                      determinantExponent + firstExponent + secondExponent + thirdExponent);
}

/// 向量变换

Vector3 Matrix3::transformVector(const Vector3& vector) const
{
    return Vector3(m_values[0] * vector.m_x + m_values[1] * vector.m_y + m_values[2] * vector.m_z,
                   m_values[3] * vector.m_x + m_values[4] * vector.m_y + m_values[5] * vector.m_z,
                   m_values[6] * vector.m_x + m_values[7] * vector.m_y + m_values[8] * vector.m_z);
}

/// 基础运算

Matrix3 Matrix3::transposed() const
{
    return Matrix3(m_values[0], m_values[3], m_values[6],
                   m_values[1], m_values[4], m_values[7],
                   m_values[2], m_values[5], m_values[8]);
}

bool Matrix3::inverted(Matrix3& result, double epsilon) const
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

    const double m00 = m_values[0] / scale;
    const double m01 = m_values[1] / scale;
    const double m02 = m_values[2] / scale;
    const double m10 = m_values[3] / scale;
    const double m11 = m_values[4] / scale;
    const double m12 = m_values[5] / scale;
    const double m20 = m_values[6] / scale;
    const double m21 = m_values[7] / scale;
    const double m22 = m_values[8] / scale;

    const double determinantValue =
        m00 * (m11 * m22 - m12 * m21) -
        m01 * (m10 * m22 - m12 * m20) +
        m02 * (m10 * m21 - m11 * m20);

    if (std::fabs(determinantValue) <= epsilon)
    {
        return false;
    }

    const double factor = 1.0 / determinantValue / scale;

    Matrix3 invertedMatrix(
        (m11 * m22 - m12 * m21) * factor,
        (m02 * m21 - m01 * m22) * factor,
        (m01 * m12 - m02 * m11) * factor,
        (m12 * m20 - m10 * m22) * factor,
        (m00 * m22 - m02 * m20) * factor,
        (m02 * m10 - m00 * m12) * factor,
        (m10 * m21 - m11 * m20) * factor,
        (m01 * m20 - m00 * m21) * factor,
        (m00 * m11 - m01 * m10) * factor);

    if (!invertedMatrix.isFinite())
    {
        return false;
    }

    result = invertedMatrix;
    return true;
}

bool Matrix3::invert(double epsilon)
{
    Matrix3 invertedMatrix;

    if (!inverted(invertedMatrix, epsilon))
    {
        return false;
    }

    *this = invertedMatrix;
    return true;
}

/// 算术运算

Matrix3 Matrix3::operator+(const Matrix3& other) const
{
    Matrix3 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] + other.m_values[i];
    }

    return result;
}

Matrix3 Matrix3::operator-(const Matrix3& other) const
{
    Matrix3 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] - other.m_values[i];
    }

    return result;
}

Matrix3 Matrix3::operator-() const
{
    Matrix3 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = -m_values[i];
    }

    return result;
}

Matrix3 Matrix3::operator*(const Matrix3& other) const
{
    Matrix3 result;

    result.m_values[0] = m_values[0] * other.m_values[0] + m_values[1] * other.m_values[3] + m_values[2] * other.m_values[6];
    result.m_values[1] = m_values[0] * other.m_values[1] + m_values[1] * other.m_values[4] + m_values[2] * other.m_values[7];
    result.m_values[2] = m_values[0] * other.m_values[2] + m_values[1] * other.m_values[5] + m_values[2] * other.m_values[8];

    result.m_values[3] = m_values[3] * other.m_values[0] + m_values[4] * other.m_values[3] + m_values[5] * other.m_values[6];
    result.m_values[4] = m_values[3] * other.m_values[1] + m_values[4] * other.m_values[4] + m_values[5] * other.m_values[7];
    result.m_values[5] = m_values[3] * other.m_values[2] + m_values[4] * other.m_values[5] + m_values[5] * other.m_values[8];

    result.m_values[6] = m_values[6] * other.m_values[0] + m_values[7] * other.m_values[3] + m_values[8] * other.m_values[6];
    result.m_values[7] = m_values[6] * other.m_values[1] + m_values[7] * other.m_values[4] + m_values[8] * other.m_values[7];
    result.m_values[8] = m_values[6] * other.m_values[2] + m_values[7] * other.m_values[5] + m_values[8] * other.m_values[8];

    return result;
}

Matrix3 Matrix3::operator*(double scalar) const
{
    Matrix3 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] * scalar;
    }

    return result;
}

Matrix3 Matrix3::operator/(double scalar) const
{
    assert(scalar != 0.0);

    Matrix3 result;

    for (int i = 0; i < ElementCount; ++i)
    {
        result.m_values[i] = m_values[i] / scalar;
    }

    return result;
}

Matrix3& Matrix3::operator+=(const Matrix3& other)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] += other.m_values[i];
    }

    return *this;
}

Matrix3& Matrix3::operator-=(const Matrix3& other)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] -= other.m_values[i];
    }

    return *this;
}

Matrix3& Matrix3::operator*=(const Matrix3& other)
{
    *this = *this * other;
    return *this;
}

Matrix3& Matrix3::operator*=(double scalar)
{
    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] *= scalar;
    }

    return *this;
}

Matrix3& Matrix3::operator/=(double scalar)
{
    assert(scalar != 0.0);

    for (int i = 0; i < ElementCount; ++i)
    {
        m_values[i] /= scalar;
    }

    return *this;
}

/// 内部辅助

void Matrix3::checkIndex(int row, int column)
{
    assert(row >= 0 && row < Size);
    assert(column >= 0 && column < Size);
}

void Matrix3::checkVectorIndex(int indexValue)
{
    assert(indexValue >= 0 && indexValue < Size);
}

int Matrix3::index(int row, int column)
{
    checkIndex(row, column);
    return row * Size + column;
}

Matrix3 operator*(double scalar, const Matrix3& matrix)
{
    return matrix * scalar;
}

}