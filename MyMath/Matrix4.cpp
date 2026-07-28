#include "Matrix4.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace
{

// 判断浮点数是否为有限数值，不包含NaN和正负无穷。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();

    return value == value && value != infinity && value != -infinity;
}

}

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

Matrix4 Matrix4::fromTranslation(const Vector3& translation)
{
    Matrix4 matrix = Matrix4::identity();
    matrix.setTranslation(translation);
    return matrix;
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

    for (int i = 0; i < Size; ++i)
    {
        m_values[i * Size + i] = 1.0;
    }
}

void Matrix4::setTranslation(const Vector3& translation)
{
    m_values[index(0, 3)] = translation.x();
    m_values[index(1, 3)] = translation.y();
    m_values[index(2, 3)] = translation.z();
}

/// 状态判断

bool Matrix4::isFinite() const
{
    for (int i = 0; i < ElementCount; ++i)
    {
        if (!isFiniteValue(m_values[i]))
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

bool Matrix4::isAffine(double epsilon) const
{
    return isFinite() &&
           std::fabs(value(3, 0)) <= epsilon &&
           std::fabs(value(3, 1)) <= epsilon &&
           std::fabs(value(3, 2)) <= epsilon &&
           std::fabs(value(3, 3) - 1.0) <= epsilon;
}

bool Matrix4::isRotationMatrix(double epsilon) const
{
    if (!isRigidTransform(epsilon))
    {
        return false;
    }

    return std::fabs(value(0, 3)) <= epsilon &&
           std::fabs(value(1, 3)) <= epsilon &&
           std::fabs(value(2, 3)) <= epsilon;
}

bool Matrix4::isRigidTransform(double epsilon) const
{
    if (!isAffine(epsilon))
    {
        return false;
    }

    const Vector3 xAxis(value(0, 0), value(1, 0), value(2, 0));
    const Vector3 yAxis(value(0, 1), value(1, 1), value(2, 1));
    const Vector3 zAxis(value(0, 2), value(1, 2), value(2, 2));

    if (!xAxis.isUnit(epsilon) || !yAxis.isUnit(epsilon) || !zAxis.isUnit(epsilon))
    {
        return false;
    }

    if (std::fabs(Vector3::dot(xAxis, yAxis)) > epsilon ||
        std::fabs(Vector3::dot(xAxis, zAxis)) > epsilon ||
        std::fabs(Vector3::dot(yAxis, zAxis)) > epsilon)
    {
        return false;
    }

    const double handedness = Vector3::dot(Vector3::cross(xAxis, yAxis), zAxis);

    return std::fabs(handedness - 1.0) <= epsilon;
}

bool Matrix4::isInvertible(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    return std::fabs(determinant()) > epsilon;
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

/// 变换分量

Vector3 Matrix4::translation() const
{
    return Vector3(value(0, 3), value(1, 3), value(2, 3));
}

/// 点和向量变换

Vector3 Matrix4::transformPoint(const Vector3& point) const
{
    return Vector3(value(0, 0) * point.x() + value(0, 1) * point.y() + value(0, 2) * point.z() + value(0, 3),
                   value(1, 0) * point.x() + value(1, 1) * point.y() + value(1, 2) * point.z() + value(1, 3),
                   value(2, 0) * point.x() + value(2, 1) * point.y() + value(2, 2) * point.z() + value(2, 3));
}

Vector3 Matrix4::transformVector(const Vector3& vector) const
{
    return Vector3(value(0, 0) * vector.x() + value(0, 1) * vector.y() + value(0, 2) * vector.z(),
                   value(1, 0) * vector.x() + value(1, 1) * vector.y() + value(1, 2) * vector.z(),
                   value(2, 0) * vector.x() + value(2, 1) * vector.y() + value(2, 2) * vector.z());
}

/// 基础运算

Matrix4 Matrix4::transposed() const
{
    Matrix4 result;

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            result(row, column) = value(column, row);
        }
    }

    return result;
}

double Matrix4::determinant() const
{
    if (!isFinite())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double values[Size][Size];

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            values[row][column] = value(row, column);
        }
    }

    double result = 1.0;
    int sign = 1;

    for (int column = 0; column < Size; ++column)
    {
        int pivotRow = column;
        double pivotAbs = std::fabs(values[column][column]);

        for (int row = column + 1; row < Size; ++row)
        {
            const double currentAbs = std::fabs(values[row][column]);

            if (currentAbs > pivotAbs)
            {
                pivotAbs = currentAbs;
                pivotRow = row;
            }
        }

        if (pivotAbs == 0.0)
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
        result *= pivotValue;

        for (int row = column + 1; row < Size; ++row)
        {
            const double factor = values[row][column] / pivotValue;

            for (int currentColumn = column + 1; currentColumn < Size; ++currentColumn)
            {
                values[row][currentColumn] -= factor * values[column][currentColumn];
            }
        }
    }

    return sign * result;
}

bool Matrix4::inverted(Matrix4& result, double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    double augmented[Size][Size * 2];

    for (int row = 0; row < Size; ++row)
    {
        for (int column = 0; column < Size; ++column)
        {
            augmented[row][column] = value(row, column);
            augmented[row][column + Size] = row == column ? 1.0 : 0.0;
        }
    }

    for (int column = 0; column < Size; ++column)
    {
        int pivotRow = column;
        double pivotAbs = std::fabs(augmented[column][column]);

        for (int row = column + 1; row < Size; ++row)
        {
            const double currentAbs = std::fabs(augmented[row][column]);

            if (currentAbs > pivotAbs)
            {
                pivotAbs = currentAbs;
                pivotRow = row;
            }
        }

        if (pivotAbs <= epsilon)
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
            invertedMatrix(row, column) = augmented[row][column + Size];
        }
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