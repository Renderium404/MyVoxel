#include "FeaturePointSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 算法思路参考OpenVDB tools/VolumeToMesh.h中的findFeaturePoint。
// Copyright Contributors to the OpenVDB Project.
// SPDX-License-Identifier: Apache-2.0

const double EigenvalueToleranceScale = 0.01; // 使用最大特征值的1%截断不稳定的弱特征方向。
const double JacobiConvergenceTolerance = 1.0e-12; // 对称矩阵非对角元素收敛阈值。
const int MaximumJacobiIterationCount = 32; // 三阶对称矩阵最多执行32次Jacobi旋转。

// 保存三阶实矩阵。
struct Matrix3
{
    Matrix3()
    {
        clear();
    }

    void clear()
    {
        for (unsigned int row = 0; row < 3; ++row)
        {
            for (unsigned int column = 0; column < 3; ++column)
            {
                values[row][column] = 0.0;
            }
        }
    }

    void setIdentity()
    {
        clear();
        values[0][0] = 1.0;
        values[1][1] = 1.0;
        values[2][2] = 1.0;
    }

    double values[3][3];
};

// 返回矩阵指定列对应的向量。
MyMath::Vector3 matrixColumn(const Matrix3& matrix, unsigned int column)
{
    return MyMath::Vector3(
        matrix.values[0][column],
        matrix.values[1][column],
        matrix.values[2][column]);
}

// 返回三个数值绝对值中的最大值。
double maximumAbsoluteValue(double value0, double value1, double value2)
{
    return (std::max)(
        std::fabs(value0),
        (std::max)(std::fabs(value1), std::fabs(value2)));
}

// 对三阶实对称矩阵执行Jacobi特征分解。
void diagonalizeSymmetricMatrix(const Matrix3& input,
                                Matrix3& eigenvectors,
                                MyMath::Vector3& eigenvalues)
{
    Matrix3 matrix = input;
    eigenvectors.setIdentity();

    for (int iteration = 0; iteration < MaximumJacobiIterationCount; ++iteration)
    {
        unsigned int indexP = 0;
        unsigned int indexQ = 1;
        double maximumOffDiagonal = std::fabs(matrix.values[0][1]);

        const double offDiagonal02 = std::fabs(matrix.values[0][2]);

        if (offDiagonal02 > maximumOffDiagonal)
        {
            indexP = 0;
            indexQ = 2;
            maximumOffDiagonal = offDiagonal02;
        }

        const double offDiagonal12 = std::fabs(matrix.values[1][2]);

        if (offDiagonal12 > maximumOffDiagonal)
        {
            indexP = 1;
            indexQ = 2;
            maximumOffDiagonal = offDiagonal12;
        }

        if (maximumOffDiagonal <= JacobiConvergenceTolerance)
        {
            break;
        }

        const double valuePP = matrix.values[indexP][indexP];
        const double valueQQ = matrix.values[indexQ][indexQ];
        const double valuePQ = matrix.values[indexP][indexQ];

        if (valuePQ == 0.0)
        {
            continue;
        }

        const double theta = (valueQQ - valuePP) / (2.0 * valuePQ);
        const double tangent =
            theta >= 0.0
                ? 1.0 / (theta + std::sqrt(theta * theta + 1.0))
                : -1.0 / (-theta + std::sqrt(theta * theta + 1.0));

        const double cosine = 1.0 / std::sqrt(1.0 + tangent * tangent);
        const double sine = tangent * cosine;

        for (unsigned int index = 0; index < 3; ++index)
        {
            if (index == indexP || index == indexQ)
            {
                continue;
            }

            const double valueIP = matrix.values[index][indexP];
            const double valueIQ = matrix.values[index][indexQ];

            const double rotatedIP = cosine * valueIP - sine * valueIQ;
            const double rotatedIQ = sine * valueIP + cosine * valueIQ;

            matrix.values[index][indexP] = rotatedIP;
            matrix.values[indexP][index] = rotatedIP;
            matrix.values[index][indexQ] = rotatedIQ;
            matrix.values[indexQ][index] = rotatedIQ;
        }

        matrix.values[indexP][indexP] =
            cosine * cosine * valuePP -
            2.0 * sine * cosine * valuePQ +
            sine * sine * valueQQ;

        matrix.values[indexQ][indexQ] =
            sine * sine * valuePP +
            2.0 * sine * cosine * valuePQ +
            cosine * cosine * valueQQ;

        matrix.values[indexP][indexQ] = 0.0;
        matrix.values[indexQ][indexP] = 0.0;

        for (unsigned int row = 0; row < 3; ++row)
        {
            const double vectorP = eigenvectors.values[row][indexP];
            const double vectorQ = eigenvectors.values[row][indexQ];

            eigenvectors.values[row][indexP] = cosine * vectorP - sine * vectorQ;
            eigenvectors.values[row][indexQ] = sine * vectorP + cosine * vectorQ;
        }
    }

    eigenvalues.set(
        matrix.values[0][0],
        matrix.values[1][1],
        matrix.values[2][2]);
}

}

namespace MyVoxel
{
namespace Meshing
{

FeaturePointSolver::FeaturePointSolver()
    : m_sampleCount(0)
{
}

/// 采样管理

void FeaturePointSolver::clear()
{
    m_sampleCount = 0;
}

bool FeaturePointSolver::addSample(const MyMath::Vector3& point, const MyMath::Vector3& normal)
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "FeaturePointSolver sample point must be finite.");
    MYVOXEL_ASSERT_MESSAGE(normal.isFinite(), "FeaturePointSolver sample normal must be finite.");
    MYVOXEL_ASSERT_MESSAGE(m_sampleCount < static_cast<std::size_t>(MaximumSampleCount),
                           "FeaturePointSolver cannot store more than twelve samples.");

    MyMath::Vector3 unitNormal = normal;

    if (!unitNormal.normalize())
    {
        return false;
    }

    m_points[m_sampleCount] = point;
    m_normals[m_sampleCount] = unitNormal;
    ++m_sampleCount;
    return true;
}

std::size_t FeaturePointSolver::sampleCount() const
{
    return m_sampleCount;
}

/// 顶点求解

MyMath::Vector3 FeaturePointSolver::averagePoint() const
{
    if (m_sampleCount == 0)
    {
        return MyMath::Vector3::zero();
    }

    MyMath::Vector3 result = MyMath::Vector3::zero();

    for (std::size_t sampleIndex = 0; sampleIndex < m_sampleCount; ++sampleIndex)
    {
        result += m_points[sampleIndex];
    }

    result /= static_cast<double>(m_sampleCount);
    return result;
}

bool FeaturePointSolver::solve(MyMath::Vector3& result) const
{
    const MyMath::Vector3 average = averagePoint();
    result = average;

    if (m_sampleCount == 0)
    {
        return false;
    }

    Matrix3 normalMatrix;
    MyMath::Vector3 rightHandSide = MyMath::Vector3::zero();

    for (std::size_t sampleIndex = 0; sampleIndex < m_sampleCount; ++sampleIndex)
    {
        const MyMath::Vector3& point = m_points[sampleIndex];
        const MyMath::Vector3& normal = m_normals[sampleIndex];

        normalMatrix.values[0][0] += normal.x() * normal.x();
        normalMatrix.values[0][1] += normal.x() * normal.y();
        normalMatrix.values[0][2] += normal.x() * normal.z();

        normalMatrix.values[1][0] += normal.y() * normal.x();
        normalMatrix.values[1][1] += normal.y() * normal.y();
        normalMatrix.values[1][2] += normal.y() * normal.z();

        normalMatrix.values[2][0] += normal.z() * normal.x();
        normalMatrix.values[2][1] += normal.z() * normal.y();
        normalMatrix.values[2][2] += normal.z() * normal.z();

        const double planeOffset = MyMath::Vector3::dot(normal, point - average);
        rightHandSide += normal * planeOffset;
    }

    Matrix3 eigenvectors;
    MyMath::Vector3 eigenvalues;

    diagonalizeSymmetricMatrix(normalMatrix, eigenvectors, eigenvalues);

    const double maximumEigenvalue =
        maximumAbsoluteValue(
            eigenvalues.x(),
            eigenvalues.y(),
            eigenvalues.z());

    if (maximumEigenvalue <= (std::numeric_limits<double>::epsilon)())
    {
        return false;
    }

    const double eigenvalueTolerance = maximumEigenvalue * EigenvalueToleranceScale;
    MyMath::Vector3 offset = MyMath::Vector3::zero();
    unsigned int retainedDirectionCount = 0;

    const double values[3] =
    {
        eigenvalues.x(),
        eigenvalues.y(),
        eigenvalues.z()
    };

    for (unsigned int directionIndex = 0; directionIndex < 3; ++directionIndex)
    {
        const double eigenvalue = values[directionIndex];

        if (std::fabs(eigenvalue) < eigenvalueTolerance)
        {
            continue;
        }

        const MyMath::Vector3 direction = matrixColumn(eigenvectors, directionIndex);
        const double component = MyMath::Vector3::dot(direction, rightHandSide) / eigenvalue;

        offset += direction * component;
        ++retainedDirectionCount;
    }

    if (retainedDirectionCount == 0)
    {
        return false;
    }

    const MyMath::Vector3 solvedPoint = average + offset;

    if (!solvedPoint.isFinite())
    {
        return false;
    }

    result = solvedPoint;
    return true;
}

}
}