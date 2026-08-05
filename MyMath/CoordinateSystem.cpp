#include "CoordinateSystem.h"

#include <cassert>
#include <cmath>

namespace MyMath
{

const double CoordinateSystem::DefaultEpsilon = 1.0e-10; // 坐标轴单位性、正交性和旋转转换默认误差。

CoordinateSystem::CoordinateSystem()
    : m_origin(Vector3::zero())
    , m_axes(Matrix3::identity())
{
}

/// 坐标系创建

CoordinateSystem CoordinateSystem::identity()
{
    return CoordinateSystem();
}

CoordinateSystem CoordinateSystem::fromAxes(const Vector3& origin,
                                            const Vector3& xAxis,
                                            const Vector3& yAxis,
                                            const Vector3& zAxis,
                                            double epsilon)
{
    assert(origin.isFinite());
    assert(validateAxes(xAxis, yAxis, zAxis, epsilon));

    CoordinateSystem coordinateSystem;
    coordinateSystem.assign(origin, Matrix3::fromColumns(xAxis, yAxis, zAxis));

    return coordinateSystem;
}

CoordinateSystem CoordinateSystem::fromXY(const Vector3& origin,
                                          const Vector3& xDirection,
                                          const Vector3& yReference,
                                          double epsilon)
{
    assert(origin.isFinite());
    assert(xDirection.isVector(epsilon));
    assert(yReference.isVector(epsilon));

    const Vector3 xAxis = xDirection.normalized(epsilon);
    const Vector3 zDirection = Vector3::cross(xAxis, yReference);

    assert(zDirection.isVector(epsilon));

    const Vector3 zAxis = zDirection.normalized(epsilon);
    const Vector3 yAxis = Vector3::cross(zAxis, xAxis).normalized(epsilon);

    return fromAxes(origin, xAxis, yAxis, zAxis, epsilon);
}

CoordinateSystem CoordinateSystem::fromYZ(const Vector3& origin,
                                          const Vector3& yDirection,
                                          const Vector3& zReference,
                                          double epsilon)
{
    assert(origin.isFinite());
    assert(yDirection.isVector(epsilon));
    assert(zReference.isVector(epsilon));

    const Vector3 yAxis = yDirection.normalized(epsilon);
    const Vector3 xDirection = Vector3::cross(yAxis, zReference);

    assert(xDirection.isVector(epsilon));

    const Vector3 xAxis = xDirection.normalized(epsilon);
    const Vector3 zAxis = Vector3::cross(xAxis, yAxis).normalized(epsilon);

    return fromAxes(origin, xAxis, yAxis, zAxis, epsilon);
}

CoordinateSystem CoordinateSystem::fromZX(const Vector3& origin,
                                          const Vector3& zDirection,
                                          const Vector3& xReference,
                                          double epsilon)
{
    assert(origin.isFinite());
    assert(zDirection.isVector(epsilon));
    assert(xReference.isVector(epsilon));

    const Vector3 zAxis = zDirection.normalized(epsilon);
    const Vector3 yDirection = Vector3::cross(zAxis, xReference);

    assert(yDirection.isVector(epsilon));

    const Vector3 yAxis = yDirection.normalized(epsilon);
    const Vector3 xAxis = Vector3::cross(yAxis, zAxis).normalized(epsilon);

    return fromAxes(origin, xAxis, yAxis, zAxis, epsilon);
}

CoordinateSystem CoordinateSystem::fromQuaternion(const Vector3& origin,
                                                  const Quaternion& orientation,
                                                  double epsilon)
{
    assert(origin.isFinite());
    assert(orientation.isUnit(epsilon));

    return fromMatrix(origin, orientation.toRotationMatrix(epsilon), epsilon);
}

CoordinateSystem CoordinateSystem::fromMatrix(const Vector3& origin,
                                              const Matrix3& matrix,
                                              double epsilon)
{
    assert(origin.isFinite());
    assert(matrix.isOrthogonal(epsilon));

    CoordinateSystem coordinateSystem;
    coordinateSystem.assign(origin, matrix);

    return coordinateSystem;
}

CoordinateSystem CoordinateSystem::fromMatrix(const Matrix4& matrix, double epsilon)
{
    assert(matrix.isFinite());

    const int homogeneousIndex = Matrix4::Size - 1; // 齐次矩阵最后一行和最后一列的下标。

    assert(std::fabs(matrix(homogeneousIndex, 0)) <= epsilon);
    assert(std::fabs(matrix(homogeneousIndex, 1)) <= epsilon);
    assert(std::fabs(matrix(homogeneousIndex, 2)) <= epsilon);
    assert(std::fabs(matrix(homogeneousIndex, homogeneousIndex) - 1.0) <= epsilon);

    const Vector3 origin(matrix(0, homogeneousIndex),
                         matrix(1, homogeneousIndex),
                         matrix(2, homogeneousIndex));

    const Matrix3 axes(matrix(0, 0), matrix(0, 1), matrix(0, 2),
                       matrix(1, 0), matrix(1, 1), matrix(1, 2),
                       matrix(2, 0), matrix(2, 1), matrix(2, 2));

    return fromMatrix(origin, axes, epsilon);
}

/// 坐标系属性

Vector3 CoordinateSystem::origin() const
{
    return m_origin;
}

Matrix3 CoordinateSystem::axes() const
{
    return m_axes;
}

Vector3 CoordinateSystem::xAxis() const
{
    return Vector3(m_axes.m_values[0], m_axes.m_values[3], m_axes.m_values[6]);
}

Vector3 CoordinateSystem::yAxis() const
{
    return Vector3(m_axes.m_values[1], m_axes.m_values[4], m_axes.m_values[7]);
}

Vector3 CoordinateSystem::zAxis() const
{
    return Vector3(m_axes.m_values[2], m_axes.m_values[5], m_axes.m_values[8]);
}

bool CoordinateSystem::setOrigin(const Vector3& origin)
{
    if (!origin.isFinite())
    {
        return false;
    }

    m_origin = origin;
    return true;
}

Matrix4 CoordinateSystem::toMatrix() const
{
    return Matrix4(m_axes.m_values[0], m_axes.m_values[1], m_axes.m_values[2], m_origin.m_x,
                   m_axes.m_values[3], m_axes.m_values[4], m_axes.m_values[5], m_origin.m_y,
                   m_axes.m_values[6], m_axes.m_values[7], m_axes.m_values[8], m_origin.m_z,
                   0.0, 0.0, 0.0, 1.0);
}

/// 状态判断

bool CoordinateSystem::isValid(double epsilon) const
{
    return m_origin.isFinite() && m_axes.isOrthogonal(epsilon);
}

bool CoordinateSystem::isLeftHanded(double epsilon) const
{
    if (!isValid(epsilon))
    {
        return false;
    }

    const double crossX = m_axes.m_values[3] * m_axes.m_values[7] - m_axes.m_values[6] * m_axes.m_values[4];
    const double crossY = m_axes.m_values[6] * m_axes.m_values[1] - m_axes.m_values[0] * m_axes.m_values[7];
    const double crossZ = m_axes.m_values[0] * m_axes.m_values[4] - m_axes.m_values[3] * m_axes.m_values[1];
    const double handedness = crossX * m_axes.m_values[2] + crossY * m_axes.m_values[5] + crossZ * m_axes.m_values[8];

    return handedness < 0.0;
}

bool CoordinateSystem::orientation(Quaternion& result, double epsilon) const
{
    if (!isValid(epsilon) || isLeftHanded(epsilon))
    {
        return false;
    }

    const Quaternion quaternion = Quaternion::fromRotationMatrix(m_axes, epsilon);

    if (!quaternion.isUnit(epsilon))
    {
        return false;
    }

    result = quaternion;
    return true;
}

/// 点、向量和相对坐标系变换

Vector3 CoordinateSystem::toGlobal(const Vector3& localPoint) const
{
    return Vector3(m_origin.m_x + m_axes.m_values[0] * localPoint.m_x + m_axes.m_values[1] * localPoint.m_y + m_axes.m_values[2] * localPoint.m_z,
                   m_origin.m_y + m_axes.m_values[3] * localPoint.m_x + m_axes.m_values[4] * localPoint.m_y + m_axes.m_values[5] * localPoint.m_z,
                   m_origin.m_z + m_axes.m_values[6] * localPoint.m_x + m_axes.m_values[7] * localPoint.m_y + m_axes.m_values[8] * localPoint.m_z);
}

Vector3 CoordinateSystem::toLocal(const Vector3& globalPoint) const
{
    const double offsetX = globalPoint.m_x - m_origin.m_x;
    const double offsetY = globalPoint.m_y - m_origin.m_y;
    const double offsetZ = globalPoint.m_z - m_origin.m_z;

    return Vector3(m_axes.m_values[0] * offsetX + m_axes.m_values[3] * offsetY + m_axes.m_values[6] * offsetZ,
                   m_axes.m_values[1] * offsetX + m_axes.m_values[4] * offsetY + m_axes.m_values[7] * offsetZ,
                   m_axes.m_values[2] * offsetX + m_axes.m_values[5] * offsetY + m_axes.m_values[8] * offsetZ);
}

Vector3 CoordinateSystem::mapVector(const Vector3& localVector) const
{
    return Vector3(m_axes.m_values[0] * localVector.m_x + m_axes.m_values[1] * localVector.m_y + m_axes.m_values[2] * localVector.m_z,
                   m_axes.m_values[3] * localVector.m_x + m_axes.m_values[4] * localVector.m_y + m_axes.m_values[5] * localVector.m_z,
                   m_axes.m_values[6] * localVector.m_x + m_axes.m_values[7] * localVector.m_y + m_axes.m_values[8] * localVector.m_z);
}

Vector3 CoordinateSystem::unmapVector(const Vector3& globalVector) const
{
    return Vector3(m_axes.m_values[0] * globalVector.m_x + m_axes.m_values[3] * globalVector.m_y + m_axes.m_values[6] * globalVector.m_z,
                   m_axes.m_values[1] * globalVector.m_x + m_axes.m_values[4] * globalVector.m_y + m_axes.m_values[7] * globalVector.m_z,
                   m_axes.m_values[2] * globalVector.m_x + m_axes.m_values[5] * globalVector.m_y + m_axes.m_values[8] * globalVector.m_z);
}

CoordinateSystem CoordinateSystem::toGlobalFromRelative(const CoordinateSystem& relativeSystem) const
{
    CoordinateSystem result;

    result.m_origin = toGlobal(relativeSystem.m_origin);
    result.m_axes = m_axes * relativeSystem.m_axes;

    return result;
}

CoordinateSystem CoordinateSystem::inverted() const
{
    CoordinateSystem result;

    result.m_axes = Matrix3(m_axes.m_values[0], m_axes.m_values[3], m_axes.m_values[6],
                            m_axes.m_values[1], m_axes.m_values[4], m_axes.m_values[7],
                            m_axes.m_values[2], m_axes.m_values[5], m_axes.m_values[8]);

    result.m_origin = Vector3(-(m_axes.m_values[0] * m_origin.m_x + m_axes.m_values[3] * m_origin.m_y + m_axes.m_values[6] * m_origin.m_z),
                              -(m_axes.m_values[1] * m_origin.m_x + m_axes.m_values[4] * m_origin.m_y + m_axes.m_values[7] * m_origin.m_z),
                              -(m_axes.m_values[2] * m_origin.m_x + m_axes.m_values[5] * m_origin.m_y + m_axes.m_values[8] * m_origin.m_z));

    return result;
}

/// 坐标系运动

CoordinateSystem& CoordinateSystem::translate(const Vector3& localOffset)
{
    assert(localOffset.isFinite());

    m_origin.m_x += m_axes.m_values[0] * localOffset.m_x + m_axes.m_values[1] * localOffset.m_y + m_axes.m_values[2] * localOffset.m_z;
    m_origin.m_y += m_axes.m_values[3] * localOffset.m_x + m_axes.m_values[4] * localOffset.m_y + m_axes.m_values[5] * localOffset.m_z;
    m_origin.m_z += m_axes.m_values[6] * localOffset.m_x + m_axes.m_values[7] * localOffset.m_y + m_axes.m_values[8] * localOffset.m_z;

    return *this;
}

CoordinateSystem& CoordinateSystem::translateGlobal(const Vector3& globalOffset)
{
    assert(globalOffset.isFinite());

    m_origin.m_x += globalOffset.m_x;
    m_origin.m_y += globalOffset.m_y;
    m_origin.m_z += globalOffset.m_z;

    return *this;
}

bool CoordinateSystem::rotate(const Vector3& localAxis, double angle, double epsilon)
{
    const Quaternion rotation = Quaternion::fromAxisAngle(localAxis, angle, epsilon);

    if (!rotation.isUnit(epsilon))
    {
        return false;
    }

    m_axes *= rotation.toRotationMatrix(epsilon);

    return true;
}

bool CoordinateSystem::rotateX(double angle, double epsilon)
{
    return rotate(Vector3::unitX(), angle, epsilon);
}

bool CoordinateSystem::rotateY(double angle, double epsilon)
{
    return rotate(Vector3::unitY(), angle, epsilon);
}

bool CoordinateSystem::rotateZ(double angle, double epsilon)
{
    return rotate(Vector3::unitZ(), angle, epsilon);
}

bool CoordinateSystem::revolve(const Vector3& globalPoint,
                               const Vector3& globalAxis,
                               double angle,
                               double epsilon)
{
    if (!globalPoint.isFinite())
    {
        return false;
    }

    const Quaternion rotation = Quaternion::fromAxisAngle(globalAxis, angle, epsilon);

    if (!rotation.isUnit(epsilon))
    {
        return false;
    }

    const Matrix3 rotationMatrix = rotation.toRotationMatrix(epsilon);
    const double offsetX = m_origin.m_x - globalPoint.m_x;
    const double offsetY = m_origin.m_y - globalPoint.m_y;
    const double offsetZ = m_origin.m_z - globalPoint.m_z;

    m_origin = Vector3(globalPoint.m_x + rotationMatrix.m_values[0] * offsetX + rotationMatrix.m_values[1] * offsetY + rotationMatrix.m_values[2] * offsetZ,
                       globalPoint.m_y + rotationMatrix.m_values[3] * offsetX + rotationMatrix.m_values[4] * offsetY + rotationMatrix.m_values[5] * offsetZ,
                       globalPoint.m_z + rotationMatrix.m_values[6] * offsetX + rotationMatrix.m_values[7] * offsetY + rotationMatrix.m_values[8] * offsetZ);

    m_axes = rotationMatrix * m_axes;

    return true;
}

CoordinateSystem& CoordinateSystem::mirror()
{
    m_axes.m_values[2] = -m_axes.m_values[2];
    m_axes.m_values[5] = -m_axes.m_values[5];
    m_axes.m_values[8] = -m_axes.m_values[8];

    return *this;
}

/// 内部辅助

bool CoordinateSystem::validateAxes(const Vector3& xAxis,
                                    const Vector3& yAxis,
                                    const Vector3& zAxis,
                                    double epsilon)
{
    if (!xAxis.isUnit(epsilon) || !yAxis.isUnit(epsilon) || !zAxis.isUnit(epsilon))
    {
        return false;
    }

    const double xAxisYDot = xAxis.m_x * yAxis.m_x + xAxis.m_y * yAxis.m_y + xAxis.m_z * yAxis.m_z;
    const double yAxisZDot = yAxis.m_x * zAxis.m_x + yAxis.m_y * zAxis.m_y + yAxis.m_z * zAxis.m_z;
    const double zAxisXDot = zAxis.m_x * xAxis.m_x + zAxis.m_y * xAxis.m_y + zAxis.m_z * xAxis.m_z;

    if (std::fabs(xAxisYDot) > epsilon ||
        std::fabs(yAxisZDot) > epsilon ||
        std::fabs(zAxisXDot) > epsilon)
    {
        return false;
    }

    const double crossX = xAxis.m_y * yAxis.m_z - xAxis.m_z * yAxis.m_y;
    const double crossY = xAxis.m_z * yAxis.m_x - xAxis.m_x * yAxis.m_z;
    const double crossZ = xAxis.m_x * yAxis.m_y - xAxis.m_y * yAxis.m_x;
    const double handedness = crossX * zAxis.m_x + crossY * zAxis.m_y + crossZ * zAxis.m_z;

    return std::fabs(std::fabs(handedness) - 1.0) <= epsilon;
}

void CoordinateSystem::assign(const Vector3& origin, const Matrix3& axes)
{
    m_origin = origin;
    m_axes = axes;
}
/// 算术运算

Matrix4 operator*(const CoordinateSystem& left, const CoordinateSystem& right)
{
    return left.toMatrix() * right.toMatrix();
}

Matrix4 operator*(const CoordinateSystem& left, const Matrix4& right)
{
    return left.toMatrix() * right;
}

Matrix4 operator*(const Matrix4& left, const CoordinateSystem& right)
{
    return left * right.toMatrix();
}
}