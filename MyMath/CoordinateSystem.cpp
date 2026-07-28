#include "CoordinateSystem.h"

#include <cassert>
#include <cmath>

namespace MyMath
{

const double CoordinateSystem::DefaultEpsilon = 1.0e-10; // 坐标轴单位性和正交性检查误差。

CoordinateSystem::CoordinateSystem()
{
    setToIdentity();
}

/// 坐标系创建

CoordinateSystem CoordinateSystem::identity()
{
    return CoordinateSystem();
}




CoordinateSystem CoordinateSystem::fromAxes(
    const Vector3& origin,
    const Vector3& xAxis,
    const Vector3& yAxis,
    const Vector3& zAxis,
    double epsilon)
{
    assert(origin.isFinite());
    assert(validateAxes(xAxis, yAxis, zAxis, epsilon));

    CoordinateSystem coordinateSystem;
    coordinateSystem.assign(origin, xAxis, yAxis, zAxis);
    return coordinateSystem;
}

CoordinateSystem CoordinateSystem::fromXY(
    const Vector3& origin,
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

CoordinateSystem CoordinateSystem::fromYZ(
    const Vector3& origin,
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

CoordinateSystem CoordinateSystem::fromZX(
    const Vector3& origin,
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

CoordinateSystem CoordinateSystem::fromQuaternion(
    const Vector3& origin,
    const Quaternion& orientation,
    double epsilon)
{
    assert(origin.isFinite());
    assert(orientation.isUnit(epsilon));

    Matrix4 matrix = orientation.toRotationMatrix(epsilon);
    matrix.setTranslation(origin);

    return fromMatrix(matrix, epsilon);
}

CoordinateSystem CoordinateSystem::fromMatrix(
    const Matrix4& matrix,
    double epsilon)
{
    CoordinateSystem coordinateSystem;
    coordinateSystem.assign(matrix);

    assert(coordinateSystem.isValid(epsilon));

    return coordinateSystem;
}

/// 坐标系属性

Vector3 CoordinateSystem::origin() const
{
    return translation();
}

Vector3 CoordinateSystem::xAxis() const
{
    return Vector3((*this)(0, 0), (*this)(1, 0), (*this)(2, 0));
}

Vector3 CoordinateSystem::yAxis() const
{
    return Vector3((*this)(0, 1), (*this)(1, 1), (*this)(2, 1));
}

Vector3 CoordinateSystem::zAxis() const
{
    return Vector3((*this)(0, 2), (*this)(1, 2), (*this)(2, 2));
}

bool CoordinateSystem::setOrigin(const Vector3& origin)
{
    if (!origin.isFinite())
    {
        return false;
    }

    setTranslation(origin);
    return true;
}

bool CoordinateSystem::isValid(double epsilon) const
{
    return isFinite() && isAffine(epsilon) && validateAxes(xAxis(), yAxis(), zAxis(), epsilon);
}

bool CoordinateSystem::isLeftHanded(double epsilon) const
{
    if (!isValid(epsilon))
    {
        return false;
    }

    return Vector3::dot(Vector3::cross(xAxis(), yAxis()), zAxis()) < 0.0;
}

bool CoordinateSystem::orientation(Quaternion& result, double epsilon) const
{
    if (!isValid(epsilon) || isLeftHanded(epsilon))
    {
        return false;
    }

    Matrix4 rotationMatrix = static_cast<const Matrix4&>(*this);
    rotationMatrix.setTranslation(Vector3::zero());

    const Quaternion quaternion = Quaternion::fromRotationMatrix(rotationMatrix, epsilon);

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
    return transformPoint(localPoint);
}

Vector3 CoordinateSystem::toLocal(const Vector3& globalPoint) const
{
    return inverted().transformPoint(globalPoint);
}

Vector3 CoordinateSystem::mapVector(const Vector3& localVector) const
{
    return transformVector(localVector);
}

Vector3 CoordinateSystem::unmapVector(const Vector3& globalVector) const
{
    return inverted().transformVector(globalVector);
}

CoordinateSystem CoordinateSystem::toGlobalFromRelative(
    const CoordinateSystem& relativeSystem) const
{
    CoordinateSystem result;
    result.assign(static_cast<const Matrix4&>(*this) *
                  static_cast<const Matrix4&>(relativeSystem));
    return result;
}

CoordinateSystem CoordinateSystem::inverted() const
{
    CoordinateSystem result;

    const Vector3 systemOrigin = origin();
    const Vector3 systemX = xAxis();
    const Vector3 systemY = yAxis();
    const Vector3 systemZ = zAxis();

    result(0, 0) = systemX.x();
    result(0, 1) = systemX.y();
    result(0, 2) = systemX.z();
    result(1, 0) = systemY.x();
    result(1, 1) = systemY.y();
    result(1, 2) = systemY.z();
    result(2, 0) = systemZ.x();
    result(2, 1) = systemZ.y();
    result(2, 2) = systemZ.z();

    result(0, 3) = -Vector3::dot(systemX, systemOrigin);
    result(1, 3) = -Vector3::dot(systemY, systemOrigin);
    result(2, 3) = -Vector3::dot(systemZ, systemOrigin);

    return result;
}

/// 坐标系运动

CoordinateSystem& CoordinateSystem::translate(const Vector3& localOffset)
{
    setTranslation(origin() + mapVector(localOffset));
    return *this;
}

CoordinateSystem& CoordinateSystem::translateGlobal(const Vector3& globalOffset)
{
    setTranslation(origin() + globalOffset);
    return *this;
}

bool CoordinateSystem::rotate(const Vector3& localAxis, double angle, double epsilon)
{
    const Quaternion rotationQuaternion = Quaternion::fromAxisAngle(localAxis, angle, epsilon);

    if (!rotationQuaternion.isUnit(epsilon))
    {
        return false;
    }

    assign(static_cast<const Matrix4&>(*this) * rotationQuaternion.toRotationMatrix(epsilon));
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

bool CoordinateSystem::revolve(const Vector3& globalPoint, const Vector3& globalAxis,
                               double angle, double epsilon)
{
    if (!globalPoint.isFinite())
    {
        return false;
    }

    const Quaternion rotationQuaternion = Quaternion::fromAxisAngle(globalAxis, angle, epsilon);

    if (!rotationQuaternion.isUnit(epsilon))
    {
        return false;
    }

    const Matrix4 toPivot = Matrix4::fromTranslation(-globalPoint);
    const Matrix4 fromPivot = Matrix4::fromTranslation(globalPoint);
    const Matrix4 transformation =
        fromPivot * rotationQuaternion.toRotationMatrix(epsilon) * toPivot;

    assign(transformation * static_cast<const Matrix4&>(*this));
    return true;
}

CoordinateSystem& CoordinateSystem::mirror()
{
    assign(origin(), xAxis(), yAxis(), -zAxis());
    return *this;
}

/// 内部辅助

bool CoordinateSystem::validateAxes(const Vector3& xAxis, const Vector3& yAxis,
                                    const Vector3& zAxis, double epsilon)
{
    if (!xAxis.isUnit(epsilon) || !yAxis.isUnit(epsilon) || !zAxis.isUnit(epsilon))
    {
        return false;
    }

    if (std::fabs(Vector3::dot(xAxis, yAxis)) > epsilon ||
        std::fabs(Vector3::dot(yAxis, zAxis)) > epsilon ||
        std::fabs(Vector3::dot(zAxis, xAxis)) > epsilon)
    {
        return false;
    }

    const double handedness = Vector3::dot(Vector3::cross(xAxis, yAxis), zAxis);

    return std::fabs(std::fabs(handedness) - 1.0) <= epsilon;
}

void CoordinateSystem::assign(const Vector3& origin, const Vector3& xAxis,
                              const Vector3& yAxis, const Vector3& zAxis)
{
    setToIdentity();

    (*this)(0, 0) = xAxis.x();
    (*this)(1, 0) = xAxis.y();
    (*this)(2, 0) = xAxis.z();
    (*this)(0, 1) = yAxis.x();
    (*this)(1, 1) = yAxis.y();
    (*this)(2, 1) = yAxis.z();
    (*this)(0, 2) = zAxis.x();
    (*this)(1, 2) = zAxis.y();
    (*this)(2, 2) = zAxis.z();

    setTranslation(origin);
}

void CoordinateSystem::assign(const Matrix4& matrix)
{
    static_cast<Matrix4&>(*this) = matrix;
}

}