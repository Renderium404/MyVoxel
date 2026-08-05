#include "MeshQuery.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double MinimumLengthTolerance = 1.0e-12; // 查询尺度误差不能低于该绝对长度下限。
const double FloatCoordinateToleranceScale = 1.0; // float坐标量化误差转换为查询长度误差时使用的安全倍数。
const double RayDirectionEpsilon = 1.0e-15; // 射线方向分量接近零时使用的平行判断误差。
const double RayParallelRelativeTolerance = 64.0 * std::numeric_limits<double>::epsilon(); // 射线与三角形近似平行判断使用的相对误差。
const double RayBarycentricTolerance = 8.0 * std::numeric_limits<float>::epsilon(); // 射线命中三角形边或顶点时使用的重心坐标歧义范围。
const unsigned int MaximumRayDirectionAttempts = 16; // 点内外查询最多尝试十六条确定性非轴向射线。
const std::uint32_t MaximumLeafTriangleCount = 8; // 一个BVH叶节点最多连续保存八个三角形。
const std::size_t MaximumTraversalStackSize = 128; // 中位数二分BVH查询允许的最大局部遍历深度.

}

namespace MyVoxel
{
namespace Geometry
{

MeshQuery::BvhNode::BvhNode()
    : leftChild(0)
    , rightChild(0)
    , firstTriangle(0)
    , triangleCount(0)
{
}

bool MeshQuery::BvhNode::isLeaf() const
{
    return triangleCount != 0;
}

MeshQuery::TriangleCenterLess::TriangleCenterLess(unsigned int axisValue)
    : axis(axisValue)
{
}

bool MeshQuery::TriangleCenterLess::operator()(const Triangle& first, const Triangle& second) const
{
    return MeshQuery::component(first.bounds.center(), axis) < MeshQuery::component(second.bounds.center(), axis);
}

MeshQuery::MeshQuery(const Mesh& mesh)
    : m_lengthTolerance(MinimumLengthTolerance)
    , m_lengthToleranceSquared(MinimumLengthTolerance * MinimumLengthTolerance)
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isGeometryValid(), "MeshQuery requires a geometrically valid Geometry::Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!mesh.isEmpty(), "MeshQuery requires a non-empty Geometry::Mesh.");

    if (!mesh.isGeometryValid() || mesh.isEmpty())
    {
        return;
    }

    const bool hasDegenerateTriangles = mesh.hasDegenerateTriangles();

    MYVOXEL_ASSERT_MESSAGE(
        !hasDegenerateTriangles,
        "MeshQuery does not accept degenerate triangles.");

    if (hasDegenerateTriangles)
    {
        return;
    }

    const Bounds3 meshBounds = mesh.localBounds();

    if (!meshBounds.isValid() || !meshBounds.hasVolume())
    {
        return;
    }

    m_lengthTolerance = queryLengthTolerance(meshBounds);
    m_lengthToleranceSquared = m_lengthTolerance * m_lengthTolerance;

    const std::vector<MeshVertex>& vertices = mesh.vertices();
    const std::vector<std::uint32_t>& indices = mesh.indices();

    MYVOXEL_ASSERT_MESSAGE(indices.size() % 3 == 0, "MeshQuery index count must be divisible by three.");

    const std::size_t triangleCountValue = indices.size() / 3;
    const std::size_t maximumTriangleCount =
        static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)());

    MYVOXEL_ASSERT_MESSAGE(
        triangleCountValue <= maximumTriangleCount,
        "MeshQuery triangle count exceeds the 32-bit BVH range.");

    if (triangleCountValue > maximumTriangleCount)
    {
        return;
    }

    m_triangles.reserve(triangleCountValue);

    for (std::size_t indexPosition = 0; indexPosition < indices.size(); indexPosition += 3)
    {
        const std::uint32_t index0 = indices[indexPosition];
        const std::uint32_t index1 = indices[indexPosition + 1];
        const std::uint32_t index2 = indices[indexPosition + 2];

        Triangle triangle;
        triangle.point0 = vertexPosition(vertices[index0]);
        triangle.point1 = vertexPosition(vertices[index1]);
        triangle.point2 = vertexPosition(vertices[index2]);
        triangle.bounds = triangleBounds(triangle.point0, triangle.point1, triangle.point2);

        m_triangles.push_back(triangle);
    }

    if (m_triangles.empty())
    {
        return;
    }

    if (m_triangles.size() <= m_bvhNodes.max_size() / 2)
    {
        m_bvhNodes.reserve(m_triangles.size() * 2 - 1);
    }

    const std::uint32_t rootNode =
        buildBvhNode(0, static_cast<std::uint32_t>(m_triangles.size()));

    if (rootNode == 0 && !m_bvhNodes.empty())
    {
        m_queryBounds = m_bvhNodes[0].bounds;
        m_valid = m_queryBounds.isValid() && m_queryBounds.hasVolume();
    }

    MYVOXEL_ASSERT_MESSAGE(m_valid, "MeshQuery construction produced invalid BVH query data.");
}

/// 状态判断

bool MeshQuery::isValid() const
{
    return m_valid;
}

/// 查询数据

const Bounds3& MeshQuery::queryBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid MeshQuery.");
    return m_queryBounds;
}

std::size_t MeshQuery::triangleCount() const
{
    return m_triangles.size();
}

/// 点查询

double MeshQuery::distanceSquaredToPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid MeshQuery.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "MeshQuery point must be finite.");

    double minimumDistanceSquared = (std::numeric_limits<double>::max)();
    std::uint32_t nodeStack[MaximumTraversalStackSize];
    std::size_t stackSize = 1;
    nodeStack[0] = 0;

    while (stackSize != 0)
    {
        const std::uint32_t nodeIndex = nodeStack[--stackSize];
        const BvhNode& node = m_bvhNodes[nodeIndex];

        if (pointBoundsDistanceSquared(point, node.bounds) > minimumDistanceSquared)
        {
            continue;
        }

        if (node.isLeaf())
        {
            const std::uint32_t endTriangle = node.firstTriangle + node.triangleCount;

            for (std::uint32_t triangleIndex = node.firstTriangle; triangleIndex < endTriangle; ++triangleIndex)
            {
                minimumDistanceSquared =
                    (std::min)(minimumDistanceSquared, pointTriangleDistanceSquared(point, m_triangles[triangleIndex]));
            }

            continue;
        }

        const double leftDistanceSquared = pointBoundsDistanceSquared(point, m_bvhNodes[node.leftChild].bounds);
        const double rightDistanceSquared = pointBoundsDistanceSquared(point, m_bvhNodes[node.rightChild].bounds);

        MYVOXEL_ASSERT_MESSAGE(stackSize + 2 <= MaximumTraversalStackSize, "MeshQuery BVH distance traversal stack exceeded its fixed capacity.");

        if (leftDistanceSquared <= rightDistanceSquared)
        {
            if (rightDistanceSquared <= minimumDistanceSquared)
            {
                nodeStack[stackSize++] = node.rightChild;
            }

            if (leftDistanceSquared <= minimumDistanceSquared)
            {
                nodeStack[stackSize++] = node.leftChild;
            }
        }
        else
        {
            if (leftDistanceSquared <= minimumDistanceSquared)
            {
                nodeStack[stackSize++] = node.leftChild;
            }

            if (rightDistanceSquared <= minimumDistanceSquared)
            {
                nodeStack[stackSize++] = node.rightChild;
            }
        }
    }

    return minimumDistanceSquared;
}

double MeshQuery::distanceToPoint(const MyMath::Vector3& point) const
{
    return std::sqrt(distanceSquaredToPoint(point));
}

bool MeshQuery::containsPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid MeshQuery.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "MeshQuery point must be finite.");

    if (!m_queryBounds.contains(point, m_lengthTolerance))
    {
        return false;
    }

    if (isPointOnSurface(point))
    {
        return true;
    }

    for (unsigned int attemptIndex = 0; attemptIndex < MaximumRayDirectionAttempts; ++attemptIndex)
    {
        std::size_t intersectionCount = 0;

        if (tryRayIntersectionCount(point, rayDirection(attemptIndex), intersectionCount))
        {
            return intersectionCount % 2 != 0;
        }
    }

    // 查询点已经确认不在边界误差范围内，使用小于边界误差的确定性偏移不会跨越网格表面。
    const double offset = m_lengthTolerance * 0.25;
    const MyMath::Vector3 offsetPoint(
        point.x() + offset,
        point.y() - offset * 0.5,
        point.z() + offset * 0.25);

    for (unsigned int attemptIndex = 0; attemptIndex < MaximumRayDirectionAttempts; ++attemptIndex)
    {
        std::size_t intersectionCount = 0;

        if (tryRayIntersectionCount(offsetPoint, rayDirection(attemptIndex), intersectionCount))
        {
            return intersectionCount % 2 != 0;
        }
    }

    MYVOXEL_ASSERT_MESSAGE(
        false,
        "MeshQuery could not find a ray that avoids triangle edge and vertex ambiguity.");

    return rayIntersectionCountFallback(point, rayDirection(0)) % 2 != 0;
}

/// 范围查询

ShapeRelation MeshQuery::classifyBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid MeshQuery.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "MeshQuery bounds must be valid.");

    return classifyBounds(bounds.center(), bounds.extent());
}

ShapeRelation MeshQuery::classifyBounds(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid MeshQuery.");
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "MeshQuery bounds center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "MeshQuery bounds extent must be finite and non-negative.");

    if (!boundsIntersect(center, extent, m_queryBounds, m_lengthTolerance))
    {
        return ShapeRelation::Outside;
    }

    if (intersectsAnyTriangle(center, extent))
    {
        return ShapeRelation::Intersecting;
    }

    return containsPoint(center) ? ShapeRelation::Inside : ShapeRelation::Outside;
}

/// 内部辅助

MyMath::Vector3 MeshQuery::vertexPosition(const MeshVertex& vertex)
{
    return MyMath::Vector3(
        static_cast<double>(vertex.x),
        static_cast<double>(vertex.y),
        static_cast<double>(vertex.z));
}

Bounds3 MeshQuery::triangleBounds(const MyMath::Vector3& point0,
                                  const MyMath::Vector3& point1,
                                  const MyMath::Vector3& point2)
{
    Bounds3 result;
    result.include(point0);
    result.include(point1);
    result.include(point2);
    return result;
}

double MeshQuery::component(const MyMath::Vector3& vector, unsigned int axis)
{
    MYVOXEL_ASSERT_MESSAGE(axis < 3, "MeshQuery coordinate axis must be in range [0, 2].");

    if (axis == 0)
    {
        return vector.x();
    }

    if (axis == 1)
    {
        return vector.y();
    }

    return vector.z();
}

bool MeshQuery::isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

double MeshQuery::queryLengthTolerance(const Bounds3& bounds)
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "MeshQuery tolerance requires valid bounds.");

    const MyMath::Vector3 minimum = bounds.minimum();
    const MyMath::Vector3 maximum = bounds.maximum();
    double coordinateScale = 1.0;

    coordinateScale = (std::max)(coordinateScale, std::fabs(minimum.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(minimum.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(minimum.z()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(maximum.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(maximum.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(maximum.z()));

    const double floatTolerance =
        coordinateScale *
        static_cast<double>(std::numeric_limits<float>::epsilon()) *
        FloatCoordinateToleranceScale;

    return (std::max)(MinimumLengthTolerance, floatTolerance);
}

MyMath::Vector3 MeshQuery::rayDirection(unsigned int attemptIndex)
{
    if (attemptIndex == 0)
    {
        return MyMath::Vector3(1.0, 0.3713906763541037, 0.5299989400031800);
    }

    if (attemptIndex == 1)
    {
        return MyMath::Vector3(1.0, -0.6147348123496731, 0.2817391079501187);
    }

    if (attemptIndex == 2)
    {
        return MyMath::Vector3(0.4472135954999579, 1.0, -0.7123498417312297);
    }

    if (attemptIndex == 3)
    {
        return MyMath::Vector3(-0.3281739130434783, 0.5932718492740917, 1.0);
    }

    const double firstFraction =
        std::fmod(0.3713906763541037 + static_cast<double>(attemptIndex) * 0.6180339887498948, 1.0);

    const double secondFraction =
        std::fmod(0.5299989400031800 + static_cast<double>(attemptIndex) * 0.4142135623730950, 1.0);

    const double sign = (attemptIndex & 1U) != 0 ? -1.0 : 1.0;

    return MyMath::Vector3(
        1.0,
        0.2 + firstFraction * 0.8,
        sign * (0.2 + secondFraction * 0.8));
}

bool MeshQuery::boundsIntersect(const MyMath::Vector3& center,
                                const MyMath::Vector3& extent,
                                const Bounds3& bounds,
                                double tolerance)
{
    const MyMath::Vector3 minimum = center - extent;
    const MyMath::Vector3 maximum = center + extent;

    return maximum.x() + tolerance >= bounds.minimum().x() &&
           bounds.maximum().x() + tolerance >= minimum.x() &&
           maximum.y() + tolerance >= bounds.minimum().y() &&
           bounds.maximum().y() + tolerance >= minimum.y() &&
           maximum.z() + tolerance >= bounds.minimum().z() &&
           bounds.maximum().z() + tolerance >= minimum.z();
}

double MeshQuery::pointBoundsDistanceSquared(const MyMath::Vector3& point, const Bounds3& bounds)
{
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaZ = 0.0;

    if (point.x() < bounds.minimum().x())
    {
        deltaX = bounds.minimum().x() - point.x();
    }
    else if (point.x() > bounds.maximum().x())
    {
        deltaX = point.x() - bounds.maximum().x();
    }

    if (point.y() < bounds.minimum().y())
    {
        deltaY = bounds.minimum().y() - point.y();
    }
    else if (point.y() > bounds.maximum().y())
    {
        deltaY = point.y() - bounds.maximum().y();
    }

    if (point.z() < bounds.minimum().z())
    {
        deltaZ = bounds.minimum().z() - point.z();
    }
    else if (point.z() > bounds.maximum().z())
    {
        deltaZ = point.z() - bounds.maximum().z();
    }

    return deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
}

bool MeshQuery::rayIntersectsBounds(const MyMath::Vector3& origin,
                                    const MyMath::Vector3& direction,
                                    const Bounds3& bounds)
{
    double minimumDistance = 0.0;
    double maximumDistance = (std::numeric_limits<double>::max)();

    for (unsigned int axis = 0; axis < 3; ++axis)
    {
        const double originComponent = component(origin, axis);
        const double directionComponent = component(direction, axis);
        const double minimumComponent = component(bounds.minimum(), axis);
        const double maximumComponent = component(bounds.maximum(), axis);

        if (std::fabs(directionComponent) <= RayDirectionEpsilon)
        {
            if (originComponent < minimumComponent || originComponent > maximumComponent)
            {
                return false;
            }

            continue;
        }

        const double inverseDirection = 1.0 / directionComponent;
        double firstDistance = (minimumComponent - originComponent) * inverseDirection;
        double secondDistance = (maximumComponent - originComponent) * inverseDirection;

        if (firstDistance > secondDistance)
        {
            std::swap(firstDistance, secondDistance);
        }

        minimumDistance = (std::max)(minimumDistance, firstDistance);
        maximumDistance = (std::min)(maximumDistance, secondDistance);

        if (maximumDistance < minimumDistance)
        {
            return false;
        }
    }

    return maximumDistance > RayDirectionEpsilon;
}

double MeshQuery::pointSegmentDistanceSquared(const MyMath::Vector3& point,
                                              const MyMath::Vector3& segmentStart,
                                              const MyMath::Vector3& segmentEnd)
{
    const MyMath::Vector3 segment = segmentEnd - segmentStart;
    const double lengthSquared = segment.lengthSquared();

    if (lengthSquared <= 0.0)
    {
        return point.distanceSquaredTo(segmentStart);
    }

    double factor = MyMath::Vector3::dot(point - segmentStart, segment) / lengthSquared;
    factor = (std::max)(0.0, (std::min)(1.0, factor));

    return point.distanceSquaredTo(segmentStart + segment * factor);
}

double MeshQuery::pointTriangleDistanceSquared(const MyMath::Vector3& point, const Triangle& triangle)
{
    const MyMath::Vector3 edge01 = triangle.point1 - triangle.point0;
    const MyMath::Vector3 edge02 = triangle.point2 - triangle.point0;
    const MyMath::Vector3 point0ToPoint = point - triangle.point0;

    const double d1 = MyMath::Vector3::dot(edge01, point0ToPoint);
    const double d2 = MyMath::Vector3::dot(edge02, point0ToPoint);

    if (d1 <= 0.0 && d2 <= 0.0)
    {
        return point.distanceSquaredTo(triangle.point0);
    }

    const MyMath::Vector3 point1ToPoint = point - triangle.point1;
    const double d3 = MyMath::Vector3::dot(edge01, point1ToPoint);
    const double d4 = MyMath::Vector3::dot(edge02, point1ToPoint);

    if (d3 >= 0.0 && d4 <= d3)
    {
        return point.distanceSquaredTo(triangle.point1);
    }

    const double edge01Region = d1 * d4 - d3 * d2;

    if (edge01Region <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double factor = d1 / (d1 - d3);
        return point.distanceSquaredTo(triangle.point0 + edge01 * factor);
    }

    const MyMath::Vector3 point2ToPoint = point - triangle.point2;
    const double d5 = MyMath::Vector3::dot(edge01, point2ToPoint);
    const double d6 = MyMath::Vector3::dot(edge02, point2ToPoint);

    if (d6 >= 0.0 && d5 <= d6)
    {
        return point.distanceSquaredTo(triangle.point2);
    }

    const double edge02Region = d5 * d2 - d1 * d6;

    if (edge02Region <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double factor = d2 / (d2 - d6);
        return point.distanceSquaredTo(triangle.point0 + edge02 * factor);
    }

    const double edge12Region = d3 * d6 - d5 * d4;

    if (edge12Region <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0)
    {
        const MyMath::Vector3 edge12 = triangle.point2 - triangle.point1;
        const double factor = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return point.distanceSquaredTo(triangle.point1 + edge12 * factor);
    }

    const MyMath::Vector3 normal = MyMath::Vector3::cross(edge01, edge02);
    const double normalLengthSquared = normal.lengthSquared();

    if (normalLengthSquared <= 0.0)
    {
        return (std::min)(
            pointSegmentDistanceSquared(point, triangle.point0, triangle.point1),
            (std::min)(
                pointSegmentDistanceSquared(point, triangle.point1, triangle.point2),
                pointSegmentDistanceSquared(point, triangle.point2, triangle.point0)));
    }

    const double signedPlaneDistance = MyMath::Vector3::dot(point0ToPoint, normal);
    return signedPlaneDistance * signedPlaneDistance / normalLengthSquared;
}

MeshQuery::RayTriangleRelation MeshQuery::classifyRayTriangle(
    const MyMath::Vector3& origin,
    const MyMath::Vector3& direction,
    const Triangle& triangle,
    double lengthTolerance)
{
    const MyMath::Vector3 edge1 = triangle.point1 - triangle.point0;
    const MyMath::Vector3 edge2 = triangle.point2 - triangle.point0;
    const MyMath::Vector3 crossDirectionEdge2 = MyMath::Vector3::cross(direction, edge2);
    const double determinant = MyMath::Vector3::dot(edge1, crossDirectionEdge2);

    const double determinantScaleSquared =
        edge1.lengthSquared() * edge2.lengthSquared() * direction.lengthSquared();

    if (!std::isfinite(determinantScaleSquared) || determinantScaleSquared <= 0.0)
    {
        return RayTriangleRelation::Ambiguous;
    }

    const double determinantTolerance =
        std::sqrt(determinantScaleSquared) * RayParallelRelativeTolerance;

    const MyMath::Vector3 originOffset = origin - triangle.point0;

    if (std::fabs(determinant) <= determinantTolerance)
    {
        const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(edge1, edge2);
        const double normalLength = std::sqrt(triangleNormal.lengthSquared());
        const double planeOffset = MyMath::Vector3::dot(originOffset, triangleNormal);

        return std::fabs(planeOffset) <= lengthTolerance * normalLength
            ? RayTriangleRelation::Ambiguous
            : RayTriangleRelation::Miss;
    }

    const double inverseDeterminant = 1.0 / determinant;

    const double coordinateU =
        MyMath::Vector3::dot(originOffset, crossDirectionEdge2) * inverseDeterminant;

    if (coordinateU < -RayBarycentricTolerance ||
        coordinateU > 1.0 + RayBarycentricTolerance)
    {
        return RayTriangleRelation::Miss;
    }

    const MyMath::Vector3 crossOriginEdge1 = MyMath::Vector3::cross(originOffset, edge1);

    const double coordinateV =
        MyMath::Vector3::dot(direction, crossOriginEdge1) * inverseDeterminant;

    const double coordinateW = 1.0 - coordinateU - coordinateV;

    if (coordinateV < -RayBarycentricTolerance ||
        coordinateW < -RayBarycentricTolerance)
    {
        return RayTriangleRelation::Miss;
    }

    const double distance =
        MyMath::Vector3::dot(edge2, crossOriginEdge1) * inverseDeterminant;

    const double directionLength = std::sqrt(direction.lengthSquared());
    const double distanceTolerance = lengthTolerance / directionLength;

    if (distance < -distanceTolerance)
    {
        return RayTriangleRelation::Miss;
    }

    if (distance <= distanceTolerance ||
        coordinateU <= RayBarycentricTolerance ||
        coordinateV <= RayBarycentricTolerance ||
        coordinateW <= RayBarycentricTolerance)
    {
        return RayTriangleRelation::Ambiguous;
    }

    return RayTriangleRelation::Hit;
}

bool MeshQuery::rayIntersectsTriangleInclusive(const MyMath::Vector3& origin,
                                               const MyMath::Vector3& direction,
                                               const Triangle& triangle)
{
    const MyMath::Vector3 edge1 = triangle.point1 - triangle.point0;
    const MyMath::Vector3 edge2 = triangle.point2 - triangle.point0;
    const MyMath::Vector3 crossDirectionEdge2 = MyMath::Vector3::cross(direction, edge2);
    const double determinant = MyMath::Vector3::dot(edge1, crossDirectionEdge2);

    if (std::fabs(determinant) <= RayDirectionEpsilon)
    {
        return false;
    }

    const double inverseDeterminant = 1.0 / determinant;
    const MyMath::Vector3 originOffset = origin - triangle.point0;
    const double coordinateU =
        MyMath::Vector3::dot(originOffset, crossDirectionEdge2) * inverseDeterminant;

    if (coordinateU < 0.0 || coordinateU > 1.0)
    {
        return false;
    }

    const MyMath::Vector3 crossOriginEdge1 = MyMath::Vector3::cross(originOffset, edge1);
    const double coordinateV =
        MyMath::Vector3::dot(direction, crossOriginEdge1) * inverseDeterminant;

    if (coordinateV < 0.0 || coordinateU + coordinateV > 1.0)
    {
        return false;
    }

    const double distance =
        MyMath::Vector3::dot(edge2, crossOriginEdge1) * inverseDeterminant;

    return distance > RayDirectionEpsilon;
}

bool MeshQuery::triangleIntersectsBounds(const Triangle& triangle,
                                         const MyMath::Vector3& center,
                                         const MyMath::Vector3& extent,
                                         double lengthTolerance)
{
    const MyMath::Vector3 relativePoint0 = triangle.point0 - center;
    const MyMath::Vector3 relativePoint1 = triangle.point1 - center;
    const MyMath::Vector3 relativePoint2 = triangle.point2 - center;

    const MyMath::Vector3 edge01 = relativePoint1 - relativePoint0;
    const MyMath::Vector3 edge12 = relativePoint2 - relativePoint1;
    const MyMath::Vector3 edge20 = relativePoint0 - relativePoint2;

    const MyMath::Vector3 axisX = MyMath::Vector3::unitX();
    const MyMath::Vector3 axisY = MyMath::Vector3::unitY();
    const MyMath::Vector3 axisZ = MyMath::Vector3::unitZ();

    if (!overlapsOnAxis(relativePoint0, relativePoint1, relativePoint2, extent, axisX, lengthTolerance) ||
        !overlapsOnAxis(relativePoint0, relativePoint1, relativePoint2, extent, axisY, lengthTolerance) ||
        !overlapsOnAxis(relativePoint0, relativePoint1, relativePoint2, extent, axisZ, lengthTolerance))
    {
        return false;
    }

    const MyMath::Vector3 triangleNormal =
        MyMath::Vector3::cross(edge01, relativePoint2 - relativePoint0);

    if (!overlapsOnAxis(
            relativePoint0, relativePoint1, relativePoint2,
            extent, triangleNormal, lengthTolerance))
    {
        return false;
    }

    const MyMath::Vector3 edges[3] = {edge01, edge12, edge20};
    const MyMath::Vector3 boxAxes[3] = {axisX, axisY, axisZ};

    for (std::size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
    {
        for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex)
        {
            const MyMath::Vector3 separatingAxis =
                MyMath::Vector3::cross(edges[edgeIndex], boxAxes[axisIndex]);

            if (!overlapsOnAxis(
                    relativePoint0, relativePoint1, relativePoint2,
                    extent, separatingAxis, lengthTolerance))
            {
                return false;
            }
        }
    }

    return true;
}

bool MeshQuery::overlapsOnAxis(const MyMath::Vector3& relativePoint0,
                               const MyMath::Vector3& relativePoint1,
                               const MyMath::Vector3& relativePoint2,
                               const MyMath::Vector3& extent,
                               const MyMath::Vector3& axis,
                               double lengthTolerance)
{
    const double axisLengthSquared = axis.lengthSquared();

    if (!std::isfinite(axisLengthSquared) || axisLengthSquared <= 0.0)
    {
        return true;
    }

    const double projection0 = MyMath::Vector3::dot(relativePoint0, axis);
    const double projection1 = MyMath::Vector3::dot(relativePoint1, axis);
    const double projection2 = MyMath::Vector3::dot(relativePoint2, axis);

    const double minimumProjection =
        (std::min)(projection0, (std::min)(projection1, projection2));

    const double maximumProjection =
        (std::max)(projection0, (std::max)(projection1, projection2));

    const double boxRadius =
        extent.x() * std::fabs(axis.x()) +
        extent.y() * std::fabs(axis.y()) +
        extent.z() * std::fabs(axis.z());

    const double axisLength = std::sqrt(axisLengthSquared);
    const double projectionScale =
        (std::max)(boxRadius, (std::max)(std::fabs(minimumProjection), std::fabs(maximumProjection)));

    const double tolerance =
        lengthTolerance * axisLength +
        16.0 * std::numeric_limits<double>::epsilon() * (1.0 + projectionScale);

    return minimumProjection <= boxRadius + tolerance &&
           maximumProjection >= -boxRadius - tolerance;
}

std::uint32_t MeshQuery::buildBvhNode(std::uint32_t firstTriangle, std::uint32_t triangleCountValue)
{
    MYVOXEL_ASSERT_MESSAGE(triangleCountValue != 0, "MeshQuery cannot build an empty BVH node.");
    MYVOXEL_ASSERT_MESSAGE(static_cast<std::size_t>(firstTriangle) + triangleCountValue <= m_triangles.size(),
                           "MeshQuery BVH triangle range exceeds the triangle array.");
    MYVOXEL_ASSERT_MESSAGE(m_bvhNodes.size() < static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)()),
                           "MeshQuery BVH node count exceeds the 32-bit index range.");

    const std::uint32_t nodeIndex = static_cast<std::uint32_t>(m_bvhNodes.size());
    m_bvhNodes.push_back(BvhNode());

    Bounds3 nodeBounds;
    Bounds3 centerBounds;
    const std::uint32_t endTriangle = firstTriangle + triangleCountValue;

    for (std::uint32_t triangleIndex = firstTriangle; triangleIndex < endTriangle; ++triangleIndex)
    {
        nodeBounds.include(m_triangles[triangleIndex].bounds);
        centerBounds.include(m_triangles[triangleIndex].bounds.center());
    }

    MYVOXEL_ASSERT_MESSAGE(nodeBounds.isValid(), "MeshQuery BVH node produced invalid bounds.");
    MYVOXEL_ASSERT_MESSAGE(centerBounds.isValid(), "MeshQuery BVH node produced invalid center bounds.");

    m_bvhNodes[nodeIndex].bounds = nodeBounds;

    if (triangleCountValue <= MaximumLeafTriangleCount)
    {
        m_bvhNodes[nodeIndex].firstTriangle = firstTriangle;
        m_bvhNodes[nodeIndex].triangleCount = triangleCountValue;
        return nodeIndex;
    }

    const MyMath::Vector3 centerSize = centerBounds.size();
    unsigned int splitAxis = 0;

    if (centerSize.y() > centerSize.x())
    {
        splitAxis = 1;
    }

    if (centerSize.z() > component(centerSize, splitAxis))
    {
        splitAxis = 2;
    }

    const std::uint32_t leftTriangleCount = triangleCountValue / 2;
    const std::uint32_t middleTriangle = firstTriangle + leftTriangleCount;

    std::nth_element(
        m_triangles.begin() + firstTriangle,
        m_triangles.begin() + middleTriangle,
        m_triangles.begin() + endTriangle,
        TriangleCenterLess(splitAxis));

    const std::uint32_t leftChild = buildBvhNode(firstTriangle, leftTriangleCount);
    const std::uint32_t rightChild = buildBvhNode(middleTriangle, triangleCountValue - leftTriangleCount);

    m_bvhNodes[nodeIndex].leftChild = leftChild;
    m_bvhNodes[nodeIndex].rightChild = rightChild;
    return nodeIndex;
}

bool MeshQuery::isPointOnSurface(const MyMath::Vector3& point) const
{
    const double boundaryToleranceSquared = m_lengthToleranceSquared;
    std::uint32_t nodeStack[MaximumTraversalStackSize];
    std::size_t stackSize = 1;
    nodeStack[0] = 0;

    while (stackSize != 0)
    {
        const BvhNode& node = m_bvhNodes[nodeStack[--stackSize]];

        if (!node.bounds.contains(point, m_lengthTolerance))
        {
            continue;
        }

        if (node.isLeaf())
        {
            const std::uint32_t endTriangle = node.firstTriangle + node.triangleCount;

            for (std::uint32_t triangleIndex = node.firstTriangle; triangleIndex < endTriangle; ++triangleIndex)
            {
                const Triangle& triangle = m_triangles[triangleIndex];

                if (triangle.bounds.contains(point, m_lengthTolerance) &&
                    pointTriangleDistanceSquared(point, triangle) <= boundaryToleranceSquared)
                {
                    return true;
                }
            }

            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(stackSize + 2 <= MaximumTraversalStackSize, "MeshQuery BVH surface traversal stack exceeded its fixed capacity.");
        nodeStack[stackSize++] = node.rightChild;
        nodeStack[stackSize++] = node.leftChild;
    }

    return false;
}

bool MeshQuery::tryRayIntersectionCount(const MyMath::Vector3& origin,
                                              const MyMath::Vector3& direction,
                                              std::size_t& intersectionCount) const
{
    intersectionCount = 0;

    std::uint32_t nodeStack[MaximumTraversalStackSize];
    std::size_t stackSize = 1;
    nodeStack[0] = 0;

    while (stackSize != 0)
    {
        const BvhNode& node = m_bvhNodes[nodeStack[--stackSize]];

        if (!rayIntersectsBounds(origin, direction, node.bounds))
        {
            continue;
        }

        if (node.isLeaf())
        {
            const std::uint32_t endTriangle = node.firstTriangle + node.triangleCount;

            for (std::uint32_t triangleIndex = node.firstTriangle;
                 triangleIndex < endTriangle;
                 ++triangleIndex)
            {
                const RayTriangleRelation relation =
                    classifyRayTriangle(origin, direction, m_triangles[triangleIndex], m_lengthTolerance);

                if (relation == RayTriangleRelation::Ambiguous)
                {
                    return false;
                }

                if (relation == RayTriangleRelation::Hit)
                {
                    ++intersectionCount;
                }
            }

            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(
            stackSize + 2 <= MaximumTraversalStackSize,
            "MeshQuery BVH ray traversal stack exceeded its fixed capacity.");

        nodeStack[stackSize++] = node.rightChild;
        nodeStack[stackSize++] = node.leftChild;
    }

    return true;
}

std::size_t MeshQuery::rayIntersectionCountFallback(
    const MyMath::Vector3& origin,
    const MyMath::Vector3& direction) const
{
    std::size_t intersectionCount = 0;

    std::uint32_t nodeStack[MaximumTraversalStackSize];
    std::size_t stackSize = 1;
    nodeStack[0] = 0;

    while (stackSize != 0)
    {
        const BvhNode& node = m_bvhNodes[nodeStack[--stackSize]];

        if (!rayIntersectsBounds(origin, direction, node.bounds))
        {
            continue;
        }

        if (node.isLeaf())
        {
            const std::uint32_t endTriangle = node.firstTriangle + node.triangleCount;

            for (std::uint32_t triangleIndex = node.firstTriangle;
                 triangleIndex < endTriangle;
                 ++triangleIndex)
            {
                if (rayIntersectsTriangleInclusive(origin, direction, m_triangles[triangleIndex]))
                {
                    ++intersectionCount;
                }
            }

            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(
            stackSize + 2 <= MaximumTraversalStackSize,
            "MeshQuery BVH fallback ray traversal stack exceeded its fixed capacity.");

        nodeStack[stackSize++] = node.rightChild;
        nodeStack[stackSize++] = node.leftChild;
    }

    return intersectionCount;
}

bool MeshQuery::intersectsAnyTriangle(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    std::uint32_t nodeStack[MaximumTraversalStackSize];
    std::size_t stackSize = 1;
    nodeStack[0] = 0;

    while (stackSize != 0)
    {
        const BvhNode& node = m_bvhNodes[nodeStack[--stackSize]];

        if (!boundsIntersect(center, extent, node.bounds, m_lengthTolerance))
        {
            continue;
        }

        if (node.isLeaf())
        {
            const std::uint32_t endTriangle = node.firstTriangle + node.triangleCount;

            for (std::uint32_t triangleIndex = node.firstTriangle; triangleIndex < endTriangle; ++triangleIndex)
            {
                const Triangle& triangle = m_triangles[triangleIndex];

                if (boundsIntersect(center, extent, triangle.bounds, m_lengthTolerance) &&
                    triangleIntersectsBounds(triangle, center, extent, m_lengthTolerance))
                {
                    return true;
                }
            }

            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(stackSize + 2 <= MaximumTraversalStackSize, "MeshQuery BVH bounds traversal stack exceeded its fixed capacity.");
        nodeStack[stackSize++] = node.rightChild;
        nodeStack[stackSize++] = node.leftChild;
    }

    return false;
}

}
}