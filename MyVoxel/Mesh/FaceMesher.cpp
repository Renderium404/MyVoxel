#include "FaceMesher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const unsigned int DefaultCurveSegmentCount = 32; // 当前非直线边界默认离散为32段，优先保证测试稳定和实现直接。
const double DefaultTriangulationTolerance = 1.0e-12; // UV三角化默认绝对误差，仅用于数值去重和退化判断。

struct ParameterPoint
{
    ParameterPoint()
        : u(0.0)
        , v(0.0)
    {
    }

    ParameterPoint(double uValue, double vValue)
        : u(uValue)
        , v(vValue)
    {
    }

    double u; // 当前边界点的U参数。
    double v; // 当前边界点的V参数。
};

double cross2D(const ParameterPoint& first, const ParameterPoint& second, const ParameterPoint& third)
{
    return (second.u - first.u) * (third.v - first.v) - (second.v - first.v) * (third.u - first.u);
}

double distanceSquared(const ParameterPoint& first, const ParameterPoint& second)
{
    const double deltaU = second.u - first.u;
    const double deltaV = second.v - first.v;
    return deltaU * deltaU + deltaV * deltaV;
}

bool samePoint(const ParameterPoint& first, const ParameterPoint& second, double tolerance)
{
    return distanceSquared(first, second) <= tolerance * tolerance;
}

double polygonSignedArea(const std::vector<ParameterPoint>& polygon)
{
    double twiceArea = 0.0;

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const ParameterPoint& current = polygon[index];
        const ParameterPoint& next = polygon[(index + 1) % polygon.size()];
        twiceArea += current.u * next.v - next.u * current.v;
    }

    return twiceArea * 0.5;
}

bool pointInsideTriangle(const ParameterPoint& point, const ParameterPoint& first, const ParameterPoint& second,
                         const ParameterPoint& third, double orientation, double tolerance)
{
    const double firstCross = orientation * cross2D(first, second, point);
    const double secondCross = orientation * cross2D(second, third, point);
    const double thirdCross = orientation * cross2D(third, first, point);
    return firstCross >= -tolerance && secondCross >= -tolerance && thirdCross >= -tolerance;
}

void removeDuplicateAndCollinearPoints(std::vector<ParameterPoint>& polygon, double tolerance)
{
    if (polygon.empty())
    {
        return;
    }

    std::vector<ParameterPoint> uniquePoints;
    uniquePoints.reserve(polygon.size());

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        if (uniquePoints.empty() || !samePoint(uniquePoints.back(), polygon[index], tolerance))
        {
            uniquePoints.push_back(polygon[index]);
        }
    }

    if (uniquePoints.size() > 1 && samePoint(uniquePoints.front(), uniquePoints.back(), tolerance))
    {
        uniquePoints.pop_back();
    }

    polygon.swap(uniquePoints);

    bool changed = true;

    while (changed && polygon.size() >= 3)
    {
        changed = false;

        for (std::size_t index = 0; index < polygon.size(); ++index)
        {
            const std::size_t previousIndex = index == 0 ? polygon.size() - 1 : index - 1;
            const std::size_t nextIndex = (index + 1) % polygon.size();
            const double area = std::fabs(cross2D(polygon[previousIndex], polygon[index], polygon[nextIndex]));

            if (area <= tolerance)
            {
                polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(index));
                changed = true;
                break;
            }
        }
    }
}

void discretizeOuterWire(const MyVoxel::Topology_Wire& wire, const MyVoxel::FaceMeshingOptions& options,
                         std::vector<ParameterPoint>& polygon)
{
    polygon.clear();
    polygon.reserve(wire.curveCount() * static_cast<std::size_t>(options.curveSegmentCount + 1));

    for (std::size_t curveIndex = 0; curveIndex < wire.curveCount(); ++curveIndex)
    {
        const MyVoxel::Topology_Curve& curve = wire.curve(curveIndex);
        const unsigned int segmentCount = curve.kind() == MyVoxel::CurveKind::Line ? 1U : options.curveSegmentCount;

        if (curveIndex == 0)
        {
            const MyMath::Vector3& startPoint = curve.startPoint();
            polygon.push_back(ParameterPoint(startPoint.x(), startPoint.y()));
        }

        for (unsigned int segmentIndex = 1; segmentIndex <= segmentCount; ++segmentIndex)
        {
            const MyMath::Vector3 point = curve.pointAt(static_cast<double>(segmentIndex) / static_cast<double>(segmentCount));
            polygon.push_back(ParameterPoint(point.x(), point.y()));
        }
    }

    removeDuplicateAndCollinearPoints(polygon, options.triangulationTolerance);
}

void triangulateSimplePolygon(const std::vector<ParameterPoint>& polygon, double tolerance, std::vector<std::uint32_t>& triangleIndices)
{
    MYVOXEL_ASSERT_MESSAGE(polygon.size() >= 3, "FaceMesher polygon requires at least three unique non-collinear points.");

    triangleIndices.clear();
    triangleIndices.reserve((polygon.size() - 2) * 3);

    const double signedArea = polygonSignedArea(polygon);
    MYVOXEL_ASSERT_MESSAGE(std::fabs(signedArea) > tolerance, "FaceMesher polygon area must be non-zero.");
    const double orientation = signedArea > 0.0 ? 1.0 : -1.0;

    std::vector<std::uint32_t> remaining;
    remaining.reserve(polygon.size());

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        MYVOXEL_ASSERT_MESSAGE(index <= static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)()),
                               "FaceMesher polygon vertex count exceeds uint32 index range.");
        remaining.push_back(static_cast<std::uint32_t>(index));
    }

    while (remaining.size() > 3)
    {
        bool clipped = false;

        for (std::size_t position = 0; position < remaining.size(); ++position)
        {
            const std::size_t previousPosition = position == 0 ? remaining.size() - 1 : position - 1;
            const std::size_t nextPosition = (position + 1) % remaining.size();
            const std::uint32_t previousIndex = remaining[previousPosition];
            const std::uint32_t currentIndex = remaining[position];
            const std::uint32_t nextIndex = remaining[nextPosition];
            const ParameterPoint& previous = polygon[previousIndex];
            const ParameterPoint& current = polygon[currentIndex];
            const ParameterPoint& next = polygon[nextIndex];

            if (orientation * cross2D(previous, current, next) <= tolerance)
            {
                continue;
            }

            bool containsOtherPoint = false;

            for (std::size_t testPosition = 0; testPosition < remaining.size(); ++testPosition)
            {
                const std::uint32_t testIndex = remaining[testPosition];

                if (testIndex == previousIndex || testIndex == currentIndex || testIndex == nextIndex)
                {
                    continue;
                }

                if (pointInsideTriangle(polygon[testIndex], previous, current, next, orientation, tolerance))
                {
                    containsOtherPoint = true;
                    break;
                }
            }

            if (containsOtherPoint)
            {
                continue;
            }

            triangleIndices.push_back(previousIndex);
            triangleIndices.push_back(currentIndex);
            triangleIndices.push_back(nextIndex);
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(position));
            clipped = true;
            break;
        }

        MYVOXEL_ASSERT_MESSAGE(clipped, "FaceMesher ear clipping failed; parameter polygon may be self-intersecting or numerically degenerate.");

        if (!clipped)
        {
            triangleIndices.clear();
            return;
        }
    }

    triangleIndices.push_back(remaining[0]);
    triangleIndices.push_back(remaining[1]);
    triangleIndices.push_back(remaining[2]);
}

MyVoxel::Display_Normal displayNormal(const MyMath::Vector3& normal)
{
    MYVOXEL_ASSERT_MESSAGE(normal.isFinite() && normal.lengthSquared() > 0.0, "FaceMesher surface normal must be finite and non-zero.");
    const MyMath::Vector3 unitNormal = normal.normalized(0.0);
    return MyVoxel::Display_Normal(unitNormal.x(), unitNormal.y(), unitNormal.z());
}

MyVoxel::Mesh buildPlaneFace(const MyVoxel::Topology_Face& face, const MyVoxel::Display_Color& color,
                             const MyVoxel::FaceMeshingOptions& options)
{
    std::vector<ParameterPoint> polygon;
    std::vector<std::uint32_t> triangleIndices;
    discretizeOuterWire(face.outerWire(), options, polygon);
    triangulateSimplePolygon(polygon, options.triangulationTolerance, triangleIndices);

    MyVoxel::Mesh mesh;
    mesh.reserve(polygon.size(), triangleIndices.size());

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const MyMath::Vector3 parameterPoint(polygon[index].u, polygon[index].v, 0.0);
        const MyMath::Vector3 point = face.pointAt(parameterPoint);
        const MyVoxel::Display_Normal normal = displayNormal(face.normalAt(parameterPoint));
        mesh.appendVertex(MyVoxel::MeshVertex(point.x(), point.y(), point.z(), normal));
    }

    for (std::size_t index = 0; index < triangleIndices.size(); index += 3)
    {
        mesh.appendTriangle(triangleIndices[index], triangleIndices[index + 1], triangleIndices[index + 2], color);
    }

    return mesh;
}

void validateOptions(const MyVoxel::FaceMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(options.curveSegmentCount >= 1, "FaceMesher curveSegmentCount must be at least one.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.triangulationTolerance) && options.triangulationTolerance >= 0.0,
                           "FaceMesher triangulationTolerance must be finite and non-negative.");
}

}

namespace MyVoxel
{

FaceMeshingOptions::FaceMeshingOptions()
    : curveSegmentCount(DefaultCurveSegmentCount)
    , triangulationTolerance(DefaultTriangulationTolerance)
{
}

/// 支持判断

bool FaceMesher::supports(const Topology_Face& face)
{
    return face.isValid() && face.surfaceKind() == SurfaceKind::Plane && face.innerWireCount() == 0;
}

bool FaceMesher::supports(const Face& face)
{
    return face.isValid() && supports(face.topology());
}

/// 局部网格构建

Mesh FaceMesher::build(const Topology_Face& face, const Display_Color& color, const FaceMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(face.isValid(), "FaceMesher requires a valid Topology_Face.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "FaceMesher requires a valid Display_Color.");
    MYVOXEL_ASSERT_MESSAGE(supports(face), "FaceMesher currently supports only Plane Face without inner wires.");
    validateOptions(options);

    Mesh result = buildPlaneFace(face, color, options);

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "FaceMesher produced an invalid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!result.isEmpty(), "Supported Face meshing must produce a non-empty Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!result.hasDegenerateTriangles(), "FaceMesher produced degenerate triangles.");
    MYVOXEL_ASSERT_MESSAGE(result.isRenderable(), "FaceMesher result must be directly consumable by Display_MeshResource.");
    return result;
}

Mesh FaceMesher::build(const Face& face, const Display_Color& color, const FaceMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(face.isValid(), "FaceMesher requires a valid Face.");
    return build(face.topology(), color, options);
}

/// 世界网格构建

Mesh FaceMesher::buildWorld(const Face& face, const Display_Color& color, const FaceMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(face.isValid(), "FaceMesher world build requires a valid Face.");
    Mesh result = build(face, color, options);
    result.transformInPlace(face.localToWorld());
    MYVOXEL_ASSERT_MESSAGE(result.isRenderable(), "FaceMesher world transform produced a non-renderable Mesh.");
    return result;
}

}