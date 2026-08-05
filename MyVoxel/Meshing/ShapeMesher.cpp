#include "ShapeMesher.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/BaseShape/ConeFrustumGeometry.h"

namespace
{

const double Pi = 3.14159265358979323846; // 圆周离散和球面离散使用的圆周率。
const unsigned int DefaultCircularSegmentCount = 64; // 默认圆周使用64段离散。
const unsigned int DefaultSphereStackCount = 32; // 默认球体从北极到南极使用32段离散。

// 创建指定位置和法线的网格顶点。
MyVoxel::Geometry::MeshVertex makeVertex(double x, double y, double z, double normalX, double normalY, double normalZ)
{
    return MyVoxel::Geometry::MeshVertex(x, y, z, normalX, normalY, normalZ);
}

// 返回两个数值的中点。
double midpoint(double first, double second)
{
    return (first + second) * 0.5;
}

// 使用Shape局部包围盒构建轴对齐盒体网格。
MyVoxel::Geometry::Mesh buildBox(const MyVoxel::Geometry::Shape& shape, const MyVoxel::Geometry::MeshColor& color)
{
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    const double minimumX = minimum.x();
    const double minimumY = minimum.y();
    const double minimumZ = minimum.z();
    const double maximumX = maximum.x();
    const double maximumY = maximum.y();
    const double maximumZ = maximum.z();

    MyVoxel::Geometry::Mesh mesh;
    mesh.reserve(24, 36); // 盒体六个面分别保存四个独立法线顶点，共24个顶点和12个三角形。

    /// Negative X

    mesh.appendQuad(
        makeVertex(minimumX, minimumY, minimumZ, -1.0, 0.0, 0.0),
        makeVertex(minimumX, minimumY, maximumZ, -1.0, 0.0, 0.0),
        makeVertex(minimumX, maximumY, maximumZ, -1.0, 0.0, 0.0),
        makeVertex(minimumX, maximumY, minimumZ, -1.0, 0.0, 0.0),
        color);

    /// Positive X

    mesh.appendQuad(
        makeVertex(maximumX, minimumY, minimumZ, 1.0, 0.0, 0.0),
        makeVertex(maximumX, maximumY, minimumZ, 1.0, 0.0, 0.0),
        makeVertex(maximumX, maximumY, maximumZ, 1.0, 0.0, 0.0),
        makeVertex(maximumX, minimumY, maximumZ, 1.0, 0.0, 0.0),
        color);

    /// Negative Y

    mesh.appendQuad(
        makeVertex(minimumX, minimumY, minimumZ, 0.0, -1.0, 0.0),
        makeVertex(maximumX, minimumY, minimumZ, 0.0, -1.0, 0.0),
        makeVertex(maximumX, minimumY, maximumZ, 0.0, -1.0, 0.0),
        makeVertex(minimumX, minimumY, maximumZ, 0.0, -1.0, 0.0),
        color);

    /// Positive Y

    mesh.appendQuad(
        makeVertex(minimumX, maximumY, minimumZ, 0.0, 1.0, 0.0),
        makeVertex(minimumX, maximumY, maximumZ, 0.0, 1.0, 0.0),
        makeVertex(maximumX, maximumY, maximumZ, 0.0, 1.0, 0.0),
        makeVertex(maximumX, maximumY, minimumZ, 0.0, 1.0, 0.0),
        color);

    /// Negative Z

    mesh.appendQuad(
        makeVertex(minimumX, minimumY, minimumZ, 0.0, 0.0, -1.0),
        makeVertex(minimumX, maximumY, minimumZ, 0.0, 0.0, -1.0),
        makeVertex(maximumX, maximumY, minimumZ, 0.0, 0.0, -1.0),
        makeVertex(maximumX, minimumY, minimumZ, 0.0, 0.0, -1.0),
        color);

    /// Positive Z

    mesh.appendQuad(
        makeVertex(minimumX, minimumY, maximumZ, 0.0, 0.0, 1.0),
        makeVertex(maximumX, minimumY, maximumZ, 0.0, 0.0, 1.0),
        makeVertex(maximumX, maximumY, maximumZ, 0.0, 0.0, 1.0),
        makeVertex(minimumX, maximumY, maximumZ, 0.0, 0.0, 1.0),
        color);

    return mesh;
}

// 使用Shape局部包围盒构建球体网格。
MyVoxel::Geometry::Mesh buildSphere(const MyVoxel::Geometry::Shape& shape,
                                   const MyVoxel::Geometry::MeshColor& color,
                                   const MyVoxel::Meshing::ShapeMeshingOptions& options)
{
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    const double centerX = midpoint(minimum.x(), maximum.x());
    const double centerY = midpoint(minimum.y(), maximum.y());
    const double centerZ = midpoint(minimum.z(), maximum.z());
    const double radius = (maximum.x() - minimum.x()) * 0.5;

    const unsigned int segmentCount = options.circularSegmentCount;
    const unsigned int stackCount = options.sphereStackCount;
    const unsigned int ringCount = stackCount - 1;

    MyVoxel::Geometry::Mesh mesh;

    const std::size_t vertexCount = static_cast<std::size_t>(ringCount) * segmentCount + 2;
    const std::size_t triangleCount = static_cast<std::size_t>(segmentCount) * 2 * (stackCount - 1);

    mesh.reserve(vertexCount, triangleCount * 3);

    const std::uint32_t topPoleIndex =
        mesh.appendVertex(makeVertex(centerX, centerY, centerZ + radius, 0.0, 0.0, 1.0));

    for (unsigned int stackIndex = 1; stackIndex < stackCount; ++stackIndex)
    {
        const double polarAngle = Pi * static_cast<double>(stackIndex) / static_cast<double>(stackCount);
        const double radialScale = std::sin(polarAngle);
        const double normalZ = std::cos(polarAngle);
        const double z = centerZ + radius * normalZ;

        for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double angle = 2.0 * Pi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
            const double normalX = radialScale * std::cos(angle);
            const double normalY = radialScale * std::sin(angle);

            mesh.appendVertex(
                makeVertex(
                    centerX + radius * normalX,
                    centerY + radius * normalY,
                    z,
                    normalX,
                    normalY,
                    normalZ));
        }
    }

    const std::uint32_t bottomPoleIndex =
        mesh.appendVertex(makeVertex(centerX, centerY, centerZ - radius, 0.0, 0.0, -1.0));

    /// 北极三角扇

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        const std::uint32_t currentIndex = 1 + segmentIndex;
        const std::uint32_t nextIndex = 1 + nextSegmentIndex;

        mesh.appendTriangle(topPoleIndex, currentIndex, nextIndex, color);
    }

    /// 中间纬向网格

    for (unsigned int ringIndex = 0; ringIndex + 1 < ringCount; ++ringIndex)
    {
        const std::uint32_t upperRingOffset = 1 + ringIndex * segmentCount;
        const std::uint32_t lowerRingOffset = upperRingOffset + segmentCount;

        for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;

            const std::uint32_t upperCurrent = upperRingOffset + segmentIndex;
            const std::uint32_t upperNext = upperRingOffset + nextSegmentIndex;
            const std::uint32_t lowerCurrent = lowerRingOffset + segmentIndex;
            const std::uint32_t lowerNext = lowerRingOffset + nextSegmentIndex;

            mesh.appendTriangle(upperCurrent, lowerCurrent, lowerNext, color);
            mesh.appendTriangle(upperCurrent, lowerNext, upperNext, color);
        }
    }

    /// 南极三角扇

    const std::uint32_t bottomRingOffset = 1 + (ringCount - 1) * segmentCount;

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        const std::uint32_t currentIndex = bottomRingOffset + segmentIndex;
        const std::uint32_t nextIndex = bottomRingOffset + nextSegmentIndex;

        mesh.appendTriangle(bottomPoleIndex, nextIndex, currentIndex, color);
    }

    return mesh;
}

// 使用Shape局部包围盒构建圆柱体网格。
MyVoxel::Geometry::Mesh buildCylinder(const MyVoxel::Geometry::Shape& shape,
                                     const MyVoxel::Geometry::MeshColor& color,
                                     const MyVoxel::Meshing::ShapeMeshingOptions& options)
{
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    const double centerX = midpoint(minimum.x(), maximum.x());
    const double centerY = midpoint(minimum.y(), maximum.y());
    const double minimumZ = minimum.z();
    const double maximumZ = maximum.z();
    const double radius = (maximum.x() - minimum.x()) * 0.5;

    const unsigned int segmentCount = options.circularSegmentCount;

    MyVoxel::Geometry::Mesh mesh;
    mesh.reserve(static_cast<std::size_t>(segmentCount) * 10,
                 static_cast<std::size_t>(segmentCount) * 12); // 每段包含侧面四顶点以及两个端面各三个顶点，共四个三角形。

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;

        const double angle0 = 2.0 * Pi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = 2.0 * Pi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);

        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);

        const double x0 = centerX + radius * cosine0;
        const double y0 = centerY + radius * sine0;
        const double x1 = centerX + radius * cosine1;
        const double y1 = centerY + radius * sine1;

        /// 侧面

        mesh.appendQuad(
            makeVertex(x0, y0, minimumZ, cosine0, sine0, 0.0),
            makeVertex(x1, y1, minimumZ, cosine1, sine1, 0.0),
            makeVertex(x1, y1, maximumZ, cosine1, sine1, 0.0),
            makeVertex(x0, y0, maximumZ, cosine0, sine0, 0.0),
            color);

        /// 底面

        mesh.appendTriangle(
            makeVertex(centerX, centerY, minimumZ, 0.0, 0.0, -1.0),
            makeVertex(x1, y1, minimumZ, 0.0, 0.0, -1.0),
            makeVertex(x0, y0, minimumZ, 0.0, 0.0, -1.0),
            color);

        /// 顶面

        mesh.appendTriangle(
            makeVertex(centerX, centerY, maximumZ, 0.0, 0.0, 1.0),
            makeVertex(x0, y0, maximumZ, 0.0, 0.0, 1.0),
            makeVertex(x1, y1, maximumZ, 0.0, 0.0, 1.0),
            color);
    }

    return mesh;
}

// 返回圆锥台指定圆周方向的单位侧面法线。
MyMath::Vector3 coneSideNormal(double cosine, double sine, double bottomRadius, double topRadius, double height)
{
    const double normalZ = (bottomRadius - topRadius) / height;
    const double length = std::sqrt(1.0 + normalZ * normalZ);

    return MyMath::Vector3(cosine / length, sine / length, normalZ / length);
}

// 构建底半径大于零、顶半径为零的圆锥侧面。
void appendConeSides(MyVoxel::Geometry::Mesh& mesh,
                     double bottomRadius,
                     double minimumZ,
                     double maximumZ,
                     const MyVoxel::Geometry::MeshColor& color,
                     unsigned int segmentCount)
{
    const double height = maximumZ - minimumZ;

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;

        const double angle0 = 2.0 * Pi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = 2.0 * Pi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);
        const double middleAngle = (angle0 + angle1) * 0.5;

        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);

        const MyMath::Vector3 normal0 = coneSideNormal(cosine0, sine0, bottomRadius, 0.0, height);
        const MyMath::Vector3 normal1 = coneSideNormal(cosine1, sine1, bottomRadius, 0.0, height);
        const MyMath::Vector3 apexNormal = coneSideNormal(std::cos(middleAngle), std::sin(middleAngle), bottomRadius, 0.0, height);

        mesh.appendTriangle(
            makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, normal0.x(), normal0.y(), normal0.z()),
            makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, normal1.x(), normal1.y(), normal1.z()),
            makeVertex(0.0, 0.0, maximumZ, apexNormal.x(), apexNormal.y(), apexNormal.z()),
            color);
    }
}

// 构建底半径和顶半径都大于零的圆锥台侧面。
void appendFrustumSides(MyVoxel::Geometry::Mesh& mesh,
                        double bottomRadius,
                        double topRadius,
                        double minimumZ,
                        double maximumZ,
                        const MyVoxel::Geometry::MeshColor& color,
                        unsigned int segmentCount)
{
    const double height = maximumZ - minimumZ;

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;

        const double angle0 = 2.0 * Pi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = 2.0 * Pi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);

        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);

        const MyMath::Vector3 normal0 = coneSideNormal(cosine0, sine0, bottomRadius, topRadius, height);
        const MyMath::Vector3 normal1 = coneSideNormal(cosine1, sine1, bottomRadius, topRadius, height);

        mesh.appendQuad(
            makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, normal0.x(), normal0.y(), normal0.z()),
            makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, normal1.x(), normal1.y(), normal1.z()),
            makeVertex(topRadius * cosine1, topRadius * sine1, maximumZ, normal1.x(), normal1.y(), normal1.z()),
            makeVertex(topRadius * cosine0, topRadius * sine0, maximumZ, normal0.x(), normal0.y(), normal0.z()),
            color);
    }
}

// 为圆锥或圆锥台追加底面和顶面。
void appendConeCaps(MyVoxel::Geometry::Mesh& mesh,
                    double bottomRadius,
                    double topRadius,
                    double minimumZ,
                    double maximumZ,
                    const MyVoxel::Geometry::MeshColor& color,
                    unsigned int segmentCount)
{
    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;

        const double angle0 = 2.0 * Pi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = 2.0 * Pi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);

        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);

        if (bottomRadius > 0.0)
        {
            mesh.appendTriangle(
                makeVertex(0.0, 0.0, minimumZ, 0.0, 0.0, -1.0),
                makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, 0.0, 0.0, -1.0),
                makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, 0.0, 0.0, -1.0),
                color);
        }

        if (topRadius > 0.0)
        {
            mesh.appendTriangle(
                makeVertex(0.0, 0.0, maximumZ, 0.0, 0.0, 1.0),
                makeVertex(topRadius * cosine0, topRadius * sine0, maximumZ, 0.0, 0.0, 1.0),
                makeVertex(topRadius * cosine1, topRadius * sine1, maximumZ, 0.0, 0.0, 1.0),
                color);
        }
    }
}

// 构建圆锥或圆锥台网格。
MyVoxel::Geometry::Mesh buildConeFrustum(const MyVoxel::Geometry::Shape& shape,
                                        const MyVoxel::Geometry::MeshColor& color,
                                        const MyVoxel::Meshing::ShapeMeshingOptions& options)
{
    const MyVoxel::Geometry::ConeFrustumGeometry& geometry =
        static_cast<const MyVoxel::Geometry::ConeFrustumGeometry&>(shape.geometry());

    const double bottomRadius = geometry.bottomRadius();
    const double topRadius = geometry.topRadius();
    const double height = geometry.height();
    const double minimumZ = -height * 0.5;
    const double maximumZ = height * 0.5;
    const unsigned int segmentCount = options.circularSegmentCount;

    MyVoxel::Geometry::Mesh mesh;

    const std::size_t sideTriangleCount =
        topRadius == 0.0 ?
        static_cast<std::size_t>(segmentCount) :
        static_cast<std::size_t>(segmentCount) * 2;

    const std::size_t capTriangleCount =
        static_cast<std::size_t>(segmentCount) +
        (topRadius > 0.0 ? static_cast<std::size_t>(segmentCount) : 0);

    mesh.reserve(
        (sideTriangleCount + capTriangleCount) * 3,
        (sideTriangleCount + capTriangleCount) * 3);

    if (topRadius == 0.0)
    {
        appendConeSides(mesh, bottomRadius, minimumZ, maximumZ, color, segmentCount);
    }
    else
    {
        appendFrustumSides(mesh, bottomRadius, topRadius, minimumZ, maximumZ, color, segmentCount);
    }

    appendConeCaps(mesh, bottomRadius, topRadius, minimumZ, maximumZ, color, segmentCount);
    return mesh;
}

}

namespace MyVoxel
{
namespace Meshing
{

ShapeMeshingOptions::ShapeMeshingOptions()
    : circularSegmentCount(DefaultCircularSegmentCount)
    , sphereStackCount(DefaultSphereStackCount)
{
}

bool ShapeMesher::supports(const Geometry::Shape& shape)
{
    if (!shape.isValid())
    {
        return false;
    }

    switch (shape.kind())
    {
    case Geometry::ShapeKind::Box:
    case Geometry::ShapeKind::Sphere:
    case Geometry::ShapeKind::Cylinder:
    case Geometry::ShapeKind::ConeFrustum:
        return true;

    case Geometry::ShapeKind::Custom:
        return false;
    }

    return false;
}

Geometry::Mesh ShapeMesher::build(const Geometry::Shape& shape,
                                  const Geometry::MeshColor& color,
                                  const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Shape meshing requires a valid Geometry::Shape.");
    MYVOXEL_ASSERT_MESSAGE(supports(shape), "Shape meshing does not support the specified ShapeKind.");
    MYVOXEL_ASSERT_MESSAGE(options.circularSegmentCount >= 3, "Shape circular meshing requires at least three segments.");
    MYVOXEL_ASSERT_MESSAGE(options.sphereStackCount >= 2, "Sphere meshing requires at least two stacks.");

    Geometry::Mesh result;

    switch (shape.kind())
    {
    case Geometry::ShapeKind::Box:
        result = buildBox(shape, color);
        break;

    case Geometry::ShapeKind::Sphere:
        result = buildSphere(shape, color, options);
        break;

    case Geometry::ShapeKind::Cylinder:
        result = buildCylinder(shape, color, options);
        break;

    case Geometry::ShapeKind::ConeFrustum:
        result = buildConeFrustum(shape, color, options);
        break;

    case Geometry::ShapeKind::Custom:
        MYVOXEL_ASSERT_MESSAGE(false, "Custom Shape geometry does not have a standard meshing implementation.");
        return Geometry::Mesh();
    }

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Shape meshing produced an invalid triangle mesh.");
    MYVOXEL_ASSERT_MESSAGE(!result.isEmpty(), "Supported Shape meshing must produce a non-empty triangle mesh.");
    return result;
}
}
}