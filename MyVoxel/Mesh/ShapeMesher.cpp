#include "ShapeMesher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"
#include "MyVoxel/Mesh/Geometry_Mesh.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周、球面和回转轮廓离散统一使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆周对应的弧度值。
const unsigned int DefaultCircularSegmentCount = 64; // 默认完整圆周使用64段离散。
const unsigned int DefaultSphereStackCount = 32; // 默认球体从北极到南极使用32段离散。
const unsigned int DefaultProfileArcSegmentCount = 64; // 默认回转轮廓中的完整圆弧使用64段离散。

// 创建指定位置和显示法线的网格顶点。
MyVoxel::MeshVertex makeVertex(double x, double y, double z, double normalX, double normalY, double normalZ)
{
    return MyVoxel::MeshVertex(x, y, z, normalX, normalY, normalZ);
}

// 创建指定位置和显示法线的网格顶点。
MyVoxel::MeshVertex makeVertex(const MyMath::Vector3& point, const MyVoxel::Display_Normal& normal)
{
    return MyVoxel::MeshVertex(point.x(), point.y(), point.z(), normal);
}

// 返回三个有序点定义的单位三角形法线。
MyVoxel::Display_Normal triangleNormal(const MyMath::Vector3& point0, const MyMath::Vector3& point1, const MyMath::Vector3& point2)
{
    const MyMath::Vector3 cross = MyMath::Vector3::cross(point1 - point0, point2 - point0);
    MYVOXEL_ASSERT_MESSAGE(cross.isFinite() && cross.lengthSquared() > 0.0, "ShapeMesher cannot create a normal for a degenerate triangle.");
    const MyMath::Vector3 normal = cross.normalized(0.0);
    return MyVoxel::Display_Normal(normal.x(), normal.y(), normal.z());
}

// 使用三个有序点和统一颜色追加一个独立平面三角形。
void appendFlatTriangle(MyVoxel::Mesh& mesh, const MyMath::Vector3& point0, const MyMath::Vector3& point1,
                        const MyMath::Vector3& point2, const MyVoxel::Display_Color& color)
{
    const MyVoxel::Display_Normal normal = triangleNormal(point0, point1, point2);
    mesh.appendTriangle(makeVertex(point0, normal), makeVertex(point1, normal), makeVertex(point2, normal), color);
}

// 使用Geometry_Box的精确局部包围盒构建轴对齐盒体网格。
MyVoxel::Mesh buildBox(const MyVoxel::Geometry_Box& geometry, const MyVoxel::Display_Color& color)
{
    const MyVoxel::Bounds3& bounds = geometry.localBounds();
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();
    MyVoxel::Mesh mesh;
    mesh.reserve(24, 36); // 六个平面分别保存四个独立法线顶点，共24个顶点和12个三角形。

    mesh.appendQuad(makeVertex(minimum.x(), minimum.y(), minimum.z(), -1.0, 0.0, 0.0), makeVertex(minimum.x(), minimum.y(), maximum.z(), -1.0, 0.0, 0.0),
                    makeVertex(minimum.x(), maximum.y(), maximum.z(), -1.0, 0.0, 0.0), makeVertex(minimum.x(), maximum.y(), minimum.z(), -1.0, 0.0, 0.0), color);
    mesh.appendQuad(makeVertex(maximum.x(), minimum.y(), minimum.z(), 1.0, 0.0, 0.0), makeVertex(maximum.x(), maximum.y(), minimum.z(), 1.0, 0.0, 0.0),
                    makeVertex(maximum.x(), maximum.y(), maximum.z(), 1.0, 0.0, 0.0), makeVertex(maximum.x(), minimum.y(), maximum.z(), 1.0, 0.0, 0.0), color);
    mesh.appendQuad(makeVertex(minimum.x(), minimum.y(), minimum.z(), 0.0, -1.0, 0.0), makeVertex(maximum.x(), minimum.y(), minimum.z(), 0.0, -1.0, 0.0),
                    makeVertex(maximum.x(), minimum.y(), maximum.z(), 0.0, -1.0, 0.0), makeVertex(minimum.x(), minimum.y(), maximum.z(), 0.0, -1.0, 0.0), color);
    mesh.appendQuad(makeVertex(minimum.x(), maximum.y(), minimum.z(), 0.0, 1.0, 0.0), makeVertex(minimum.x(), maximum.y(), maximum.z(), 0.0, 1.0, 0.0),
                    makeVertex(maximum.x(), maximum.y(), maximum.z(), 0.0, 1.0, 0.0), makeVertex(maximum.x(), maximum.y(), minimum.z(), 0.0, 1.0, 0.0), color);
    mesh.appendQuad(makeVertex(minimum.x(), minimum.y(), minimum.z(), 0.0, 0.0, -1.0), makeVertex(minimum.x(), maximum.y(), minimum.z(), 0.0, 0.0, -1.0),
                    makeVertex(maximum.x(), maximum.y(), minimum.z(), 0.0, 0.0, -1.0), makeVertex(maximum.x(), minimum.y(), minimum.z(), 0.0, 0.0, -1.0), color);
    mesh.appendQuad(makeVertex(minimum.x(), minimum.y(), maximum.z(), 0.0, 0.0, 1.0), makeVertex(maximum.x(), minimum.y(), maximum.z(), 0.0, 0.0, 1.0),
                    makeVertex(maximum.x(), maximum.y(), maximum.z(), 0.0, 0.0, 1.0), makeVertex(minimum.x(), maximum.y(), maximum.z(), 0.0, 0.0, 1.0), color);
    return mesh;
}

// 使用Geometry_Sphere标准参数构建球体网格。
MyVoxel::Mesh buildSphere(const MyVoxel::Geometry_Sphere& geometry, const MyVoxel::Display_Color& color, const MyVoxel::ShapeMeshingOptions& options)
{
    const double radius = geometry.radius();
    const unsigned int segmentCount = options.circularSegmentCount;
    const unsigned int stackCount = options.sphereStackCount;
    const unsigned int ringCount = stackCount - 1;
    MyVoxel::Mesh mesh;
    const std::size_t vertexCount = static_cast<std::size_t>(ringCount) * segmentCount + 2;
    const std::size_t triangleCount = static_cast<std::size_t>(segmentCount) * 2 * (stackCount - 1);
    mesh.reserve(vertexCount, triangleCount * 3);
    const std::uint32_t topPoleIndex = mesh.appendVertex(makeVertex(0.0, 0.0, radius, 0.0, 0.0, 1.0));

    for (unsigned int stackIndex = 1; stackIndex < stackCount; ++stackIndex)
    {
        const double polarAngle = Pi * static_cast<double>(stackIndex) / static_cast<double>(stackCount);
        const double radialScale = std::sin(polarAngle);
        const double normalZ = std::cos(polarAngle);
        const double z = radius * normalZ;

        for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double angle = TwoPi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
            const double normalX = radialScale * std::cos(angle);
            const double normalY = radialScale * std::sin(angle);
            mesh.appendVertex(makeVertex(radius * normalX, radius * normalY, z, normalX, normalY, normalZ));
        }
    }

    const std::uint32_t bottomPoleIndex = mesh.appendVertex(makeVertex(0.0, 0.0, -radius, 0.0, 0.0, -1.0));

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        mesh.appendTriangle(topPoleIndex, 1 + segmentIndex, 1 + nextSegmentIndex, color);
    }

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

    const std::uint32_t bottomRingOffset = 1 + (ringCount - 1) * segmentCount;

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        mesh.appendTriangle(bottomPoleIndex, bottomRingOffset + nextSegmentIndex, bottomRingOffset + segmentIndex, color);
    }

    return mesh;
}

// 使用Geometry_Cylinder标准参数构建圆柱体网格。
MyVoxel::Mesh buildCylinder(const MyVoxel::Geometry_Cylinder& geometry, const MyVoxel::Display_Color& color, const MyVoxel::ShapeMeshingOptions& options)
{
    const double radius = geometry.radius();
    const double halfHeight = geometry.height() * 0.5;
    const double minimumZ = -halfHeight;
    const double maximumZ = halfHeight;
    const unsigned int segmentCount = options.circularSegmentCount;
    MyVoxel::Mesh mesh;
    mesh.reserve(static_cast<std::size_t>(segmentCount) * 10,
                 static_cast<std::size_t>(segmentCount) * 12); // 每段包含侧面四顶点和两个端面各三个顶点，共四个三角形。

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        const double angle0 = TwoPi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = TwoPi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);
        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);
        const double x0 = radius * cosine0;
        const double y0 = radius * sine0;
        const double x1 = radius * cosine1;
        const double y1 = radius * sine1;

        mesh.appendQuad(makeVertex(x0, y0, minimumZ, cosine0, sine0, 0.0), makeVertex(x1, y1, minimumZ, cosine1, sine1, 0.0),
                        makeVertex(x1, y1, maximumZ, cosine1, sine1, 0.0), makeVertex(x0, y0, maximumZ, cosine0, sine0, 0.0), color);
        mesh.appendTriangle(makeVertex(0.0, 0.0, minimumZ, 0.0, 0.0, -1.0), makeVertex(x1, y1, minimumZ, 0.0, 0.0, -1.0),
                            makeVertex(x0, y0, minimumZ, 0.0, 0.0, -1.0), color);
        mesh.appendTriangle(makeVertex(0.0, 0.0, maximumZ, 0.0, 0.0, 1.0), makeVertex(x0, y0, maximumZ, 0.0, 0.0, 1.0),
                            makeVertex(x1, y1, maximumZ, 0.0, 0.0, 1.0), color);
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

// 使用Geometry_ConeFrustum标准参数构建圆锥、倒圆锥或一般圆锥台网格。
MyVoxel::Mesh buildConeFrustum(const MyVoxel::Geometry_ConeFrustum& geometry, const MyVoxel::Display_Color& color,
                               const MyVoxel::ShapeMeshingOptions& options)
{
    const double bottomRadius = geometry.bottomRadius();
    const double topRadius = geometry.topRadius();
    const double height = geometry.height();
    const double minimumZ = -height * 0.5;
    const double maximumZ = height * 0.5;
    const unsigned int segmentCount = options.circularSegmentCount;
    const std::size_t sideTriangleCount = bottomRadius > 0.0 && topRadius > 0.0 ? static_cast<std::size_t>(segmentCount) * 2 : static_cast<std::size_t>(segmentCount);
    const std::size_t capTriangleCount = (bottomRadius > 0.0 ? static_cast<std::size_t>(segmentCount) : 0) +
                                         (topRadius > 0.0 ? static_cast<std::size_t>(segmentCount) : 0);
    const std::size_t triangleCount = sideTriangleCount + capTriangleCount;
    MyVoxel::Mesh mesh;
    mesh.reserve(triangleCount * 3, triangleCount * 3); // 圆锥台按独立侧面/端面顶点预留，避免构建过程反复扩容。

    for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const unsigned int nextSegmentIndex = (segmentIndex + 1) % segmentCount;
        const double angle0 = TwoPi * static_cast<double>(segmentIndex) / static_cast<double>(segmentCount);
        const double angle1 = TwoPi * static_cast<double>(nextSegmentIndex) / static_cast<double>(segmentCount);
        const double middleAngle = angle0 + Pi / static_cast<double>(segmentCount); // 使用当前分段中心角，末段跨越2π时仍保持正确方向。
        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);
        const MyMath::Vector3 normal0 = coneSideNormal(cosine0, sine0, bottomRadius, topRadius, height);
        const MyMath::Vector3 normal1 = coneSideNormal(cosine1, sine1, bottomRadius, topRadius, height);

        if (bottomRadius > 0.0 && topRadius > 0.0)
        {
            mesh.appendQuad(makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, normal0.x(), normal0.y(), normal0.z()),
                            makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, normal1.x(), normal1.y(), normal1.z()),
                            makeVertex(topRadius * cosine1, topRadius * sine1, maximumZ, normal1.x(), normal1.y(), normal1.z()),
                            makeVertex(topRadius * cosine0, topRadius * sine0, maximumZ, normal0.x(), normal0.y(), normal0.z()), color);
        }
        else if (bottomRadius > 0.0)
        {
            const MyMath::Vector3 apexNormal = coneSideNormal(std::cos(middleAngle), std::sin(middleAngle), bottomRadius, 0.0, height);
            mesh.appendTriangle(makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, normal0.x(), normal0.y(), normal0.z()),
                                makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, normal1.x(), normal1.y(), normal1.z()),
                                makeVertex(0.0, 0.0, maximumZ, apexNormal.x(), apexNormal.y(), apexNormal.z()), color);
        }
        else
        {
            const MyMath::Vector3 apexNormal = coneSideNormal(std::cos(middleAngle), std::sin(middleAngle), 0.0, topRadius, height);
            mesh.appendTriangle(makeVertex(0.0, 0.0, minimumZ, apexNormal.x(), apexNormal.y(), apexNormal.z()),
                                makeVertex(topRadius * cosine1, topRadius * sine1, maximumZ, normal1.x(), normal1.y(), normal1.z()),
                                makeVertex(topRadius * cosine0, topRadius * sine0, maximumZ, normal0.x(), normal0.y(), normal0.z()), color);
        }

        if (bottomRadius > 0.0)
        {
            mesh.appendTriangle(makeVertex(0.0, 0.0, minimumZ, 0.0, 0.0, -1.0), makeVertex(bottomRadius * cosine1, bottomRadius * sine1, minimumZ, 0.0, 0.0, -1.0),
                                makeVertex(bottomRadius * cosine0, bottomRadius * sine0, minimumZ, 0.0, 0.0, -1.0), color);
        }

        if (topRadius > 0.0)
        {
            mesh.appendTriangle(makeVertex(0.0, 0.0, maximumZ, 0.0, 0.0, 1.0), makeVertex(topRadius * cosine0, topRadius * sine0, maximumZ, 0.0, 0.0, 1.0),
                                makeVertex(topRadius * cosine1, topRadius * sine1, maximumZ, 0.0, 0.0, 1.0), color);
        }
    }

    return mesh;
}

// 返回指定轮廓圆弧根据完整圆离散数应使用的局部段数。
unsigned int profileArcSegments(const MyVoxel::Geometry_Arc& arc, const MyVoxel::ShapeMeshingOptions& options)
{
    const double requested = std::fabs(arc.sweepAngle()) / TwoPi * static_cast<double>(options.profileArcSegmentCount);
    return (std::max)(1U, static_cast<unsigned int>(std::ceil(requested)));
}

// 将闭合回转轮廓按曲线类型离散为连续点列，最后一点与第一点重合。
void buildRevolvedProfilePoints(const MyVoxel::Geometry_Revolved& geometry, const MyVoxel::ShapeMeshingOptions& options,
                                std::vector<MyMath::Vector3>& points)
{
    points.clear();
    MYVOXEL_ASSERT_MESSAGE(geometry.profileCurveCount() > 0, "Revolved meshing requires a non-empty profile.");
    points.push_back(geometry.profileCurve(0).startPoint());

    for (std::size_t curveIndex = 0; curveIndex < geometry.profileCurveCount(); ++curveIndex)
    {
        const MyVoxel::Geometry_Curve& curve = geometry.profileCurve(curveIndex);
        unsigned int segmentCount = 1;

        if (curve.kind() == MyVoxel::CurveKind::Arc)
        {
            segmentCount = profileArcSegments(static_cast<const MyVoxel::Geometry_Arc&>(curve), options);
        }

        for (unsigned int segmentIndex = 1; segmentIndex <= segmentCount; ++segmentIndex)
        {
            points.push_back(curve.pointAt(static_cast<double>(segmentIndex) / static_cast<double>(segmentCount)));
        }
    }

    MYVOXEL_ASSERT_MESSAGE(points.size() >= 2, "Revolved meshing profile discretization produced too few points.");
    MYVOXEL_ASSERT_MESSAGE(points.front().isEqualTo(points.back(), geometry.connectionTolerance()),
                           "Revolved meshing profile discretization must remain closed.");
}

// 将轮廓X坐标映射到非负回转半径，并清除轴附近的浮点舍入噪声。
double revolvedRadius(const MyVoxel::Geometry_Revolved& geometry, double profileX, double axisTolerance)
{
    double radius = geometry.radialSign() * profileX;

    if (radius < 0.0)
    {
        MYVOXEL_ASSERT_MESSAGE(radius >= -geometry.connectionTolerance(),
                               "Revolved profile point lies beyond the accepted rotation-axis tolerance.");
        radius = 0.0;
    }

    return radius <= axisTolerance ? 0.0 : radius;
}

// 返回指定半径、局部Z和圆周角对应的回转表面点。
MyMath::Vector3 revolvedPoint(double radius, double localZ, double angle)
{
    return MyMath::Vector3(radius * std::cos(angle), radius * std::sin(angle), localZ);
}

// 使用离散轮廓完整旋转构建Geometry_Revolved局部三角网格。
MyVoxel::Mesh buildRevolved(const MyVoxel::Geometry_Revolved& geometry, const MyVoxel::Display_Color& color,
                            const MyVoxel::ShapeMeshingOptions& options)
{
    std::vector<MyMath::Vector3> profilePoints;
    buildRevolvedProfilePoints(geometry, options, profilePoints);

    const double maximumRadius = (std::max)(std::fabs(geometry.localBounds().minimum().x()),
                                            std::fabs(geometry.localBounds().maximum().x()));
    const double axisTolerance = (std::max)(1.0, maximumRadius) *
                                 std::numeric_limits<double>::epsilon() * 64.0; // 仅吸收理论旋转轴点经三角函数计算产生的舍入误差。
    const bool counterClockwise = geometry.profileSignedArea() * geometry.radialSign() > 0.0;
    const unsigned int circularSegmentCount = options.circularSegmentCount;
    const std::size_t maximumTriangleCount = (profilePoints.size() - 1) * static_cast<std::size_t>(circularSegmentCount) * 2;
    MyVoxel::Mesh mesh;
    mesh.reserve(maximumTriangleCount * 3, maximumTriangleCount * 3); // 每个离散母线段和圆周段最多产生两个独立三角形。

    for (std::size_t profileIndex = 0; profileIndex + 1 < profilePoints.size(); ++profileIndex)
    {
        const MyMath::Vector3& profile0 = profilePoints[profileIndex];
        const MyMath::Vector3& profile1 = profilePoints[profileIndex + 1];
        const double radius0 = revolvedRadius(geometry, profile0.x(), axisTolerance);
        const double radius1 = revolvedRadius(geometry, profile1.x(), axisTolerance);
        const double z0 = profile0.y();
        const double z1 = profile1.y();

        if (radius0 == 0.0 && radius1 == 0.0)
        {
            continue;
        }

        for (unsigned int segmentIndex = 0; segmentIndex < circularSegmentCount; ++segmentIndex)
        {
            const unsigned int nextSegmentIndex = (segmentIndex + 1) % circularSegmentCount;
            const double angle0 = TwoPi * static_cast<double>(segmentIndex) / static_cast<double>(circularSegmentCount);
            const double angle1 = TwoPi * static_cast<double>(nextSegmentIndex) / static_cast<double>(circularSegmentCount);
            const MyMath::Vector3 point00 = revolvedPoint(radius0, z0, angle0);
            const MyMath::Vector3 point01 = revolvedPoint(radius0, z0, angle1);
            const MyMath::Vector3 point10 = revolvedPoint(radius1, z1, angle0);
            const MyMath::Vector3 point11 = revolvedPoint(radius1, z1, angle1);

            if (counterClockwise)
            {
                if (radius0 == 0.0)
                {
                    appendFlatTriangle(mesh, point00, point11, point10, color);
                }
                else if (radius1 == 0.0)
                {
                    appendFlatTriangle(mesh, point00, point01, point11, color);
                }
                else
                {
                    appendFlatTriangle(mesh, point00, point01, point11, color);
                    appendFlatTriangle(mesh, point00, point11, point10, color);
                }
            }
            else
            {
                if (radius0 == 0.0)
                {
                    appendFlatTriangle(mesh, point00, point10, point11, color);
                }
                else if (radius1 == 0.0)
                {
                    appendFlatTriangle(mesh, point00, point11, point01, color);
                }
                else
                {
                    appendFlatTriangle(mesh, point00, point11, point01, color);
                    appendFlatTriangle(mesh, point00, point10, point11, color);
                }
            }
        }
    }

    return mesh;
}

// 使用统一颜色重新建立Geometry_Mesh网格；已有单位法线可直接保留，否则按三角形生成平面法线。
MyVoxel::Mesh buildMeshGeometry(const MyVoxel::Geometry_Mesh& geometry, const MyVoxel::Display_Color& color)
{
    const MyVoxel::Mesh& source = geometry.mesh();
    const std::vector<MyVoxel::MeshVertex>& vertices = source.vertices();
    const std::vector<std::uint32_t>& indices = source.indices();
    MyVoxel::Mesh result;

    if (source.hasUnitNormals())
    {
        result.reserve(source.vertexCount(), source.indexCount());

        for (std::size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
        {
            result.appendVertex(vertices[vertexIndex]);
        }

        for (std::size_t triangleIndex = 0; triangleIndex < source.triangleCount(); ++triangleIndex)
        {
            const std::size_t indexOffset = triangleIndex * 3;
            result.appendTriangle(indices[indexOffset], indices[indexOffset + 1], indices[indexOffset + 2], color);
        }

        return result;
    }

    result.reserve(source.triangleCount() * 3, source.indexCount());

    for (std::size_t triangleIndex = 0; triangleIndex < source.triangleCount(); ++triangleIndex)
    {
        const std::size_t indexOffset = triangleIndex * 3;
        const MyVoxel::MeshVertex& vertex0 = vertices[indices[indexOffset]];
        const MyVoxel::MeshVertex& vertex1 = vertices[indices[indexOffset + 1]];
        const MyVoxel::MeshVertex& vertex2 = vertices[indices[indexOffset + 2]];
        appendFlatTriangle(result, MyMath::Vector3(vertex0.x, vertex0.y, vertex0.z),
                           MyMath::Vector3(vertex1.x, vertex1.y, vertex1.z),
                           MyMath::Vector3(vertex2.x, vertex2.y, vertex2.z), color);
    }

    return result;
}

// 验证通用Shape网格离散参数。
void validateOptions(const MyVoxel::ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(options.circularSegmentCount >= 3, "Shape circular meshing requires at least three segments.");
    MYVOXEL_ASSERT_MESSAGE(options.sphereStackCount >= 2, "Sphere meshing requires at least two stacks.");
    MYVOXEL_ASSERT_MESSAGE(options.profileArcSegmentCount >= 1, "Revolved profile arc meshing requires at least one full-circle segment.");
}

}

namespace MyVoxel
{

ShapeMeshingOptions::ShapeMeshingOptions()
    : circularSegmentCount(DefaultCircularSegmentCount)
    , sphereStackCount(DefaultSphereStackCount)
    , profileArcSegmentCount(DefaultProfileArcSegmentCount)
{
}

/// 支持判断

bool ShapeMesher::supports(const Geometry_Shape& geometry)
{
    switch (geometry.kind())
    {
    case ShapeKind::Box:
    case ShapeKind::Sphere:
    case ShapeKind::Cylinder:
    case ShapeKind::ConeFrustum:
    case ShapeKind::Revolved:
    case ShapeKind::Mesh:
        return true;

    case ShapeKind::Custom:
        return false;
    }

    return false;
}

bool ShapeMesher::supports(const Topology_Shape& topology)
{
    return topology.isValid() && supports(topology.geometry());
}

bool ShapeMesher::supports(const Shape& shape)
{
    return shape.isValid() && supports(shape.geometry());
}

/// 局部网格构建

Mesh ShapeMesher::build(const Geometry_Shape& geometry, const Display_Color& color, const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "ShapeMesher requires a valid Display_Color.");
    MYVOXEL_ASSERT_MESSAGE(supports(geometry), "ShapeMesher does not support the specified ShapeKind.");
    validateOptions(options);
    Mesh result;

    switch (geometry.kind())
    {
    case ShapeKind::Box:
        result = buildBox(static_cast<const Geometry_Box&>(geometry), color);
        break;

    case ShapeKind::Sphere:
        result = buildSphere(static_cast<const Geometry_Sphere&>(geometry), color, options);
        break;

    case ShapeKind::Cylinder:
        result = buildCylinder(static_cast<const Geometry_Cylinder&>(geometry), color, options);
        break;

    case ShapeKind::ConeFrustum:
        result = buildConeFrustum(static_cast<const Geometry_ConeFrustum&>(geometry), color, options);
        break;

    case ShapeKind::Revolved:
        result = buildRevolved(static_cast<const Geometry_Revolved&>(geometry), color, options);
        break;

    case ShapeKind::Mesh:
        result = buildMeshGeometry(static_cast<const Geometry_Mesh&>(geometry), color);
        break;

    case ShapeKind::Custom:
        MYVOXEL_ASSERT_MESSAGE(false, "Custom Shape geometry does not have a standard meshing implementation.");
        return Mesh();
    }

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "ShapeMesher produced an invalid Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!result.isEmpty(), "Supported Shape meshing must produce a non-empty Mesh.");
    MYVOXEL_ASSERT_MESSAGE(!result.hasDegenerateTriangles(), "ShapeMesher produced degenerate triangles.");
    MYVOXEL_ASSERT_MESSAGE(result.isRenderable(), "ShapeMesher must produce complete unit normals and triangle colors.");
    return result;
}

Mesh ShapeMesher::build(const Topology_Shape& topology, const Display_Color& color, const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "ShapeMesher requires a valid Topology_Shape.");
    return build(topology.geometry(), color, options);
}

Mesh ShapeMesher::build(const Shape& shape, const Display_Color& color, const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeMesher requires a valid Shape.");
    return build(shape.geometry(), color, options);
}

/// 世界网格构建

Mesh ShapeMesher::buildWorld(const Shape& shape, const Display_Color& color, const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeMesher world build requires a valid Shape.");
    Mesh result = build(shape, color, options);
    result.transformInPlace(shape.localToWorld());
    MYVOXEL_ASSERT_MESSAGE(result.isRenderable(), "ShapeMesher world transform produced a non-renderable Mesh.");
    return result;
}

}