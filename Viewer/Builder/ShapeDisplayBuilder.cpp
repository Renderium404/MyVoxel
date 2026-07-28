#include "ShapeDisplayBuilder.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Shape/BoxGeometry.h"
#include "MyVoxel/Shape/CylinderGeometry.h"
#include "MyVoxel/Shape/ShapeBounds.h"
#include "MyVoxel/Shape/ShapeGeometry.h"

namespace
{

const double Pi = 3.14159265358979323846; // 圆周率，用于圆柱表面离散。

// 创建点变换对应的法向逆转置矩阵。
MyMath::Matrix4 createNormalTransform(const MyMath::Matrix4& transform)
{
    assert(transform.isAffine());

    MyMath::Matrix4 inverse;
    const bool inverted = transform.inverted(inverse);

    assert(inverted);
    return inverse.transposed();
}

// 将局部法向变换到世界坐标并归一化。
MyMath::Vector3 transformNormal(const MyMath::Matrix4& normalTransform, const MyMath::Vector3& normal)
{
    assert(normal.isVector());

    const MyMath::Vector3 result = normalTransform.transformVector(normal).normalized();

    assert(result.isUnit());
    return result;
}

// 向显示网格追加一个经过Shape变换的顶点。
std::uint32_t appendVertex(MyVoxelViewer::DisplayMesh& mesh, const MyMath::Matrix4& pointTransform, const MyMath::Matrix4& normalTransform, const MyMath::Vector3& point, const MyMath::Vector3& normal, const MyVoxelViewer::DisplayColor& color)
{
    assert(mesh.vertices().size() < static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)()));

    const MyMath::Vector3 worldPoint = pointTransform.transformPoint(point);
    const MyMath::Vector3 worldNormal = transformNormal(normalTransform, normal);
    const std::uint32_t index = static_cast<std::uint32_t>(mesh.vertices().size());

    mesh.vertices().push_back(MyVoxelViewer::DisplayVertex(worldPoint.x(), worldPoint.y(), worldPoint.z(), worldNormal.x(), worldNormal.y(), worldNormal.z(), color));
    return index;
}

// 向显示网格追加一个三角形。
void appendTriangle(MyVoxelViewer::DisplayMesh& mesh, std::uint32_t index0, std::uint32_t index1, std::uint32_t index2)
{
    mesh.indices().push_back(index0);
    mesh.indices().push_back(index1);
    mesh.indices().push_back(index2);
}

// 向显示网格追加一个按外侧观察逆时针排列的四边形。
void appendQuad(MyVoxelViewer::DisplayMesh& mesh, const MyMath::Matrix4& pointTransform, const MyMath::Matrix4& normalTransform, const MyMath::Vector3& point0, const MyMath::Vector3& point1, const MyMath::Vector3& point2, const MyMath::Vector3& point3, const MyMath::Vector3& normal, const MyVoxelViewer::DisplayColor& color)
{
    const std::uint32_t index0 = appendVertex(mesh, pointTransform, normalTransform, point0, normal, color);
    const std::uint32_t index1 = appendVertex(mesh, pointTransform, normalTransform, point1, normal, color);
    const std::uint32_t index2 = appendVertex(mesh, pointTransform, normalTransform, point2, normal, color);
    const std::uint32_t index3 = appendVertex(mesh, pointTransform, normalTransform, point3, normal, color);

    appendTriangle(mesh, index0, index1, index2);
    appendTriangle(mesh, index0, index2, index3);
}

// 创建标准盒体的显示网格。
MyVoxelViewer::DisplayMesh buildBox(const MyVoxel::Shape& shape, const MyVoxel::BoxGeometry& geometry, const MyVoxelViewer::DisplayColor& color)
{
    const MyVoxel::ShapeBounds bounds = geometry.localBounds();
    const MyMath::Matrix4& pointTransform = shape.transform();
    const MyMath::Matrix4 normalTransform = createNormalTransform(pointTransform);

    const MyMath::Vector3 point000(bounds.minimumX, bounds.minimumY, bounds.minimumZ);
    const MyMath::Vector3 point100(bounds.maximumX, bounds.minimumY, bounds.minimumZ);
    const MyMath::Vector3 point110(bounds.maximumX, bounds.maximumY, bounds.minimumZ);
    const MyMath::Vector3 point010(bounds.minimumX, bounds.maximumY, bounds.minimumZ);
    const MyMath::Vector3 point001(bounds.minimumX, bounds.minimumY, bounds.maximumZ);
    const MyMath::Vector3 point101(bounds.maximumX, bounds.minimumY, bounds.maximumZ);
    const MyMath::Vector3 point111(bounds.maximumX, bounds.maximumY, bounds.maximumZ);
    const MyMath::Vector3 point011(bounds.minimumX, bounds.maximumY, bounds.maximumZ);

    MyVoxelViewer::DisplayMesh mesh;
    mesh.reserve(24, 36);

    appendQuad(mesh, pointTransform, normalTransform, point000, point010, point110, point100, -MyMath::Vector3::unitZ(), color);
    appendQuad(mesh, pointTransform, normalTransform, point001, point101, point111, point011, MyMath::Vector3::unitZ(), color);
    appendQuad(mesh, pointTransform, normalTransform, point000, point001, point011, point010, -MyMath::Vector3::unitX(), color);
    appendQuad(mesh, pointTransform, normalTransform, point100, point110, point111, point101, MyMath::Vector3::unitX(), color);
    appendQuad(mesh, pointTransform, normalTransform, point000, point100, point101, point001, -MyMath::Vector3::unitY(), color);
    appendQuad(mesh, pointTransform, normalTransform, point010, point011, point111, point110, MyMath::Vector3::unitY(), color);

    return mesh;
}

// 创建标准圆柱体的显示网格。
MyVoxelViewer::DisplayMesh buildCylinder(const MyVoxel::Shape& shape, const MyVoxel::CylinderGeometry& geometry, const MyVoxelViewer::DisplayColor& color, int radialSegmentCount)
{
    assert(radialSegmentCount >= 3);

    const MyMath::Matrix4& pointTransform = shape.transform();
    const MyMath::Matrix4 normalTransform = createNormalTransform(pointTransform);
    const double angleStep = 2.0 * Pi / static_cast<double>(radialSegmentCount);
    const MyMath::Vector3 bottomCenter(geometry.centerX(), geometry.centerY(), geometry.minimumZ());
    const MyMath::Vector3 topCenter(geometry.centerX(), geometry.centerY(), geometry.maximumZ());

    MyVoxelViewer::DisplayMesh mesh;
    mesh.reserve(static_cast<std::size_t>(radialSegmentCount) * 10, static_cast<std::size_t>(radialSegmentCount) * 12);

    for (int i = 0; i < radialSegmentCount; ++i)
    {
        const double angle0 = static_cast<double>(i) * angleStep;
        const double angle1 = static_cast<double>(i + 1) * angleStep;
        const double cosine0 = std::cos(angle0);
        const double sine0 = std::sin(angle0);
        const double cosine1 = std::cos(angle1);
        const double sine1 = std::sin(angle1);

        const MyMath::Vector3 normal0(cosine0, sine0, 0.0);
        const MyMath::Vector3 normal1(cosine1, sine1, 0.0);

        const MyMath::Vector3 bottom0(geometry.centerX() + geometry.radius() * cosine0, geometry.centerY() + geometry.radius() * sine0, geometry.minimumZ());
        const MyMath::Vector3 bottom1(geometry.centerX() + geometry.radius() * cosine1, geometry.centerY() + geometry.radius() * sine1, geometry.minimumZ());
        const MyMath::Vector3 top0(geometry.centerX() + geometry.radius() * cosine0, geometry.centerY() + geometry.radius() * sine0, geometry.maximumZ());
        const MyMath::Vector3 top1(geometry.centerX() + geometry.radius() * cosine1, geometry.centerY() + geometry.radius() * sine1, geometry.maximumZ());

        const std::uint32_t sideIndex0 = appendVertex(mesh, pointTransform, normalTransform, bottom0, normal0, color);
        const std::uint32_t sideIndex1 = appendVertex(mesh, pointTransform, normalTransform, bottom1, normal1, color);
        const std::uint32_t sideIndex2 = appendVertex(mesh, pointTransform, normalTransform, top1, normal1, color);
        const std::uint32_t sideIndex3 = appendVertex(mesh, pointTransform, normalTransform, top0, normal0, color);

        appendTriangle(mesh, sideIndex0, sideIndex1, sideIndex2);
        appendTriangle(mesh, sideIndex0, sideIndex2, sideIndex3);

        const std::uint32_t bottomCenterIndex = appendVertex(mesh, pointTransform, normalTransform, bottomCenter, -MyMath::Vector3::unitZ(), color);
        const std::uint32_t bottomIndex1 = appendVertex(mesh, pointTransform, normalTransform, bottom1, -MyMath::Vector3::unitZ(), color);
        const std::uint32_t bottomIndex0 = appendVertex(mesh, pointTransform, normalTransform, bottom0, -MyMath::Vector3::unitZ(), color);

        appendTriangle(mesh, bottomCenterIndex, bottomIndex1, bottomIndex0);

        const std::uint32_t topCenterIndex = appendVertex(mesh, pointTransform, normalTransform, topCenter, MyMath::Vector3::unitZ(), color);
        const std::uint32_t topIndex0 = appendVertex(mesh, pointTransform, normalTransform, top0, MyMath::Vector3::unitZ(), color);
        const std::uint32_t topIndex1 = appendVertex(mesh, pointTransform, normalTransform, top1, MyMath::Vector3::unitZ(), color);

        appendTriangle(mesh, topCenterIndex, topIndex0, topIndex1);
    }

    return mesh;
}

}

namespace MyVoxelViewer
{

DisplayMesh ShapeDisplayBuilder::build(const MyVoxel::Shape& shape, const DisplayColor& color, int radialSegmentCount)
{
    assert(shape.transform().isAffine());
    assert(radialSegmentCount >= 3);

    const MyVoxel::ShapeGeometry& geometry = shape.geometry();

    if (const MyVoxel::BoxGeometry* box = dynamic_cast<const MyVoxel::BoxGeometry*>(&geometry))
    {
        return buildBox(shape, *box, color);
    }

    if (const MyVoxel::CylinderGeometry* cylinder = dynamic_cast<const MyVoxel::CylinderGeometry*>(&geometry))
    {
        return buildCylinder(shape, *cylinder, color, radialSegmentCount);
    }

    assert(false);
    return DisplayMesh();
}

}