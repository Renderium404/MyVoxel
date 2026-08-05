#ifndef MYVOXEL_GEOMETRY_MESHQUERY_H
#define MYVOXEL_GEOMETRY_MESHQUERY_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"

#include "Mesh.h"
#include "MyVoxel/Geometry/ShapeRelation.h"

namespace MyVoxel
{
namespace Geometry
{

// 保存通用封闭三角网格的不可变连续几何查询数据。
//
// 查询器只依赖Mesh的位置和索引，不读取顶点法线或三角形颜色。
// 构造时将索引网格展开为独立三角形数组，并建立扁平二叉包围体层次结构。
// containsPoint使用与三角形方向无关的奇偶射线规则，调用者负责保证网格表示封闭表面。
class MeshQuery
{
public:
    // 根据几何有效且非空的三角网格建立查询器。
    explicit MeshQuery(const Mesh& mesh);

    /// 状态判断
    // 判断查询器是否已经根据几何有效非空网格建立。
    bool isValid() const;

    /// 查询数据
    // 返回实际三角形使用顶点形成的局部轴对齐包围盒。
    const Bounds3& queryBounds() const;
    // 返回源网格包含的三角形数量。
    std::size_t triangleCount() const;

    /// 点查询
    // 返回指定点到网格表面的最短距离平方。
    double distanceSquaredToPoint(const MyMath::Vector3& point) const;
    // 返回指定点到网格表面的最短距离。
    double distanceToPoint(const MyMath::Vector3& point) const;
    // 使用与三角形方向无关的奇偶射线规则判断指定点是否位于封闭网格内部或边界上。
    bool containsPoint(const MyMath::Vector3& point) const;

    /// 范围查询
    // 返回指定轴对齐包围盒与封闭网格实体之间的保守空间关系。
    ShapeRelation classifyBounds(const Bounds3& bounds) const;
    // 使用已经计算好的包围盒中心和半尺寸执行保守空间分类。
    ShapeRelation classifyBounds(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

private:
    // 表示射线与三角形之间的稳定分类结果。
    enum class RayTriangleRelation
    {
        Miss,
        Hit,
        Ambiguous
    };

    // 保存一个已经从索引网格展开的独立三角形及其包围盒。
    struct Triangle
    {
        MyMath::Vector3 point0; // 第一个顶点。
        MyMath::Vector3 point1; // 第二个顶点。
        MyMath::Vector3 point2; // 第三个顶点。
        Bounds3 bounds; // 当前三角形的局部轴对齐包围盒。
    };

    // 保存一个扁平二叉BVH节点，叶节点通过triangleCount大于零进行标识。
    struct BvhNode
    {
        BvhNode();

        // 判断当前节点是否为叶节点。
        bool isLeaf() const;

        Bounds3 bounds; // 当前节点覆盖的全部三角形包围盒。
        std::uint32_t leftChild; // 内部节点左子节点索引。
        std::uint32_t rightChild; // 内部节点右子节点索引。
        std::uint32_t firstTriangle; // 叶节点第一个连续三角形索引。
        std::uint32_t triangleCount; // 叶节点连续三角形数量，内部节点为零。
    };

    // 按指定坐标轴比较两个三角形包围盒中心。
    struct TriangleCenterLess
    {
        explicit TriangleCenterLess(unsigned int axisValue);
        bool operator()(const Triangle& first, const Triangle& second) const;

        unsigned int axis; // 当前比较使用的坐标轴编号。
    };

    // 返回网格顶点位置。
    static MyMath::Vector3 vertexPosition(const MeshVertex& vertex);

    // 返回三个点形成的轴对齐包围盒。
    static Bounds3 triangleBounds(const MyMath::Vector3& point0,
                                  const MyMath::Vector3& point1,
                                  const MyMath::Vector3& point2);

    // 返回向量指定坐标轴的分量。
    static double component(const MyMath::Vector3& vector, unsigned int axis);

    // 判断指定半尺寸是否为有限非负数据。
    static bool isValidExtent(const MyMath::Vector3& extent);

    // 返回查询包围盒所需的尺度相关长度误差。
    static double queryLengthTolerance(const Bounds3& bounds);

    // 返回指定尝试编号使用的确定性非轴向射线方向。
    static MyMath::Vector3 rayDirection(unsigned int attemptIndex);

    // 判断由中心和半尺寸表示的包围盒是否与另一个包围盒相交或接触。
    static bool boundsIntersect(const MyMath::Vector3& center,
                                const MyMath::Vector3& extent,
                                const Bounds3& bounds,
                                double tolerance);

    // 返回指定点到轴对齐包围盒的最短距离平方。
    static double pointBoundsDistanceSquared(const MyMath::Vector3& point, const Bounds3& bounds);

    // 判断指定正向射线是否与轴对齐包围盒相交。
    static bool rayIntersectsBounds(const MyMath::Vector3& origin,
                                    const MyMath::Vector3& direction,
                                    const Bounds3& bounds);

    // 返回点到线段的距离平方。
    static double pointSegmentDistanceSquared(const MyMath::Vector3& point,
                                              const MyMath::Vector3& segmentStart,
                                              const MyMath::Vector3& segmentEnd);

    // 返回点到三角形的距离平方。
    static double pointTriangleDistanceSquared(const MyMath::Vector3& point, const Triangle& triangle);

    // 对射线与三角形进行带边界歧义检测的分类。
    static RayTriangleRelation classifyRayTriangle(const MyMath::Vector3& origin,
                                                   const MyMath::Vector3& direction,
                                                   const Triangle& triangle,
                                                   double lengthTolerance);

    // 使用包含边和顶点的传统规则判断正向射线是否与三角形相交。
    static bool rayIntersectsTriangleInclusive(const MyMath::Vector3& origin,
                                               const MyMath::Vector3& direction,
                                               const Triangle& triangle);

    // 判断三角形和由中心、半尺寸表示的轴对齐包围盒是否相交或接触。
    static bool triangleIntersectsBounds(const Triangle& triangle,
                                         const MyMath::Vector3& center,
                                         const MyMath::Vector3& extent,
                                         double lengthTolerance);

    // 判断三角形和包围盒在指定分离轴上的投影是否重叠。
    static bool overlapsOnAxis(const MyMath::Vector3& relativePoint0,
                               const MyMath::Vector3& relativePoint1,
                               const MyMath::Vector3& relativePoint2,
                               const MyMath::Vector3& extent,
                               const MyMath::Vector3& axis,
                               double lengthTolerance);

    // 根据连续三角形范围递归建立一个BVH节点，并返回节点索引。
    std::uint32_t buildBvhNode(std::uint32_t firstTriangle, std::uint32_t triangleCount);

    // 判断指定点是否位于任意三角形边界误差范围内。
    bool isPointOnSurface(const MyMath::Vector3& point) const;

    // 尝试使用不穿过三角形边或顶点的射线统计相交次数。
    bool tryRayIntersectionCount(const MyMath::Vector3& origin,
                                 const MyMath::Vector3& direction,
                                 std::size_t& intersectionCount) const;

    // 使用传统闭区间规则返回指定正向射线与网格三角形的相交次数。
    std::size_t rayIntersectionCountFallback(const MyMath::Vector3& origin,
                                             const MyMath::Vector3& direction) const;

    // 判断指定轴对齐包围盒是否与任意网格三角形相交。
    bool intersectsAnyTriangle(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

private:
    std::vector<Triangle> m_triangles; // 按BVH叶节点连续排列的独立三角形。
    std::vector<BvhNode> m_bvhNodes; // 深度优先排列的扁平二叉BVH节点。
    Bounds3 m_queryBounds; // 实际三角形使用顶点形成的局部轴对齐包围盒。
    double m_lengthTolerance; // 根据float坐标精度和查询尺度计算的统一长度误差。
    double m_lengthToleranceSquared; // 统一长度误差平方。
    bool m_valid; // 当前查询器是否已经成功建立。
};

}
}

#endif // MYVOXEL_GEOMETRY_MESHQUERY_H