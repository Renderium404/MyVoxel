#include "ShapeQuery.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const unsigned int XOctantMask = 1; // 八分体编号第0位控制X方向。
const unsigned int YOctantMask = 2; // 八分体编号第1位控制Y方向。
const unsigned int ZOctantMask = 4; // 八分体编号第2位控制Z方向。

// 判断半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

// 返回指定八分体的查询空间中心。
MyMath::Vector3 octantCenter(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, unsigned int octantIndex)
{
    return MyMath::Vector3(parentCenter.x() + ((octantIndex & XOctantMask) != 0 ? childExtent.x() : -childExtent.x()), parentCenter.y() + ((octantIndex & YOctantMask) != 0 ? childExtent.y() : -childExtent.y()), parentCenter.z() + ((octantIndex & ZOctantMask) != 0 ? childExtent.z() : -childExtent.z()));
}

}

namespace MyVoxel
{

ShapeQuery::ShapeQuery(const Topology_Shape& topology)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_identityQuery(true)
    , m_valid(false)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "ShapeQuery requires a valid Topology_Shape.");

    if (!topology.isValid())
    {
        return;
    }

    initialize(Shape(topology), MyMath::Matrix4::identity());
}

ShapeQuery::ShapeQuery(const Shape& shape)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_identityQuery(false)
    , m_valid(false)
{
    initialize(shape, MyMath::Matrix4::identity());
}

ShapeQuery::ShapeQuery(const Shape& shape, const MyMath::Matrix4& queryToWorld)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_identityQuery(false)
    , m_valid(false)
{
    initialize(shape, queryToWorld);
}

ShapeQuery::~ShapeQuery()
{
}

/// 状态判断

bool ShapeQuery::isValid() const
{
    return m_valid;
}

bool ShapeQuery::isIdentityQuery() const
{
    return m_identityQuery;
}

/// 查询对象与空间数据

const Shape& ShapeQuery::shape() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_shape;
}

const Topology_Shape& ShapeQuery::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_shape.topology();
}

const Geometry_Shape& ShapeQuery::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_shape.geometry();
}

const MyMath::Matrix4& ShapeQuery::queryToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_queryToLocal;
}

const MyMath::Matrix4& ShapeQuery::localToQuery() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_localToQuery;
}

const Bounds3& ShapeQuery::queryBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid ShapeQuery.");
    return m_queryBounds;
}

/// 标准空间查询

bool ShapeQuery::containsPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "ShapeQuery point must be finite.");

    if (!m_queryBounds.contains(point))
    {
        return false;
    }

    if (m_identityQuery)
    {
        return m_shape.containsLocalPoint(point);
    }

    return m_shape.containsLocalPoint(m_queryToLocal.transformPoint(point));
}

ShapeRelation ShapeQuery::classifyBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "ShapeQuery bounds must be valid.");

    if (!m_queryBounds.intersects(bounds))
    {
        return ShapeRelation::Outside;
    }

    if (m_identityQuery)
    {
        return m_shape.classifyLocalBounds(bounds);
    }

    // 标准路径显式变换八个角点，保持为快速路径提供独立正确性基准。
    return m_shape.classifyLocalBounds(bounds.transformed(m_queryToLocal));
}

/// 快速空间查询

ShapeRelation ShapeQuery::classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "ShapeQuery bounds center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(extent), "ShapeQuery bounds extent must be finite and non-negative.");

    if (!intersectsQueryBounds(center, extent))
    {
        return ShapeRelation::Outside;
    }

    return classifyBoundsFastImpl(center, extent);
}

void ShapeQuery::classifyOctantBoundsFast(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, std::array<ShapeRelation, OctantCount>& results) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(parentCenter.isFinite(), "ShapeQuery parent center must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isValidExtent(childExtent), "ShapeQuery child extent must be finite and non-negative.");
    classifyOctantBoundsFastImpl(parentCenter, childExtent, results);
}

/// 特殊Shape快速查询扩展

ShapeRelation ShapeQuery::classifyBoundsFastImpl(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    if (m_identityQuery)
    {
        return m_shape.classifyLocalBoundsFast(center, extent);
    }

    MyMath::Vector3 localCenter;
    MyMath::Vector3 localExtent;
    transformBoundsToLocalFast(center, extent, localCenter, localExtent);
    return m_shape.classifyLocalBoundsFast(localCenter, localExtent);
}

void ShapeQuery::classifyOctantBoundsFastImpl(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, std::array<ShapeRelation, OctantCount>& results) const
{
    if (m_identityQuery)
    {
        for (unsigned int octantIndex = 0; octantIndex < static_cast<unsigned int>(OctantCount); ++octantIndex)
        {
            const MyMath::Vector3 childCenter = octantCenter(parentCenter, childExtent, octantIndex);
            results[octantIndex] = intersectsQueryBounds(childCenter, childExtent) ? m_shape.classifyLocalBoundsFast(childCenter, childExtent) : ShapeRelation::Outside;
        }

        return;
    }

    const MyMath::Vector3 localParentCenter = m_queryToLocal.transformPoint(parentCenter);
    const MyMath::Vector3 localOffsetX(m_queryToLocal(0, 0) * childExtent.x(), m_queryToLocal(1, 0) * childExtent.x(), m_queryToLocal(2, 0) * childExtent.x());
    const MyMath::Vector3 localOffsetY(m_queryToLocal(0, 1) * childExtent.y(), m_queryToLocal(1, 1) * childExtent.y(), m_queryToLocal(2, 1) * childExtent.y());
    const MyMath::Vector3 localOffsetZ(m_queryToLocal(0, 2) * childExtent.z(), m_queryToLocal(1, 2) * childExtent.z(), m_queryToLocal(2, 2) * childExtent.z());
    const MyMath::Vector3 localExtent(MyMath::Vector3::dot(m_absoluteQueryToLocalRowX, childExtent), MyMath::Vector3::dot(m_absoluteQueryToLocalRowY, childExtent), MyMath::Vector3::dot(m_absoluteQueryToLocalRowZ, childExtent));

    for (unsigned int octantIndex = 0; octantIndex < static_cast<unsigned int>(OctantCount); ++octantIndex)
    {
        const MyMath::Vector3 childCenter = octantCenter(parentCenter, childExtent, octantIndex);

        if (!intersectsQueryBounds(childCenter, childExtent))
        {
            results[octantIndex] = ShapeRelation::Outside;
            continue;
        }

        const double signX = (octantIndex & XOctantMask) != 0 ? 1.0 : -1.0;
        const double signY = (octantIndex & YOctantMask) != 0 ? 1.0 : -1.0;
        const double signZ = (octantIndex & ZOctantMask) != 0 ? 1.0 : -1.0;
        const MyMath::Vector3 localCenter(localParentCenter.x() + signX * localOffsetX.x() + signY * localOffsetY.x() + signZ * localOffsetZ.x(), localParentCenter.y() + signX * localOffsetX.y() + signY * localOffsetY.y() + signZ * localOffsetZ.y(), localParentCenter.z() + signX * localOffsetX.z() + signY * localOffsetY.z() + signZ * localOffsetZ.z());
        results[octantIndex] = m_shape.classifyLocalBoundsFast(localCenter, localExtent);
    }
}

/// 通用快速查询辅助

void ShapeQuery::transformBoundsToLocalFast(const MyMath::Vector3& center, const MyMath::Vector3& extent, MyMath::Vector3& localCenter, MyMath::Vector3& localExtent) const
{
    localCenter = m_queryToLocal.transformPoint(center);
    localExtent.set(MyMath::Vector3::dot(m_absoluteQueryToLocalRowX, extent), MyMath::Vector3::dot(m_absoluteQueryToLocalRowY, extent), MyMath::Vector3::dot(m_absoluteQueryToLocalRowZ, extent));
}

bool ShapeQuery::intersectsQueryBounds(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    return center.x() + extent.x() >= m_queryBounds.minimum().x() && center.x() - extent.x() <= m_queryBounds.maximum().x() && center.y() + extent.y() >= m_queryBounds.minimum().y() && center.y() - extent.y() <= m_queryBounds.maximum().y() && center.z() + extent.z() >= m_queryBounds.minimum().z() && center.z() - extent.z() <= m_queryBounds.maximum().z();
}

/// 初始化

void ShapeQuery::initialize(const Shape& shape, const MyMath::Matrix4& queryToWorld)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "ShapeQuery requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(queryToWorld.isAffine(), "ShapeQuery query-to-world transform must be affine.");

    if (!shape.isValid() || !queryToWorld.isAffine())
    {
        return;
    }

    MyMath::Matrix4 worldToQuery;
    const bool inverted = queryToWorld.inverted(worldToQuery);

    MYVOXEL_ASSERT_MESSAGE(inverted, "ShapeQuery query-to-world transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    m_shape = shape;
    m_queryToLocal = shape.worldToLocal() * queryToWorld;
    m_localToQuery = worldToQuery * shape.localToWorld();
    m_identityQuery = m_queryToLocal.isIdentity(0.0);
    updateAbsoluteQueryToLocalRows();

    // 查询器构造不属于递归热点，使用标准八角点变换计算查询空间保守包围盒。
    m_queryBounds = shape.localBounds().transformed(m_localToQuery);
    m_valid = m_queryBounds.isValid() && m_queryBounds.hasVolume();
}

void ShapeQuery::updateAbsoluteQueryToLocalRows()
{
    m_absoluteQueryToLocalRowX.set(std::fabs(m_queryToLocal(0, 0)), std::fabs(m_queryToLocal(0, 1)), std::fabs(m_queryToLocal(0, 2)));
    m_absoluteQueryToLocalRowY.set(std::fabs(m_queryToLocal(1, 0)), std::fabs(m_queryToLocal(1, 1)), std::fabs(m_queryToLocal(1, 2)));
    m_absoluteQueryToLocalRowZ.set(std::fabs(m_queryToLocal(2, 0)), std::fabs(m_queryToLocal(2, 1)), std::fabs(m_queryToLocal(2, 2)));
}

}