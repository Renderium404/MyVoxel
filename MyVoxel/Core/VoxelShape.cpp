#include "VoxelShape.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断TSDF固定背景距离是否为有限正值。
bool isValidBackgroundDistance(float backgroundDistance)
{
    return std::isfinite(static_cast<double>(backgroundDistance)) && backgroundDistance > 0.0f;
}

// 旧构造接口默认使用一个最高层体素边长作为截断背景距离，避免破坏现有调用点；正式建模可通过显式构造指定B。
float defaultBackgroundDistance(const MyVoxel::VoxelGrid& grid)
{
    const float result = static_cast<float>(grid.minimumCellEdgeLength());
    MYVOXEL_ASSERT_MESSAGE(isValidBackgroundDistance(result), "VoxelGrid minimum cell edge length must be representable as a positive float TSDF background distance.");
    return result;
}

}

namespace MyVoxel
{

VoxelShape::SharedData::SharedData(const VoxelGrid& gridValue, float backgroundDistanceValue)
    : grid(gridValue)
    , forest(backgroundDistanceValue)
    , features()
    , featureState(VoxelFeatureState::Complete)
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "VoxelShape requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(forest.isValid(), "VoxelShape requires a valid TSDF VoxelForest.");
    MYVOXEL_ASSERT_MESSAGE(features.isValid(), "VoxelShape requires a valid initial FeatureSet.");
}

VoxelShape::SharedData::SharedData(const SharedData& other)
    : grid(other.grid)
    , forest(other.forest)
    , features(other.features)
    , featureState(other.featureState)
{
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Copied VoxelShape data must contain a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(forest.isValid(), "Copied VoxelShape data must contain a valid TSDF VoxelForest.");
    MYVOXEL_ASSERT_MESSAGE(features.isValid(), "Copied VoxelShape data must contain a valid FeatureSet.");
}

VoxelShape::VoxelShape()
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(1.0, BaseVoxelLevel), 1.0f))
    , m_transform(MyMath::Matrix4::identity())
{
}

VoxelShape::VoxelShape(const VoxelGrid& grid)
    : m_data(Foundation::makeRef<SharedData>(grid, defaultBackgroundDistance(grid)))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(const VoxelGrid& grid, float backgroundDistanceValue)
    : m_data(Foundation::makeRef<SharedData>(grid, backgroundDistanceValue))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(baseCellEdgeLength, maximumLevel), defaultBackgroundDistance(VoxelGrid(baseCellEdgeLength, maximumLevel))))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel, float backgroundDistanceValue)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(baseCellEdgeLength, maximumLevel), backgroundDistanceValue))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(origin, baseCellEdgeLength, maximumLevel), defaultBackgroundDistance(VoxelGrid(origin, baseCellEdgeLength, maximumLevel))))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

VoxelShape::VoxelShape(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel, float backgroundDistanceValue)
    : m_data(Foundation::makeRef<SharedData>(VoxelGrid(origin, baseCellEdgeLength, maximumLevel), backgroundDistanceValue))
    , m_transform(MyMath::Matrix4::identity())
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelShape parameters must define a valid voxel shape.");
}

/// 状态判断

bool VoxelShape::isValid() const
{
    return m_data && m_data->grid.isValid() && m_data->forest.isValid() && m_data->features.isValid() &&
           m_transform.isAffine() && m_transform.isInvertible();
}

bool VoxelShape::isEmpty() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    return m_data->forest.isEmpty();
}

bool VoxelShape::isDataShared() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->referenceCount() > 1;
}

bool VoxelShape::sharesDataWith(const VoxelShape& other) const
{
    MYVOXEL_ASSERT_MESSAGE(m_data && other.m_data, "VoxelShape shared data must not be null.");
    return m_data.get() == other.m_data.get();
}

/// 体素空间

const VoxelGrid& VoxelShape::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->grid;
}

float VoxelShape::backgroundDistance() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->forest.backgroundDistance();
}

bool VoxelShape::supportsAddress(const VoxelCellAddress& address) const
{
    return m_data && address.level <= m_data->grid.maximumLevel();
}

/// 体素与距离状态

VoxelState VoxelShape::state(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_data->forest.state(address);
}

bool VoxelShape::hasNode(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_data->forest.hasNode(address);
}

float VoxelShape::distance(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(address.level == grid().maximumLevel(), "VoxelShape distance queries require the highest sampling level.");
    return m_data->forest.distance(address);
}

/// 修改入口

VoxelShapeSession VoxelShape::session(VoxelLevel changeTrackingLevel)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot create a session for an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(changeTrackingLevel <= grid().maximumLevel(), "Voxel change tracking level exceeds the shape maximum level.");
    return VoxelShapeSession(*this, changeTrackingLevel);
}

/// 体素数据

const VoxelForest& VoxelShape::forest() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->forest;
}

std::size_t VoxelShape::rootCount() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelShape.");
    return m_data->forest.rootCount();
}

/// 显式表面特征

const VoxelFeatureSet& VoxelShape::features() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->features;
}

VoxelFeatureState VoxelShape::featureState() const
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");
    return m_data->featureState;
}

bool VoxelShape::hasCompleteFeatures() const
{
    return featureState() == VoxelFeatureState::Complete;
}

/// 空间变换

const MyMath::Matrix4& VoxelShape::transform() const
{
    return m_transform;
}

void VoxelShape::setTransform(const MyMath::Matrix4& transformValue)
{
    MYVOXEL_ASSERT_MESSAGE(transformValue.isAffine(), "VoxelShape transform must be affine.");
    MYVOXEL_ASSERT_MESSAGE(transformValue.isInvertible(), "VoxelShape transform must be invertible.");
    m_transform = transformValue;
}

void VoxelShape::resetTransform()
{
    m_transform = MyMath::Matrix4::identity();
}

/// 内部辅助

void VoxelShape::detach()
{
    MYVOXEL_ASSERT_MESSAGE(m_data, "VoxelShape shared data must not be null.");

    if (m_data->referenceCount() <= 1)
    {
        return;
    }

    m_data = Foundation::makeRef<SharedData>(*m_data);
    MYVOXEL_ASSERT_MESSAGE(m_data->referenceCount() == 1, "Detached VoxelShape data must be exclusively owned.");
}

}
