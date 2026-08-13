#include "VoxelShapeSession.h"

#include <utility>

#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelShapeSession::VoxelShapeSession(VoxelShape& shape, VoxelLevel changeTrackingLevel)
    : m_shape(&shape)
    , m_forest(nullptr)
    , m_changes(changeTrackingLevel)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Cannot create VoxelShapeSession for an invalid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(changeTrackingLevel <= shape.grid().maximumLevel(), "Voxel change tracking level exceeds the shape maximum level.");

    shape.detach();
    m_forest = &shape.m_data->forest;
    MYVOXEL_ASSERT_MESSAGE(!shape.isDataShared(), "VoxelShapeSession requires exclusively owned shape data.");
}

VoxelShapeSession::VoxelShapeSession(VoxelShapeSession&& other)
    : m_shape(other.m_shape)
    , m_forest(other.m_forest)
    , m_changes(std::move(other.m_changes))
{
    other.m_shape = nullptr;
    other.m_forest = nullptr;
}

VoxelShapeSession& VoxelShapeSession::operator=(VoxelShapeSession&& other)
{
    if (this == &other)
    {
        return *this;
    }

    m_shape = other.m_shape;
    m_forest = other.m_forest;
    m_changes = std::move(other.m_changes);
    other.m_shape = nullptr;
    other.m_forest = nullptr;
    return *this;
}

/// 会话状态

bool VoxelShapeSession::hasChanges() const{return m_changes.hasChanges();}
const VoxelChangeSet& VoxelShapeSession::changes() const{return m_changes;}

VoxelChangeSet VoxelShapeSession::takeChanges()
{
    VoxelChangeSet result(m_changes.trackingLevel());
    m_changes.swap(result);
    return result;
}

/// 体素空间

const VoxelGrid& VoxelShapeSession::grid() const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape, "VoxelShapeSession is not bound to a VoxelShape.");
    return m_shape->grid();
}

float VoxelShapeSession::backgroundDistance() const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    return m_forest->backgroundDistance();
}

bool VoxelShapeSession::supportsAddress(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape, "VoxelShapeSession is not bound to a VoxelShape.");
    return m_shape->supportsAddress(address);
}

/// 体素与距离状态

VoxelState VoxelShapeSession::state(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_forest->state(address);
}

bool VoxelShapeSession::hasNode(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    return m_forest->hasNode(address);
}

float VoxelShapeSession::distance(const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(address.level == grid().maximumLevel(), "VoxelShapeSession distance queries require the highest sampling level.");
    return m_forest->distance(address);
}

bool VoxelShapeSession::setState(const VoxelCellAddress& address, VoxelState stateValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(stateValue == VoxelState::Empty || stateValue == VoxelState::Material, "VoxelShapeSession::setState only accepts Empty or Material.");

    const bool changed = m_forest->setState(address, stateValue);
    if (changed)
    {
        m_changes.recordFieldChange(address);
        invalidateFeaturesForFieldChange();
    }
    return changed;
}

bool VoxelShapeSession::setDistance(const VoxelCellAddress& address, float distanceValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(address.level == grid().maximumLevel(), "VoxelShapeSession distance writes require the highest sampling level.");

    const bool changed = m_forest->setDistance(address, distanceValue);
    if (changed)
    {
        m_changes.recordFieldChange(address);
        invalidateFeaturesForFieldChange();
    }
    return changed;
}

/// 体素结构

bool VoxelShapeSession::split(const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(address.level < grid().maximumLevel(), "The maximum-level voxel cannot be subdivided.");

    const bool changed = m_forest->split(address);
    if (changed)
    {
        m_changes.recordStructureChange(address);
    }
    return changed;
}

bool VoxelShapeSession::merge(const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(address.level < grid().maximumLevel(), "The maximum-level voxel has no child voxels to merge.");

    const bool changed = m_forest->merge(address);
    if (changed)
    {
        m_changes.recordStructureChange(address);
    }
    return changed;
}

bool VoxelShapeSession::pruneTree(const VoxelCellIndex& rootIndex)
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    const bool changed = m_forest->pruneTree(rootIndex);
    if (changed)
    {
        m_changes.recordStructureChange(VoxelCellAddress(rootIndex, BaseVoxelLevel));
    }
    return changed;
}

std::size_t VoxelShapeSession::prune()
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");

    std::vector<VoxelCellIndex> rootIndices;
    rootIndices.reserve(m_forest->rootCount());
    m_forest->forEachRootCell(
        [&rootIndices](const VoxelCellAddress& rootAddress)
        {
            rootIndices.push_back(rootAddress.index);
        });

    std::size_t changedRootCount = 0;
    for (std::size_t rootPosition = 0; rootPosition < rootIndices.size(); ++rootPosition)
    {
        if (pruneTree(rootIndices[rootPosition]))
        {
            ++changedRootCount;
        }
    }
    return changedRootCount;
}

/// 独立脏区构建

VoxelChangeSet::DirtyCellRegion VoxelShapeSession::createDirtyCellRegion(const VoxelCellIndex& rootIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape, "VoxelShapeSession is not bound to a VoxelShape.");
    return VoxelChangeSet::DirtyCellRegion(rootIndex, m_changes.trackingLevel());
}

void VoxelShapeSession::recordFieldChange(VoxelChangeSet::DirtyCellRegion& dirtyRegion, const VoxelCellAddress& address) const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape, "VoxelShapeSession is not bound to a VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(supportsAddress(address), "Voxel field change address exceeds the shape maximum level.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.trackingLevel() == m_changes.trackingLevel(), "Voxel dirty region tracking level does not match the session.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.rootIndex() == rootCellAddress(address).index, "Voxel field change address belongs to another dirty Root.");
    dirtyRegion.recordFieldChange(address);
}

void VoxelShapeSession::recordMaterialChange(VoxelChangeSet::DirtyCellRegion& dirtyRegion, const VoxelCellAddress& address) const
{
    recordFieldChange(dirtyRegion, address);
}

/// 根树操作

const VoxelTree* VoxelShapeSession::tree(const VoxelCellIndex& rootIndex) const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    return m_forest->getTree(rootIndex);
}

void VoxelShapeSession::setTree(const VoxelCellIndex& rootIndex, const VoxelTree& treeValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    m_forest->setTree(rootIndex, treeValue);
    m_changes.recordFullRootChange(rootIndex);
    invalidateFeaturesForFieldChange();
}

void VoxelShapeSession::setTree(const VoxelCellIndex& rootIndex, VoxelTree&& treeValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    m_forest->setTree(rootIndex, std::move(treeValue));
    m_changes.recordFullRootChange(rootIndex);
    invalidateFeaturesForFieldChange();
}

void VoxelShapeSession::setTree(const VoxelCellIndex& rootIndex, const VoxelTree& treeValue, const VoxelChangeSet::DirtyCellRegion& dirtyRegion)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.rootIndex() == rootIndex, "Voxel tree dirty region belongs to another Root.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.trackingLevel() == m_changes.trackingLevel(), "Voxel tree dirty region tracking level does not match the session.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.changedCellCount() > 0, "Voxel tree precise commit requires a non-empty dirty region.");
    m_forest->setTree(rootIndex, treeValue);
    m_changes.recordDirtyRegion(dirtyRegion);
    invalidateFeaturesForFieldChange();
}

void VoxelShapeSession::setTree(const VoxelCellIndex& rootIndex, VoxelTree&& treeValue, const VoxelChangeSet::DirtyCellRegion& dirtyRegion)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.rootIndex() == rootIndex, "Voxel tree dirty region belongs to another Root.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.trackingLevel() == m_changes.trackingLevel(), "Voxel tree dirty region tracking level does not match the session.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.changedCellCount() > 0, "Voxel tree precise commit requires a non-empty dirty region.");
    m_forest->setTree(rootIndex, std::move(treeValue));
    m_changes.recordDirtyRegion(dirtyRegion);
    invalidateFeaturesForFieldChange();
}

bool VoxelShapeSession::eraseTree(const VoxelCellIndex& rootIndex)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    const bool changed = m_forest->eraseTree(rootIndex);
    if (changed)
    {
        m_changes.recordFullRootChange(rootIndex);
        invalidateFeaturesForFieldChange();
    }
    return changed;
}

bool VoxelShapeSession::eraseTree(const VoxelCellIndex& rootIndex, const VoxelChangeSet::DirtyCellRegion& dirtyRegion)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.rootIndex() == rootIndex, "Voxel tree dirty region belongs to another Root.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.trackingLevel() == m_changes.trackingLevel(), "Voxel tree dirty region tracking level does not match the session.");
    MYVOXEL_ASSERT_MESSAGE(dirtyRegion.changedCellCount() > 0, "Voxel tree precise erase requires a non-empty dirty region.");

    const bool changed = m_forest->eraseTree(rootIndex);
    if (changed)
    {
        m_changes.recordDirtyRegion(dirtyRegion);
        invalidateFeaturesForFieldChange();
    }
    return changed;
}

void VoxelShapeSession::moveTreesTo(std::vector<VoxelForest::TreeEntry>& trees)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");
    if (m_forest->isEmpty())
    {
        return;
    }

    m_forest->forEachRootCell(
        [this](const VoxelCellAddress& rootAddress)
        {
            m_changes.recordFullRootChange(rootAddress.index);
        });

    m_forest->moveTreesTo(trees);
    invalidateFeaturesForFieldChange();
}

/// 显式表面特征

const VoxelFeatureSet& VoxelShapeSession::features() const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_shape->m_data, "VoxelShapeSession is not fully bound.");
    return m_shape->m_data->features;
}

VoxelFeatureState VoxelShapeSession::featureState() const
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_shape->m_data, "VoxelShapeSession is not fully bound.");
    return m_shape->m_data->featureState;
}

bool VoxelShapeSession::hasCompleteFeatures() const
{
    return featureState() == VoxelFeatureState::Complete;
}

void VoxelShapeSession::setFeatures(const VoxelFeatureSet& featuresValue)
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_shape->m_data, "VoxelShapeSession is not fully bound.");
    MYVOXEL_ASSERT_MESSAGE(featuresValue.isValid(), "VoxelShapeSession requires a valid FeatureSet.");
    m_shape->m_data->features = featuresValue;
    m_shape->m_data->featureState = VoxelFeatureState::Complete;
}

bool VoxelShapeSession::clearFeatures()
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_shape->m_data, "VoxelShapeSession is not fully bound.");
    const bool changed = !m_shape->m_data->features.isEmpty() || m_shape->m_data->featureState != VoxelFeatureState::Complete;
    m_shape->m_data->features.clear();
    m_shape->m_data->featureState = VoxelFeatureState::Complete;
    return changed;
}

void VoxelShapeSession::invalidateFeatures()
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_shape->m_data, "VoxelShapeSession is not fully bound.");
    m_shape->m_data->features.clear();
    m_shape->m_data->featureState = VoxelFeatureState::Unavailable;
}

/// 体素数据

const VoxelForest& VoxelShapeSession::forest() const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    return *m_forest;
}

std::size_t VoxelShapeSession::rootCount() const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    return m_forest->rootCount();
}

bool VoxelShapeSession::isEmpty() const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelShapeSession forest must not be null.");
    return m_forest->isEmpty();
}

bool VoxelShapeSession::clear()
{
    MYVOXEL_ASSERT_MESSAGE(m_shape && m_forest, "VoxelShapeSession is not fully bound.");

    bool fieldChanged = false;

    if (!m_forest->isEmpty())
    {
        m_forest->forEachRootCell(
            [this](const VoxelCellAddress& rootAddress)
            {
                m_changes.recordFullRootChange(rootAddress.index);
            });

        m_forest->clear();
        fieldChanged = true;
    }

    const bool featureChanged = clearFeatures();
    return fieldChanged || featureChanged;
}

/// 内部辅助

void VoxelShapeSession::invalidateFeaturesForFieldChange()
{
    invalidateFeatures();
}

}
