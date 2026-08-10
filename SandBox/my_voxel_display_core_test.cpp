#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Object/Display_Object.h"
#include "MyVoxel/Display/Object/Display_ObjectManager.h"
#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Display/Object/Display_ObjectUpdate.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace
{

const double TestTolerance = 1.0e-10; // Display基础层空间数据测试使用的统一浮点比较容差。

class TestRunner
{
public:
    TestRunner()
        : m_passed(0)
        , m_failed(0)
    {
    }

    void check(bool condition, const char* name)
    {
        if (condition)
        {
            ++m_passed;
            std::cout << "[PASS] " << name << std::endl;
        }
        else
        {
            ++m_failed;
            std::cout << "[FAIL] " << name << std::endl;
        }
    }

    int passed() const
    {
        return m_passed;
    }

    int failed() const
    {
        return m_failed;
    }

private:
    int m_passed;
    int m_failed;
};

// 判断两个双精度数是否在测试容差内相等。
bool nearlyEqual(double first, double second, double tolerance = TestTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

// 判断两个三维点是否在测试容差内相等。
bool pointEqual(const MyMath::Vector3& first, const MyMath::Vector3& second, double tolerance = TestTolerance)
{
    return first.isEqualTo(second, tolerance);
}

// 创建一个具有完整单位法线和逐三角形颜色的可渲染单三角形Mesh。
MyVoxel::Mesh makeTriangleMesh(const MyVoxel::Display_Color& color)
{
    MyVoxel::Mesh mesh;

    const MyVoxel::MeshVertex vertex0(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    const MyVoxel::MeshVertex vertex1(2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    const MyVoxel::MeshVertex vertex2(0.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    mesh.appendTriangle(vertex0, vertex1, vertex2, color);
    return mesh;
}

// 创建第二个具有不同范围的可渲染单三角形Mesh，用于验证资源替换。
MyVoxel::Mesh makeSecondTriangleMesh(const MyVoxel::Display_Color& color)
{
    MyVoxel::Mesh mesh;

    const MyVoxel::MeshVertex vertex0(-1.0, -1.0, 0.0, 0.0, 0.0, 1.0);
    const MyVoxel::MeshVertex vertex1(4.0, -1.0, 0.0, 0.0, 0.0, 1.0);
    const MyVoxel::MeshVertex vertex2(-1.0, 5.0, 0.0, 0.0, 0.0, 1.0);

    mesh.appendTriangle(vertex0, vertex1, vertex2, color);
    return mesh;
}

// 创建两个独立线段使用的GL_LINES端点序列。
std::vector<MyMath::Vector3> makeLinePoints()
{
    std::vector<MyMath::Vector3> points;
    points.push_back(MyMath::Vector3(0.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(2.0, 0.0, 0.0));
    points.push_back(MyMath::Vector3(2.0, 1.0, 0.0));
    points.push_back(MyMath::Vector3(4.0, 1.0, 0.0));
    return points;
}

/// Display_ResourceManager

void testResourceManager(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager manager;

    runner.check(manager.resourceCount() == 0U, "ResourceManager.initialCount");
    runner.check(manager.memoryByteSize() == 0U, "ResourceManager.initialMemory");
    runner.check(!manager.contains(0), "ResourceManager.zeroIdRejected");
    runner.check(!manager.resource(0), "ResourceManager.zeroResourceEmpty");

    const MyVoxel::Mesh mesh = makeTriangleMesh(MyVoxel::Display_Color::redColor());
    runner.check(mesh.isRenderable(), "ResourceManager.sourceMeshRenderable");

    const MyVoxel::Display_ResourceId meshId = manager.createMeshResource(mesh);

    runner.check(meshId != 0, "ResourceManager.meshId");
    runner.check(manager.contains(meshId), "ResourceManager.containsMesh");
    runner.check(manager.resourceCount() == 1U, "ResourceManager.meshCount");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_Resource> meshBase = manager.resource(meshId);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> meshResource = manager.meshResource(meshId);

    runner.check(meshBase && meshBase->isValid(), "ResourceManager.meshBaseValid");
    runner.check(meshBase && meshBase->kind() == MyVoxel::Display_ResourceKind::Mesh, "ResourceManager.meshKind");
    runner.check(meshResource && meshResource->isValid(), "ResourceManager.meshTypedValid");
    runner.check(!manager.lineResource(meshId), "ResourceManager.meshRejectLineQuery");

    if (meshResource)
    {
        runner.check(meshResource->vertexCount() == 3U, "ResourceManager.meshVertexCount");
        runner.check(meshResource->triangleCount() == 1U, "ResourceManager.meshTriangleCount");
        runner.check(meshResource->memoryByteSize() == meshResource->vertexCount() * sizeof(MyVoxel::Display_MeshVertex),
                     "ResourceManager.meshMemory");
        runner.check(pointEqual(meshResource->localBounds().minimum(), MyMath::Vector3(0.0, 0.0, 0.0)),
                     "ResourceManager.meshBoundsMinimum");
        runner.check(pointEqual(meshResource->localBounds().maximum(), MyMath::Vector3(2.0, 3.0, 0.0)),
                     "ResourceManager.meshBoundsMaximum");
    }

    const std::vector<MyMath::Vector3> linePoints = makeLinePoints();
    const MyVoxel::Display_ResourceId lineId = manager.createLineResource(linePoints, MyVoxel::Display_Color::greenColor());

    runner.check(lineId != 0, "ResourceManager.lineId");
    runner.check(lineId != meshId, "ResourceManager.uniqueIds");
    runner.check(manager.contains(lineId), "ResourceManager.containsLine");
    runner.check(manager.resourceCount() == 2U, "ResourceManager.totalCount");

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> lineResource = manager.lineResource(lineId);

    runner.check(lineResource && lineResource->isValid(), "ResourceManager.lineTypedValid");
    runner.check(lineResource && lineResource->kind() == MyVoxel::Display_ResourceKind::Line, "ResourceManager.lineKind");
    runner.check(!manager.meshResource(lineId), "ResourceManager.lineRejectMeshQuery");

    if (lineResource)
    {
        runner.check(lineResource->vertexCount() == 4U, "ResourceManager.lineVertexCount");
        runner.check(lineResource->segmentCount() == 2U, "ResourceManager.lineSegmentCount");
        runner.check(lineResource->memoryByteSize() == lineResource->vertexCount() * sizeof(MyVoxel::Display_LineVertex),
                     "ResourceManager.lineMemory");
        runner.check(pointEqual(lineResource->localBounds().minimum(), MyMath::Vector3(0.0, 0.0, 0.0)),
                     "ResourceManager.lineBoundsMinimum");
        runner.check(pointEqual(lineResource->localBounds().maximum(), MyMath::Vector3(4.0, 1.0, 0.0)),
                     "ResourceManager.lineBoundsMaximum");
    }

    const std::size_t expectedMemory =
        (meshResource ? meshResource->memoryByteSize() : 0U) +
        (lineResource ? lineResource->memoryByteSize() : 0U);

    runner.check(manager.memoryByteSize() == expectedMemory, "ResourceManager.totalMemory");

    // manager.remove只释放管理器自己的RefPtr，外部已有引用必须继续保持资源生命周期。
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> retainedMesh = meshResource;
    runner.check(manager.remove(meshId), "ResourceManager.removeMesh");
    runner.check(!manager.contains(meshId), "ResourceManager.meshRemoved");
    runner.check(!manager.resource(meshId), "ResourceManager.removedLookupEmpty");
    runner.check(retainedMesh && retainedMesh->isValid(), "ResourceManager.externalRefSurvivesRemoval");
    runner.check(manager.resourceCount() == 1U, "ResourceManager.countAfterRemoval");
    runner.check(!manager.remove(meshId), "ResourceManager.removeMissingRejected");

    manager.clear();

    runner.check(manager.resourceCount() == 0U, "ResourceManager.clearCount");
    runner.check(manager.memoryByteSize() == 0U, "ResourceManager.clearMemory");
    runner.check(retainedMesh && retainedMesh->isValid(), "ResourceManager.externalRefSurvivesClear");
}

/// Display_Object基础状态

void testObjectState(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ObjectId meshObjectId =
        objectManager.createObject(MyVoxel::Display_ResourceKind::Mesh, MyVoxel::Display_ObjectUsage::Static);

    runner.check(meshObjectId != 0, "Object.meshCreate");
    runner.check(objectManager.contains(meshObjectId), "Object.meshContains");
    runner.check(objectManager.objectCount() == 1U, "Object.initialCount");

    MyVoxel::Display_Object object = objectManager.object(meshObjectId);

    runner.check(object.isValid(), "Object.emptyMeshValid");
    runner.check(object.objectId() == meshObjectId, "Object.identity");
    runner.check(object.resourceKind() == MyVoxel::Display_ResourceKind::Mesh, "Object.meshKind");
    runner.check(object.usage() == MyVoxel::Display_ObjectUsage::Static, "Object.initialUsage");
    runner.check(object.visible(), "Object.initialVisible");
    runner.check(object.stateVersion() == 1U, "Object.initialStateVersion");
    runner.check(object.partCount() == 0U, "Object.initialPartCount");

    runner.check(objectManager.setVisible(meshObjectId, false), "Object.setVisible");
    object = objectManager.object(meshObjectId);
    runner.check(!object.visible(), "Object.visibleChanged");
    runner.check(object.stateVersion() == 2U, "Object.visibleVersion");

    // 重复设置完全相同状态不得制造无意义版本。
    runner.check(objectManager.setVisible(meshObjectId, false), "Object.repeatVisibleAccepted");
    runner.check(objectManager.object(meshObjectId).stateVersion() == 2U, "Object.repeatVisibleNoVersionChange");

    const MyMath::Matrix4 translation = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    runner.check(objectManager.setLocalToWorld(meshObjectId, translation), "Object.setTransform");
    object = objectManager.object(meshObjectId);
    runner.check(object.localToWorld().isEqualTo(translation, 0.001), "Object.transformChanged");
    runner.check(object.stateVersion() == 3U, "Object.transformVersion");

    runner.check(objectManager.setLocalToWorld(meshObjectId, translation), "Object.repeatTransformAccepted");
    runner.check(objectManager.object(meshObjectId).stateVersion() == 3U, "Object.repeatTransformNoVersionChange");

    runner.check(objectManager.setUsage(meshObjectId, MyVoxel::Display_ObjectUsage::Dynamic), "Object.setUsage");
    object = objectManager.object(meshObjectId);
    runner.check(object.usage() == MyVoxel::Display_ObjectUsage::Dynamic, "Object.usageChanged");
    runner.check(object.stateVersion() == 4U, "Object.usageVersion");

    runner.check(objectManager.setUsage(meshObjectId, MyVoxel::Display_ObjectUsage::Dynamic), "Object.repeatUsageAccepted");
    runner.check(objectManager.object(meshObjectId).stateVersion() == 4U, "Object.repeatUsageNoVersionChange");

    runner.check(!objectManager.setLineWidth(meshObjectId, 2.0), "Object.meshRejectLineWidth");
    runner.check(objectManager.object(meshObjectId).stateVersion() == 4U, "Object.rejectedLineWidthNoVersionChange");

    runner.check(!objectManager.object(0), "Object.zeroLookupInvalid");
    runner.check(!objectManager.object(999999), "Object.missingLookupInvalid");
}

/// Display_Object资源分片与版本

void testObjectParts(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ResourceId meshId1 =
        resourceManager.createMeshResource(makeTriangleMesh(MyVoxel::Display_Color::redColor()));
    const MyVoxel::Display_ResourceId meshId2 =
        resourceManager.createMeshResource(makeSecondTriangleMesh(MyVoxel::Display_Color::blueColor()));
    const MyVoxel::Display_ResourceId lineId =
        resourceManager.createLineResource(makeLinePoints(), MyVoxel::Display_Color::greenColor());

    runner.check(meshId1 != 0 && meshId2 != 0 && lineId != 0, "ObjectParts.resourcesCreated");

    const MyVoxel::Display_ObjectId objectId =
        objectManager.createObject(MyVoxel::Display_ResourceKind::Mesh, MyVoxel::Display_ObjectUsage::Dynamic);

    runner.check(objectId != 0, "ObjectParts.objectCreated");
    runner.check(objectManager.setPart(objectId, 0, meshId1), "ObjectParts.addPart");

    MyVoxel::Display_Object object = objectManager.object(objectId);

    runner.check(object.partCount() == 1U, "ObjectParts.partCountOne");
    runner.check(object.hasPart(0), "ObjectParts.hasPart0");
    runner.check(object.partVersion(0) == 1U, "ObjectParts.initialVersion");
    runner.check(object.part(0).resourceId == meshId1, "ObjectParts.initialResource");
    runner.check(objectManager.partCount() == 1U, "ObjectParts.managerPartCount");

    // 同一个part重复设置同一个ResourceId应视为无变化。
    runner.check(objectManager.setPart(objectId, 0, meshId1), "ObjectParts.repeatResourceAccepted");
    object = objectManager.object(objectId);
    runner.check(object.partVersion(0) == 1U, "ObjectParts.repeatResourceNoVersionChange");

    // 替换资源必须推进该part版本。
    runner.check(objectManager.setPart(objectId, 0, meshId2), "ObjectParts.replaceResource");
    object = objectManager.object(objectId);
    runner.check(object.partVersion(0) == 2U, "ObjectParts.replaceVersion");
    runner.check(object.part(0).resourceId == meshId2, "ObjectParts.replacedResource");

    // Mesh对象不能接收Line资源。
    runner.check(!objectManager.setPart(objectId, 1, lineId), "ObjectParts.rejectWrongResourceKind");
    runner.check(!objectManager.object(objectId).hasPart(1), "ObjectParts.wrongKindNotInserted");

    // 新part从version=1开始。
    runner.check(objectManager.setPart(objectId, 7, meshId1), "ObjectParts.addPart7");
    object = objectManager.object(objectId);
    runner.check(object.partVersion(7) == 1U, "ObjectParts.part7InitialVersion");
    runner.check(object.partCount() == 2U, "ObjectParts.twoActiveParts");

    // 删除后活动分片消失，但历史版本继续保留并推进。
    runner.check(objectManager.removePart(objectId, 0), "ObjectParts.removePart0");
    object = objectManager.object(objectId);
    runner.check(!object.hasPart(0), "ObjectParts.part0Removed");
    runner.check(object.partVersion(0) == 3U, "ObjectParts.removeAdvancesVersion");
    runner.check(object.partCount() == 1U, "ObjectParts.onePartAfterRemove");
    runner.check(!objectManager.removePart(objectId, 0), "ObjectParts.repeatRemoveRejected");

    // 重建同一个partId必须继续使用历史版本，而不能重新从1开始。
    runner.check(objectManager.setPart(objectId, 0, meshId1), "ObjectParts.reAddPart0");
    object = objectManager.object(objectId);
    runner.check(object.partVersion(0) == 4U, "ObjectParts.reAddContinuesVersion");
    runner.check(object.hasPart(0), "ObjectParts.reAddActive");

    const std::uint64_t part0VersionBeforeClear = object.partVersion(0);
    const std::uint64_t part7VersionBeforeClear = object.partVersion(7);

    runner.check(objectManager.clearParts(objectId), "ObjectParts.clearParts");
    object = objectManager.object(objectId);

    runner.check(object.partCount() == 0U, "ObjectParts.clearPartCount");
    runner.check(object.partVersion(0) == part0VersionBeforeClear + 1U, "ObjectParts.clearAdvancesPart0");
    runner.check(object.partVersion(7) == part7VersionBeforeClear + 1U, "ObjectParts.clearAdvancesPart7");

    runner.check(objectManager.setPart(objectId, 7, meshId2), "ObjectParts.reAddPart7");
    object = objectManager.object(objectId);
    runner.check(object.partVersion(7) == part7VersionBeforeClear + 2U, "ObjectParts.part7VersionContinues");
}

/// ResourceManager与ObjectManager生命周期解耦

void testResourceObjectLifetime(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ResourceId resourceId =
        resourceManager.createMeshResource(makeTriangleMesh(MyVoxel::Display_Color::yellow()));

    const MyVoxel::Display_ObjectId objectId =
        objectManager.createObject(resourceId, MyVoxel::Display_ObjectUsage::Static, 0);

    runner.check(resourceId != 0 && objectId != 0, "Lifetime.objectCreatedFromResource");

    const MyVoxel::Display_Object beforeRemoval = objectManager.object(objectId);

    runner.check(beforeRemoval.isValid(), "Lifetime.objectInitiallyValid");
    runner.check(beforeRemoval.partCount() == 1U, "Lifetime.objectInitialPart");
    runner.check(beforeRemoval.part(0).resource && beforeRemoval.part(0).resource->isValid(), "Lifetime.objectOwnsResource");

    // 删除资源目录入口后，Display_Object持有的RefPtr必须继续保证资源有效。
    runner.check(resourceManager.remove(resourceId), "Lifetime.removeResourceDirectoryEntry");
    runner.check(!resourceManager.contains(resourceId), "Lifetime.resourceDirectoryEntryGone");

    const MyVoxel::Display_Object afterRemoval = objectManager.object(objectId);

    runner.check(afterRemoval.isValid(), "Lifetime.objectSurvivesResourceRemoval");
    runner.check(afterRemoval.part(0).resource && afterRemoval.part(0).resource->isValid(), "Lifetime.objectResourceStillAlive");
    runner.check(afterRemoval.part(0).resourceId == resourceId, "Lifetime.resourceIdentityPreserved");

    const MyVoxel::Display_ObjectSnapshot snapshot = objectManager.snapshot(objectId);

    runner.check(snapshot.isValid(), "Lifetime.snapshotAfterResourceRemoval");
    runner.check(snapshot.parts.size() == 1U, "Lifetime.snapshotPartPreserved");

    // ResourceId已经不再注册，因此不能用旧ID重新建立新的对象或Part。
    runner.check(objectManager.createObject(resourceId) == 0, "Lifetime.removedResourceCannotCreateObject");
    runner.check(!objectManager.setPart(objectId, 1, resourceId), "Lifetime.removedResourceCannotCreatePart");
}

/// Line Display_Object

void testLineObject(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ResourceId lineId =
        resourceManager.createLineResource(makeLinePoints(), MyVoxel::Display_Color::cyan());

    const MyVoxel::Display_ObjectId objectId =
        objectManager.createObject(lineId, MyVoxel::Display_ObjectUsage::Dynamic, 0);

    runner.check(lineId != 0 && objectId != 0, "LineObject.created");

    MyVoxel::Display_Object object = objectManager.object(objectId);

    runner.check(object.isValid(), "LineObject.valid");
    runner.check(object.resourceKind() == MyVoxel::Display_ResourceKind::Line, "LineObject.kind");
    runner.check(nearlyEqual(object.lineWidth(), 1.0), "LineObject.defaultWidth");
    runner.check(object.stateVersion() == 1U, "LineObject.initialStateVersion");

    runner.check(objectManager.setLineWidth(objectId, 2.5), "LineObject.setWidth");
    object = objectManager.object(objectId);

    runner.check(nearlyEqual(object.lineWidth(), 2.5), "LineObject.widthChanged");
    runner.check(object.stateVersion() == 2U, "LineObject.widthVersion");

    runner.check(objectManager.setLineWidth(objectId, 2.5), "LineObject.repeatWidthAccepted");
    runner.check(objectManager.object(objectId).stateVersion() == 2U, "LineObject.repeatWidthNoVersionChange");

    runner.check(!objectManager.setLineWidth(objectId, 0.0), "LineObject.rejectZeroWidth");
    runner.check(!objectManager.setLineWidth(objectId, -1.0), "LineObject.rejectNegativeWidth");
    runner.check(objectManager.object(objectId).stateVersion() == 2U, "LineObject.invalidWidthNoVersionChange");

    const MyMath::Matrix4 translation = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    runner.check(objectManager.setLocalToWorld(objectId, translation), "LineObject.setTransform");

    const MyVoxel::Display_ObjectSnapshot snapshot = objectManager.snapshot(objectId);

    runner.check(snapshot.isValid(), "LineObject.snapshotValid");
    runner.check(snapshot.resourceKind == MyVoxel::Display_ResourceKind::Line, "LineObject.snapshotKind");
    runner.check(snapshot.parts.size() == 1U, "LineObject.snapshotPartCount");
    runner.check(nearlyEqual(snapshot.lineWidth, 2.5), "LineObject.snapshotWidth");
    runner.check(pointEqual(snapshot.localBounds().minimum(), MyMath::Vector3(0.0, 0.0, 0.0)),
                 "LineObject.snapshotLocalMinimum");
    runner.check(pointEqual(snapshot.localBounds().maximum(), MyMath::Vector3(4.0, 1.0, 0.0)),
                 "LineObject.snapshotLocalMaximum");
    runner.check(pointEqual(snapshot.worldBounds().minimum(), MyMath::Vector3(10.0, 20.0, 30.0)),
                 "LineObject.snapshotWorldMinimum");
    runner.check(pointEqual(snapshot.worldBounds().maximum(), MyMath::Vector3(14.0, 21.0, 30.0)),
                 "LineObject.snapshotWorldMaximum");
}

/// Display_ObjectSnapshot

void testSnapshots(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ResourceId meshId =
        resourceManager.createMeshResource(makeTriangleMesh(MyVoxel::Display_Color::magenta()));
    const MyVoxel::Display_ResourceId lineId =
        resourceManager.createLineResource(makeLinePoints(), MyVoxel::Display_Color::white());

    const MyVoxel::Display_ObjectId meshObjectId =
        objectManager.createObject(meshId, MyVoxel::Display_ObjectUsage::Static, 0);
    const MyVoxel::Display_ObjectId lineObjectId =
        objectManager.createObject(lineId, MyVoxel::Display_ObjectUsage::Dynamic, 3);

    runner.check(meshObjectId != 0 && lineObjectId != 0, "Snapshot.objectsCreated");
    runner.check(objectManager.objectCount() == 2U, "Snapshot.objectCount");
    runner.check(objectManager.partCount() == 2U, "Snapshot.partCount");

    runner.check(objectManager.setVisible(meshObjectId, false), "Snapshot.meshVisibilityChanged");
    runner.check(objectManager.setLineWidth(lineObjectId, 3.0), "Snapshot.lineWidthChanged");

    const MyVoxel::Display_ObjectSnapshot meshSnapshot = objectManager.snapshot(meshObjectId);
    const MyVoxel::Display_ObjectSnapshot lineSnapshot = objectManager.snapshot(lineObjectId);

    runner.check(meshSnapshot.isValid(), "Snapshot.meshValid");
    runner.check(lineSnapshot.isValid(), "Snapshot.lineValid");
    runner.check(meshSnapshot.objectId == meshObjectId, "Snapshot.meshIdentity");
    runner.check(lineSnapshot.objectId == lineObjectId, "Snapshot.lineIdentity");
    runner.check(meshSnapshot.resourceKind == MyVoxel::Display_ResourceKind::Mesh, "Snapshot.meshKind");
    runner.check(lineSnapshot.resourceKind == MyVoxel::Display_ResourceKind::Line, "Snapshot.lineKind");
    runner.check(!meshSnapshot.visible, "Snapshot.meshVisibleState");
    runner.check(nearlyEqual(lineSnapshot.lineWidth, 3.0), "Snapshot.lineWidthState");

    runner.check(meshSnapshot.parts.size() == 1U, "Snapshot.meshPartCount");
    runner.check(lineSnapshot.parts.size() == 1U, "Snapshot.linePartCount");

    if (!meshSnapshot.parts.empty())
    {
        runner.check(meshSnapshot.parts[0].partId == 0, "Snapshot.meshPartId");
        runner.check(meshSnapshot.parts[0].version == 1U, "Snapshot.meshPartVersion");
        runner.check(meshSnapshot.parts[0].resourceId == meshId, "Snapshot.meshResourceId");
    }

    if (!lineSnapshot.parts.empty())
    {
        runner.check(lineSnapshot.parts[0].partId == 3, "Snapshot.linePartId");
        runner.check(lineSnapshot.parts[0].version == 1U, "Snapshot.linePartVersion");
        runner.check(lineSnapshot.parts[0].resourceId == lineId, "Snapshot.lineResourceId");
    }

    const std::vector<MyVoxel::Display_ObjectSnapshot> snapshots = objectManager.snapshots();

    runner.check(snapshots.size() == 2U, "Snapshot.allCount");
    runner.check(snapshots[0].isValid() && snapshots[1].isValid(), "Snapshot.allValid");

    runner.check(!objectManager.snapshot(0).isValid(), "Snapshot.zeroIdInvalid");
    runner.check(!objectManager.snapshot(999999).isValid(), "Snapshot.missingIdInvalid");

    const std::size_t meshMemory = resourceManager.meshResource(meshId)->memoryByteSize();
    runner.check(meshSnapshot.memoryByteSize() == meshMemory, "Snapshot.memoryByteSize");
}

/// Display_ObjectUpdate

void testUpdates(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;

    const MyVoxel::Display_ResourceId meshId =
        resourceManager.createMeshResource(makeTriangleMesh(MyVoxel::Display_Color::blueColor()));
    const MyVoxel::Display_ResourceId lineId =
        resourceManager.createLineResource(makeLinePoints(), MyVoxel::Display_Color::yellow());

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> meshResource = resourceManager.meshResource(meshId);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> lineResource = resourceManager.lineResource(lineId);

    MyVoxel::Display_ObjectPartUpdate defaultPartUpdate;
    runner.check(!defaultPartUpdate.isValid(), "Update.defaultPartInvalid");

    const MyVoxel::Display_ObjectPartUpdate meshReplacement =
        MyVoxel::Display_ObjectPartUpdate::replacement(5, 1, meshId, meshResource);

    runner.check(meshReplacement.isValid(), "Update.replacementValid");
    runner.check(meshReplacement.operation == MyVoxel::Display_ObjectPartOperation::Replace, "Update.replacementOperation");
    runner.check(meshReplacement.partId == 5, "Update.replacementPartId");
    runner.check(meshReplacement.version == 1U, "Update.replacementVersion");
    runner.check(meshReplacement.resourceId == meshId, "Update.replacementResourceId");

    const MyVoxel::Display_ObjectPartUpdate removal =
        MyVoxel::Display_ObjectPartUpdate::removal(5, 2);

    runner.check(removal.isValid(), "Update.removalValid");
    runner.check(removal.operation == MyVoxel::Display_ObjectPartOperation::Remove, "Update.removalOperation");
    runner.check(removal.resourceId == 0, "Update.removalNoResourceId");
    runner.check(!removal.resource, "Update.removalNoResource");

    MyVoxel::Display_ObjectPartsUpdate partsUpdate;
    partsUpdate.objectId = 100;
    partsUpdate.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    partsUpdate.parts.push_back(meshReplacement);

    runner.check(partsUpdate.isValid(), "Update.partsValid");

    MyVoxel::Display_ObjectPartsUpdate wrongKindUpdate;
    wrongKindUpdate.objectId = 101;
    wrongKindUpdate.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    wrongKindUpdate.parts.push_back(
        MyVoxel::Display_ObjectPartUpdate::replacement(0, 1, lineId, lineResource));

    runner.check(!wrongKindUpdate.isValid(), "Update.partsRejectWrongKind");

    MyVoxel::Display_ObjectPartsUpdate duplicateUpdate;
    duplicateUpdate.objectId = 102;
    duplicateUpdate.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    duplicateUpdate.parts.push_back(meshReplacement);
    duplicateUpdate.parts.push_back(MyVoxel::Display_ObjectPartUpdate::removal(5, 2));

    runner.check(!duplicateUpdate.isValid(), "Update.partsRejectDuplicatePart");

    MyVoxel::Display_ObjectPartsUpdate emptyUpdate;
    emptyUpdate.objectId = 103;
    emptyUpdate.resourceKind = MyVoxel::Display_ResourceKind::Mesh;

    runner.check(!emptyUpdate.isValid(), "Update.partsRejectEmpty");

    MyVoxel::Display_ObjectStateUpdate stateUpdate;
    runner.check(!stateUpdate.isValid(), "Update.defaultStateInvalid");

    stateUpdate.objectId = 200;
    stateUpdate.stateVersion = 2;
    stateUpdate.hasVisible = true;
    stateUpdate.visible = false;

    runner.check(stateUpdate.isValid(), "Update.visibleStateValid");

    MyVoxel::Display_ObjectStateUpdate transformUpdate;
    transformUpdate.objectId = 201;
    transformUpdate.stateVersion = 3;
    transformUpdate.hasLocalToWorld = true;
    transformUpdate.localToWorld = MyMath::Matrix4::fromTranslation(MyMath::Vector3(1.0, 2.0, 3.0));

    runner.check(transformUpdate.isValid(), "Update.transformStateValid");

    MyVoxel::Display_ObjectStateUpdate widthUpdate;
    widthUpdate.objectId = 202;
    widthUpdate.stateVersion = 4;
    widthUpdate.hasLineWidth = true;
    widthUpdate.lineWidth = 2.0f;

    runner.check(widthUpdate.isValid(), "Update.widthStateValid");

    widthUpdate.lineWidth = 0.0f;
    runner.check(!widthUpdate.isValid(), "Update.rejectZeroWidth");

    MyVoxel::Display_ObjectStateUpdate noVersionUpdate;
    noVersionUpdate.objectId = 203;
    noVersionUpdate.hasVisible = true;

    runner.check(!noVersionUpdate.isValid(), "Update.rejectZeroStateVersion");
}

/// ObjectManager释放行为

void testObjectRemoval(TestRunner& runner)
{
    MyVoxel::Display_ResourceManager resourceManager;
    MyVoxel::Display_ObjectManager objectManager(resourceManager);

    const MyVoxel::Display_ResourceId meshId =
        resourceManager.createMeshResource(makeTriangleMesh(MyVoxel::Display_Color::gray()));
    const MyVoxel::Display_ResourceId lineId =
        resourceManager.createLineResource(makeLinePoints(), MyVoxel::Display_Color::white());

    const MyVoxel::Display_ObjectId meshObjectId = objectManager.createObject(meshId);
    const MyVoxel::Display_ObjectId lineObjectId = objectManager.createObject(lineId);

    runner.check(objectManager.objectCount() == 2U, "Removal.initialObjectCount");
    runner.check(objectManager.partCount() == 2U, "Removal.initialPartCount");
    runner.check(resourceManager.resourceCount() == 2U, "Removal.initialResourceCount");

    runner.check(objectManager.remove(meshObjectId), "Removal.removeMeshObject");
    runner.check(!objectManager.contains(meshObjectId), "Removal.meshObjectGone");
    runner.check(objectManager.objectCount() == 1U, "Removal.objectCountAfterRemove");
    runner.check(objectManager.partCount() == 1U, "Removal.partCountAfterRemove");

    // 删除Display_Object不能隐式删除Display_ResourceManager中的共享资源。
    runner.check(resourceManager.contains(meshId), "Removal.resourceSurvivesObjectRemove");
    runner.check(resourceManager.resourceCount() == 2U, "Removal.resourceCountUnchanged");

    runner.check(!objectManager.remove(meshObjectId), "Removal.repeatRemoveRejected");

    objectManager.clear();

    runner.check(objectManager.objectCount() == 0U, "Removal.clearObjectCount");
    runner.check(objectManager.partCount() == 0U, "Removal.clearPartCount");
    runner.check(resourceManager.resourceCount() == 2U, "Removal.resourcesSurviveObjectClear");

    runner.check(resourceManager.contains(lineId), "Removal.lineResourceStillRegistered");
    runner.check(!objectManager.contains(lineObjectId), "Removal.lineObjectCleared");
}

}

int main()
{
    std::cout << "============================================================" << std::endl;
    std::cout << "MyVoxel display core test" << std::endl;
    std::cout << "============================================================" << std::endl;

    TestRunner runner;

    testResourceManager(runner);
    testObjectState(runner);
    testObjectParts(runner);
    testResourceObjectLifetime(runner);
    testLineObject(runner);
    testSnapshots(runner);
    testUpdates(runner);
    testObjectRemoval(runner);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << runner.passed() << std::endl;
    std::cout << "Failed: " << runner.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return runner.failed() == 0 ? 0 : 1;
}