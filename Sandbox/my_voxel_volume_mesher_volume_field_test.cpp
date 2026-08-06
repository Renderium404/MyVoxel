#include <cstdlib>
#include <iostream>
#include <string>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Meshing/VolumeMeshingWorkspace.h"
#include "MyVoxel/Volume/VolumeFieldBuilder.h"
#include "MyVoxel/Volume/VolumeFieldView.h"

namespace
{

int g_passed = 0;
int g_failed = 0;

// 记录一项测试结果。
void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[通过] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[失败] " << name << std::endl;
    }
}

// 判断两个网格是否具有一致的基础拓扑数量。
bool sameMeshCounts(const MyVoxel::Geometry::Mesh& first, const MyVoxel::Geometry::Mesh& second)
{
    return first.vertexCount() == second.vertexCount() && first.triangleCount() == second.triangleCount();
}

}

int main()
{
    using namespace MyVoxel;
    using namespace MyVoxel::Meshing;

    std::cout << "开始VoxelShape距离场网格化测试" << std::endl << std::endl;

    VolumeFieldBuildOptions fieldOptions;
    fieldOptions.exteriorBandWidth = 2.0;
    fieldOptions.interiorBandWidth = 2.0;

    VoxelShape emptyShape(4.0, 2);
    VolumeFieldBuilder::build(emptyShape, fieldOptions);
    VolumeFieldView emptyView(emptyShape);
    check(emptyView.isCurrent(), "空Shape距离场有效");
    check(!VolumeMesher::defaultSampleRange(emptyView).isValid(), "空距离场没有自动网格化范围");
    const Geometry::Mesh emptyMesh = VolumeMesher::build(emptyView);
    check(emptyMesh.isValid() && emptyMesh.isEmpty(), "空距离场生成有效空网格");

    VoxelShape shape(4.0, 2);
    const VoxelCellAddress materialAddress(VoxelCellIndex(0, 0, 0), 2);
    {
        VoxelShapeSession session = shape.session(2);
        check(session.setState(materialAddress, VoxelState::Material), "创建单个最高层材料体素");
    }

    VolumeFieldBuilder::build(shape, fieldOptions);
    VolumeFieldView view(shape);
    check(view.isCurrent(), "单体素距离场构建后有效");

    const VoxelCellRange sampleRange = VolumeMesher::defaultSampleRange(view);
    check(sampleRange.isValid() && sampleRange.level == view.sampleLevel(), "自动网格化范围使用距离场最高层级");
    check(sampleRange.contains(materialAddress.index), "自动网格化范围包含材料样本");
    check(sampleRange.countX() >= 6 && sampleRange.countY() >= 6 && sampleRange.countZ() >= 6,
          "自动网格化范围包含距离块和一层Halo");

    VolumeMeshingOptions standardOptions;
    standardOptions.mode = VolumeMeshingMode::StandardSurfaceNets;
    standardOptions.topologyMode = VolumeTopologyMode::StandardDynamic;
    standardOptions.samplingMode = VolumeSamplingMode::StandardOnDemand;
    standardOptions.signMode = VolumeSignMode::StandardDirect;
    const Geometry::Mesh standardMesh = VolumeMesher::build(view, standardOptions);
    check(standardMesh.isValid() && !standardMesh.isEmpty(), "标准路径从VolumeField生成有效非空网格");
    check(standardMesh.vertexCount() > 0 && standardMesh.triangleCount() > 0, "标准路径生成顶点和三角形");

    const Geometry::Mesh fastMesh = VolumeMesher::build(view);
    check(fastMesh.isValid() && !fastMesh.isEmpty(), "快速路径从VolumeField生成有效非空网格");
    check(sameMeshCounts(standardMesh, fastMesh), "标准路径与快速路径基础拓扑数量一致");

    const Geometry::Mesh explicitRangeMesh = VolumeMesher::build(view, sampleRange);
    check(sameMeshCounts(fastMesh, explicitRangeMesh), "显式范围与自动范围网格化结果数量一致");

    VolumeMeshingWorkspace workspace(sampleRange);
    const Geometry::Mesh workspaceMesh = VolumeMesher::buildWithWorkspace(view, workspace);
    check(workspaceMesh.isValid() && sameMeshCounts(fastMesh, workspaceMesh), "外部工作区生成结果与内部工作区一致");
    check(workspace.hasCompleteSampleSigns() && workspace.hasCompleteCellSignMasks(), "快速路径完成采样和单元符号预计算");
    check(workspace.activeCellCount() > 0, "工作区记录活动表面单元");

    const Geometry::Mesh reusedWorkspaceMesh = VolumeMesher::buildWithWorkspace(view, workspace);
    check(reusedWorkspaceMesh.isValid() && sameMeshCounts(workspaceMesh, reusedWorkspaceMesh), "同一工作区可以重复用于范围一致的距离场");

    VolumeMeshingOptions featureOptions;
    featureOptions.mode = VolumeMeshingMode::FeatureSensitiveSurfaceNets;
    const Geometry::Mesh featureMesh = VolumeMesher::buildWithWorkspace(view, workspace, featureOptions);
    check(featureMesh.isValid() && !featureMesh.isEmpty(), "特征敏感路径从VolumeField生成有效网格");
    check(featureMesh.triangleCount() == fastMesh.triangleCount(), "特征敏感路径保持表面三角形拓扑数量");
    check(workspace.computedGradientCount() > 0, "特征敏感快速路径计算并缓存距离梯度");

    const std::size_t vertexCountBeforeTransform = fastMesh.vertexCount();
    const std::size_t triangleCountBeforeTransform = fastMesh.triangleCount();
    shape.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -2.0, 3.0)));
    const Geometry::Mesh transformedShapeMesh = VolumeMesher::build(view);
    check(transformedShapeMesh.vertexCount() == vertexCountBeforeTransform &&
          transformedShapeMesh.triangleCount() == triangleCountBeforeTransform,
          "Shape刚体变换不影响局部距离场网格拓扑");

    {
        VoxelShapeSession session = shape.session(2);
        check(session.setState(materialAddress, VoxelState::Empty), "删除单个材料体素");
    }
    check(!view.isCurrent(), "材料修改后既有距离场视图失效");

    VolumeFieldBuilder::build(shape, fieldOptions);
    check(view.isCurrent(), "全量重建后既有距离场视图恢复同步");
    const Geometry::Mesh rebuiltEmptyMesh = VolumeMesher::build(view);
    check(rebuiltEmptyMesh.isValid() && rebuiltEmptyMesh.isEmpty(), "删除全部材料后距离场重新生成空网格");

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}