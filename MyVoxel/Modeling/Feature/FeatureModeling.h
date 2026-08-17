#ifndef MYVOXEL_MODELING_FEATURE_FEATUREMODELING_H
#define MYVOXEL_MODELING_FEATURE_FEATUREMODELING_H

#include "MyVoxel/Modeling/Feature/FeatureTrace.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"

namespace MyVoxel
{
namespace Modeling
{

/// 特征拓扑创建

// 将至少包含两个点的FeatureTrace转换为单条Topology_Edge，全部轨迹点作为Geometry_Polyline几何采样点。
Topology_Edge createFeatureEdge(const FeatureTrace& trace);

}
}

#endif // MYVOXEL_MODELING_FEATURE_FEATUREMODELING_H