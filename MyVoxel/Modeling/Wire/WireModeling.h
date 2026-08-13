#ifndef MYVOXEL_MODELING_WIRE_WIREMODELING_H
#define MYVOXEL_MODELING_WIRE_WIREMODELING_H

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Instance/Curve.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"
#include "MyVoxel/Instance/Wire.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Wire创建

// 使用已经按连接顺序排列的Topology_Edge创建开放或闭合Topology_Wire，相邻Edge必须共享同一Topology_Vertex身份。
Topology_Wire createWire(const std::vector<Topology_Edge>& edges);
// 使用局部XY平面顶点序列创建开放折线Topology_Wire，相邻线段共享同一Topology_Vertex。
Topology_Wire createPolyline(const std::vector<MyMath::Vector3>& points);
// 使用不重复首点的局部XY平面顶点序列创建闭合多边形Topology_Wire，并保留输入顶点顺序定义的方向。
Topology_Wire createPolygon(const std::vector<MyMath::Vector3>& points);
// 创建以局部原点为中心、边平行于XY坐标轴且逆时针的闭合矩形Topology_Wire。
Topology_Wire createRectangle(double sizeX, double sizeY);
// 创建以指定局部XY平面点为中心、边平行于XY坐标轴且逆时针的闭合矩形Topology_Wire。
Topology_Wire createRectangle(const MyMath::Vector3& center, double sizeX, double sizeY);
// 创建以局部原点为圆心且逆时针的单Edge闭合圆Topology_Wire。
Topology_Wire createCircle(double radius);
// 创建以指定局部XY平面点为圆心且逆时针的单Edge闭合圆Topology_Wire。
Topology_Wire createCircle(const MyMath::Vector3& center, double radius);

/// 空间Wire实例创建

// 使用单位变换创建通用Wire实例。
Wire makeWire(const std::vector<Topology_Edge>& edges);
// 使用指定可逆仿射变换创建通用Wire实例。
Wire makeWire(const std::vector<Topology_Edge>& edges, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建开放折线Wire实例。
Wire makePolyline(const std::vector<MyMath::Vector3>& points);
// 使用指定可逆仿射变换创建开放折线Wire实例。
Wire makePolyline(const std::vector<MyMath::Vector3>& points, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建闭合多边形Wire实例。
Wire makePolygon(const std::vector<MyMath::Vector3>& points);
// 使用指定可逆仿射变换创建闭合多边形Wire实例。
Wire makePolygon(const std::vector<MyMath::Vector3>& points, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准矩形Wire实例。
Wire makeRectangle(double sizeX, double sizeY);
// 使用指定可逆仿射变换创建标准矩形Wire实例。
Wire makeRectangle(double sizeX, double sizeY, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建指定局部中心的矩形Wire实例。
Wire makeRectangle(const MyMath::Vector3& center, double sizeX, double sizeY);
// 使用指定可逆仿射变换创建指定局部中心的矩形Wire实例。
Wire makeRectangle(const MyMath::Vector3& center, double sizeX, double sizeY, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建标准圆Wire实例。
Wire makeCircle(double radius);
// 使用指定可逆仿射变换创建标准圆Wire实例。
Wire makeCircle(double radius, const MyMath::Matrix4& localToWorld);
// 使用单位变换创建指定局部圆心的圆Wire实例。
Wire makeCircle(const MyMath::Vector3& center, double radius);
// 使用指定可逆仿射变换创建指定局部圆心的圆Wire实例。
Wire makeCircle(const MyMath::Vector3& center, double radius, const MyMath::Matrix4& localToWorld);

}
}

#endif // MYVOXEL_MODELING_WIRE_WIREMODELING_H