#ifndef MYVOXEL_DISPLAY_LINE_DISPLAY_LINESNAPSHOT_H
#define MYVOXEL_DISPLAY_LINE_DISPLAY_LINESNAPSHOT_H

#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 保存完整线对象快照中的一个不可变非空分片。
struct Display_LinePartSnapshot
{
    Display_LinePartSnapshot();
    Display_LinePartSnapshot(Display_LinePartId partIdValue, std::uint64_t versionValue,
                             const Foundation::RefPtr<const Display_LineResource>& resourceValue);

    // 判断分片版本和显示资源是否完整有效。
    bool isValid() const;

    Display_LinePartId partId; // 对象内部稳定线分片标识，零值允许作为普通对象唯一分片。
    std::uint64_t version; // 当前分片单调递增版本，零值无效。
    Foundation::RefPtr<const Display_LineResource> resource; // 当前分片不可变CPU线资源。
};

// 保存完整替换一个线显示对象所需的前端快照。
struct Display_LineObjectSnapshot
{
    Display_LineObjectSnapshot();

    // 判断对象标识、变换和全部非重复分片是否有效。
    bool isValid() const;
    // 返回全部非空分片形成的局部轴对齐包围盒。
    Bounds3 localBounds() const;
    // 返回全部分片线段数量。
    std::size_t segmentCount() const;

    Display_LineObjectId objectId; // 全局线显示对象标识，零值无效。
    Display_LineUsage usage; // 当前对象GPU缓冲的预期更新频率。
    MyMath::Matrix4 localToWorld; // 对象局部空间到显示世界空间的可逆仿射变换。
    bool visible; // 当前对象是否参与绘制。
    float width; // 期望OpenGL线宽，必须为正有限值；实际支持范围由显示后端决定。
    std::vector<Display_LinePartSnapshot> parts; // 当前对象全部非空线分片。
};

}

#endif // MYVOXEL_DISPLAY_LINE_DISPLAY_LINESNAPSHOT_H
