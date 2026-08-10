#include "Display_LineResource.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Display_LineResource::Display_LineResource(const std::vector<MyMath::Vector3>& points, const Display_Color& color)
    : m_segmentCount(0)
    , m_valid(false)
{
    const bool validInput = color.isValid() && points.size() >= 2 && (points.size() % 2) == 0;
    MYVOXEL_ASSERT_MESSAGE(validInput, "Display line resource requires valid color and a non-empty even point sequence.");

    if (!validInput)
    {
        return;
    }

    m_vertices.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        if (!points[index].isFinite())
        {
            return;
        }

        m_vertices.push_back(Display_LineVertex(points[index].x(), points[index].y(), points[index].z(), color));
        m_localBounds.include(points[index]);
    }

    m_segmentCount = points.size() / 2;
    m_valid = m_segmentCount > 0 && m_vertices.size() == m_segmentCount * 2 && m_localBounds.isValid();
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Display line resource construction produced inconsistent data.");
}



/// 资源属性

Display_ResourceKind Display_LineResource::kind() const
{
    return Display_ResourceKind::Line;
}

bool Display_LineResource::isValid() const
{
    return m_valid;
}

/// 资源数据

const std::vector<Display_LineVertex>& Display_LineResource::vertices() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an invalid display line resource.");
    return m_vertices;
}

std::size_t Display_LineResource::vertexCount() const
{
    return m_vertices.size();
}

std::size_t Display_LineResource::segmentCount() const
{
    return m_segmentCount;
}

const Bounds3& Display_LineResource::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access bounds of an invalid display line resource.");
    return m_localBounds;
}

std::size_t Display_LineResource::memoryByteSize() const
{
    return m_vertices.size() * sizeof(Display_LineVertex);
}

}
