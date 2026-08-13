#include "Topology_Object.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_Object::Topology_Object()
    : m_orientation(Topology_Orientation::Forward)
{
}

Topology_Object::Topology_Object(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation)
    : m_object(object)
    , m_orientation(orientation)
{
    MYVOXEL_ASSERT_MESSAGE(object, "Topology_Object requires a non-null Topology_TObject.");

    if (!object)
    {
        m_orientation = Topology_Orientation::Forward;
    }
}

/// 状态判断

bool Topology_Object::isValid() const
{
    return static_cast<bool>(m_object);
}

bool Topology_Object::isNull() const
{
    return !m_object;
}

Topology_Object::operator bool() const
{
    return isValid();
}

/// 拓扑身份

bool Topology_Object::isSame(const Topology_Object& other) const
{
    return m_object && m_object.get() == other.m_object.get();
}

/// 使用方向

Topology_Orientation Topology_Object::orientation() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the orientation of an invalid Topology_Object.");
    return m_orientation;
}

bool Topology_Object::isForward() const
{
    return isValid() && m_orientation == Topology_Orientation::Forward;
}

bool Topology_Object::isReversed() const
{
    return isValid() && m_orientation == Topology_Orientation::Reversed;
}

/// 派生类支持

const Foundation::RefPtr<const Topology_TObject>& Topology_Object::tObject() const
{
    return m_object;
}

Topology_Orientation Topology_Object::reversedOrientation() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse the orientation of an invalid Topology_Object.");
    return oppositeOrientation(m_orientation);
}

}
