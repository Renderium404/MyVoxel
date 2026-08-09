#include "Instance_Object.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Instance_Object::Instance_Object()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_placementValid(true)
{
}

Instance_Object::Instance_Object(const MyMath::Matrix4& localToWorld)
    : m_localToWorld(localToWorld)
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_placementValid(false)
{
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "Instance_Object transform must be affine.");

    if (!localToWorld.isAffine())
    {
        return;
    }

    m_placementValid = localToWorld.inverted(m_worldToLocal);
    MYVOXEL_ASSERT_MESSAGE(m_placementValid, "Instance_Object transform must be invertible.");
}

Instance_Object::~Instance_Object()
{
}

/// 空间放置

bool Instance_Object::isPlacementValid() const
{
    return m_placementValid;
}

const MyMath::Matrix4& Instance_Object::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isPlacementValid(), "Cannot access the transform of an invalid Instance_Object placement.");
    return m_localToWorld;
}

const MyMath::Matrix4& Instance_Object::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isPlacementValid(), "Cannot access the inverse transform of an invalid Instance_Object placement.");
    return m_worldToLocal;
}

}
