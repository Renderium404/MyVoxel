#include "Display_ShapeBuilder.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{

/// 支持判断

bool Display_ShapeBuilder::supports(const Topology_Shape& topology)
{
    return ShapeMesher::supports(topology);
}

/// 资源创建

Display_ResourceId Display_ShapeBuilder::createResource(const Topology_Shape& topology,
                                                        Display_ResourceManager& resourceManager,
                                                        const Display_Color& color,
                                                        const ShapeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "Display_ShapeBuilder requires a valid Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(ShapeMesher::supports(topology), "Display_ShapeBuilder requires a supported Topology_Shape.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Display_ShapeBuilder requires a valid display color.");

    if (!topology.isValid() || !ShapeMesher::supports(topology) || !color.isValid())
    {
        return 0;
    }

    const Mesh mesh = ShapeMesher::build(topology, color, options);

    if (!mesh.isRenderable())
    {
        return 0;
    }

    return resourceManager.createMeshResource(mesh);
}

/// 对象创建

Display_ObjectId Display_ShapeBuilder::createObject(const Shape& shape, Display_ResourceId resourceId,
                                                    Display_ObjectManager& objectManager, Display_ObjectUsage usage)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Display_ShapeBuilder requires a valid Shape instance.");

    if (!shape.isValid() || resourceId == 0)
    {
        return 0;
    }

    const Display_ObjectId objectId = objectManager.createObject(resourceId, usage, 0);

    if (objectId == 0)
    {
        return 0;
    }

    const Display_Object objectValue = objectManager.object(objectId);

    if (!objectValue.isValid() || objectValue.resourceKind() != Display_ResourceKind::Mesh)
    {
        objectManager.remove(objectId);
        return 0;
    }

    if (!objectManager.setLocalToWorld(objectId, shape.localToWorld()))
    {
        objectManager.remove(objectId);
        return 0;
    }

    return objectId;
}

}