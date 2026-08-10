#include "Display_WireBuilder.h"

#include <limits>

#include "MyVoxel/Display/Builder/Display_CurveBuilder.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Instance/Wire.h"
#include "MyVoxel/Topology/Wire/Topology_Wire.h"

namespace MyVoxel
{

/// 支持判断

bool Display_WireBuilder::supports(const Topology_Wire& wire)
{
    if (!wire.isValid() || wire.edgeCount() == 0)
    {
        return false;
    }

    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        if (!Display_CurveBuilder::supports(wire.edge(index)))
        {
            return false;
        }
    }

    return true;
}

/// 资源创建

std::vector<Display_ResourceId> Display_WireBuilder::createResources(const Topology_Wire& wire,
                                                                     Display_ResourceManager& resourceManager,
                                                                     const Display_Color& color,
                                                                     const Display_CurveBuildOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(supports(wire), "Display_WireBuilder requires a supported valid Topology_Wire.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Display_WireBuilder requires a valid display color.");
    MYVOXEL_ASSERT_MESSAGE(options.isValid(), "Display_WireBuilder requires valid curve build options.");

    std::vector<Display_ResourceId> result;

    if (!supports(wire) || !color.isValid() || !options.isValid())
    {
        return result;
    }

    result.reserve(wire.edgeCount());

    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        const Display_ResourceId resourceId = Display_CurveBuilder::createResource(wire.edge(index), resourceManager, color, options);

        if (resourceId == 0)
        {
            for (std::size_t cleanupIndex = 0; cleanupIndex < result.size(); ++cleanupIndex)
            {
                resourceManager.remove(result[cleanupIndex]);
            }

            result.clear();
            return result;
        }

        result.push_back(resourceId);
    }

    return result;
}

/// 对象创建

Display_ObjectId Display_WireBuilder::createObject(const Wire& wire, const std::vector<Display_ResourceId>& resourceIds,
                                                   Display_ObjectManager& objectManager, Display_ObjectUsage usage,
                                                   double lineWidth)
{
    MYVOXEL_ASSERT_MESSAGE(wire.isValid(), "Display_WireBuilder requires a valid Wire instance.");
    MYVOXEL_ASSERT_MESSAGE(resourceIds.size() == wire.topology().edgeCount(),
                           "Display_WireBuilder requires exactly one resource for each Wire Edge.");

    if (!wire.isValid() || resourceIds.empty() || resourceIds.size() != wire.topology().edgeCount())
    {
        return 0;
    }

    const std::size_t maximumPartId = static_cast<std::size_t>((std::numeric_limits<Display_ObjectPartId>::max)());

    if (resourceIds.size() - 1 > maximumPartId)
    {
        return 0;
    }

    const Display_ObjectId objectId = objectManager.createObject(Display_ResourceKind::Line, usage);

    if (objectId == 0)
    {
        return 0;
    }

    if (!objectManager.setLocalToWorld(objectId, wire.localToWorld()) || !objectManager.setLineWidth(objectId, lineWidth))
    {
        objectManager.remove(objectId);
        return 0;
    }

    for (std::size_t index = 0; index < resourceIds.size(); ++index)
    {
        if (!objectManager.setPart(objectId, static_cast<Display_ObjectPartId>(index), resourceIds[index]))
        {
            objectManager.remove(objectId);
            return 0;
        }
    }

    return objectId;
}

}