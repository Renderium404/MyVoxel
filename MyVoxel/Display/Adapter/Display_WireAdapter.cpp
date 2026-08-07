#include "Display_WireAdapter.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

Display_LineObjectSnapshot Display_WireAdapter::buildSnapshot(Display_LineObjectId objectId, const Wire& wire,
                                                              const Display_Color& color, float width,
                                                              const CurveDiscretizationOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(objectId != 0, "Wire display adapter requires a non-zero object id.");
    MYVOXEL_ASSERT_MESSAGE(wire.isValid(), "Wire display adapter requires a valid Wire.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Wire display adapter requires a valid color.");
    MYVOXEL_ASSERT_MESSAGE(options.isValid(), "Wire display adapter requires valid discretization options.");

    Display_LineObjectSnapshot snapshot;

    if (objectId == 0 || !wire.isValid() || !color.isValid() || !options.isValid())
    {
        return snapshot;
    }

    snapshot.objectId = objectId;
    snapshot.usage = Display_LineUsage::Static;
    snapshot.localToWorld = wire.localToWorld();
    snapshot.visible = true;
    snapshot.width = width;
    snapshot.parts.reserve(wire.curveCount());

    for (std::size_t index = 0; index < wire.curveCount(); ++index)
    {
        const std::vector<MyMath::Vector3> points = CurveDiscretizer::buildSegments(wire.localCurve(index), options);
        Foundation::RefPtr<Display_LineResource> mutableResource = Foundation::makeRef<Display_LineResource>(points, color);
        Foundation::RefPtr<const Display_LineResource> resource = mutableResource;
        snapshot.parts.push_back(Display_LinePartSnapshot(static_cast<Display_LinePartId>(index + 1), 1, resource));
    }

    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "Wire display adapter produced an invalid line snapshot.");
    return snapshot;
}

}
