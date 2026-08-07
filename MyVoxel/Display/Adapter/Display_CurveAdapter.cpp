#include "Display_CurveAdapter.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

Display_LineObjectSnapshot Display_CurveAdapter::buildSnapshot(Display_LineObjectId objectId, const Curve& curve,
                                                               const Display_Color& color, float width,
                                                               const CurveDiscretizationOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(objectId != 0, "Curve display adapter requires a non-zero object id.");
    MYVOXEL_ASSERT_MESSAGE(curve.isValid(), "Curve display adapter requires a valid Curve.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Curve display adapter requires a valid color.");
    MYVOXEL_ASSERT_MESSAGE(options.isValid(), "Curve display adapter requires valid discretization options.");

    Display_LineObjectSnapshot snapshot;

    if (objectId == 0 || !curve.isValid() || !color.isValid() || !options.isValid())
    {
        return snapshot;
    }

    const std::vector<MyMath::Vector3> points = CurveDiscretizer::buildSegments(curve.topology(), options);
    Foundation::RefPtr<Display_LineResource> mutableResource = Foundation::makeRef<Display_LineResource>(points, color);
    Foundation::RefPtr<const Display_LineResource> resource = mutableResource;

    snapshot.objectId = objectId;
    snapshot.usage = Display_LineUsage::Static;
    snapshot.localToWorld = curve.localToWorld();
    snapshot.visible = true;
    snapshot.width = width;
    snapshot.parts.push_back(Display_LinePartSnapshot(0, 1, resource));

    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "Curve display adapter produced an invalid line snapshot.");
    return snapshot;
}

}
