#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Operation/ShapeBooleanOperation.h"

namespace
{

std::size_t g_passed = 0;
std::size_t g_failed = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

}

int main()
{
    const MyVoxel::VoxelGrid grid(8.0, static_cast<MyVoxel::VoxelLevel>(4));
    const MyVoxel::Shape box = MyVoxel::Modeling::makeBox(6.0, 6.0, 6.0);
    const MyVoxel::VoxelShape object = MyVoxel::Modeling::voxelize(box, grid);

    check(object.isValid() && !object.isEmpty(), "Fixture voxelization");
    check(MyVoxel::Operation::BooleanOperation::isAligned(object, object), "Self alignment");

    const MyVoxel::VoxelShape voxelUnion = MyVoxel::Operation::BooleanOperation::unite(object, object);
    check(voxelUnion.sharesDataWith(object), "Voxel self union unchanged");

    const MyVoxel::VoxelShape voxelIntersection = MyVoxel::Operation::BooleanOperation::intersect(object, object);
    check(voxelIntersection.sharesDataWith(object), "Voxel self intersection unchanged");

    const MyVoxel::VoxelShape voxelDifference = MyVoxel::Operation::BooleanOperation::subtract(object, object);
    check(voxelDifference.isEmpty(), "Voxel self difference empty");

    const MyVoxel::VoxelShape voxelExclusiveOr = MyVoxel::Operation::BooleanOperation::exclusiveOr(object, object);
    check(voxelExclusiveOr.isEmpty(), "Voxel self xor empty");

    const MyVoxel::VoxelShape shapeUnion = MyVoxel::Operation::ShapeBooleanOperation::unite(object, box);
    check(shapeUnion.sharesDataWith(object), "Shape union identical object unchanged");

    const MyVoxel::VoxelShape shapeIntersection = MyVoxel::Operation::ShapeBooleanOperation::intersect(object, box);
    check(shapeIntersection.sharesDataWith(object), "Shape intersection identical object unchanged");

    const MyVoxel::VoxelShape shapeDifference = MyVoxel::Operation::ShapeBooleanOperation::subtract(object, box);
    check(shapeDifference.isEmpty(), "Shape direct difference identical object empty");

    const MyVoxel::VoxelShape shapeExclusiveOr = MyVoxel::Operation::ShapeBooleanOperation::exclusiveOr(object, box);
    check(shapeExclusiveOr.isEmpty(), "Shape xor identical object empty");

    MyVoxel::VoxelShape inPlaceUnion = object;
    check(!MyVoxel::Operation::ShapeBooleanOperation::uniteInPlace(inPlaceUnion, box) &&
          inPlaceUnion.sharesDataWith(object), "Shape union in-place unchanged");

    MyVoxel::VoxelShape inPlaceIntersection = object;
    check(!MyVoxel::Operation::ShapeBooleanOperation::intersectInPlace(inPlaceIntersection, box) &&
          inPlaceIntersection.sharesDataWith(object), "Shape intersection in-place unchanged");

    MyVoxel::VoxelShape inPlaceDifference = object;
    check(MyVoxel::Operation::ShapeBooleanOperation::subtractInPlace(inPlaceDifference, box) &&
          inPlaceDifference.isEmpty(), "Shape difference in-place changed");

    MyVoxel::VoxelShape inPlaceExclusiveOr = object;
    check(MyVoxel::Operation::ShapeBooleanOperation::exclusiveOrInPlace(inPlaceExclusiveOr, box) &&
          inPlaceExclusiveOr.isEmpty(), "Shape xor in-place changed");

    const MyVoxel::Shape movedSphere =
        MyVoxel::Modeling::makeSphere(2.0, MyMath::Matrix4::fromTranslation(MyMath::Vector3(2.0, 0.0, 0.0)));
    const MyVoxel::VoxelShape movedToolVoxel = MyVoxel::Modeling::voxelizeAligned(movedSphere, object);
    check(MyVoxel::Operation::BooleanOperation::isAligned(object, movedToolVoxel), "Aligned Shape voxelization");

    const MyVoxel::VoxelShape referenceUnion = MyVoxel::Operation::BooleanOperation::unite(object, movedToolVoxel);
    const MyVoxel::VoxelShape shapeMovedUnion = MyVoxel::Operation::ShapeBooleanOperation::unite(object, movedSphere);
    check(referenceUnion.rootCount() == shapeMovedUnion.rootCount() &&
          referenceUnion.isEmpty() == shapeMovedUnion.isEmpty(), "Shape union follows aligned voxel path");

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Operation::ShapeBooleanStatistics statistics;
    const MyVoxel::VoxelShape statisticsResult =
        MyVoxel::Operation::ShapeBooleanOperation::apply(object, movedSphere,
            MyVoxel::Operation::VoxelBooleanType::Union, nullptr, statistics);
    check(statisticsResult.isValid() && !statistics.usedDirectShapeCut &&
          statistics.voxelizationMilliseconds >= 0.0 && statistics.booleanMilliseconds >= 0.0,
          "Shape boolean statistics voxel path");

    const MyVoxel::VoxelShape statisticsCut =
        MyVoxel::Operation::ShapeBooleanOperation::apply(object, movedSphere,
            MyVoxel::Operation::VoxelBooleanType::Difference, nullptr, statistics);
    check(statisticsCut.isValid() && statistics.usedDirectShapeCut &&
          statistics.shapeCutMilliseconds >= 0.0, "Shape boolean statistics direct cut path");
#endif

    std::cout << "Boolean Operation Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
