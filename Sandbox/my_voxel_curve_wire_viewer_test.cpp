#include <cstdlib>
#include <iostream>

#include <QApplication>
#include <QStatusBar>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Adapter/Display_CurveAdapter.h"
#include "MyVoxel/Display/Adapter/Display_WireAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Modeling/Wire/WireModeling.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Curve / Wire Display"));
    window.resize(1400, 900);
    window.show();
    application.processEvents();

    const MyVoxel::Curve line = MyVoxel::Modeling::makeLine(
        MyMath::Vector3(-4.0, 0.0, 0.0), MyMath::Vector3(4.0, 0.0, 0.0),
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(-8.0, 5.0, 0.0)));

    const MyVoxel::Curve arc = MyVoxel::Modeling::makeArc(
        MyMath::Vector3(0.0, 0.0, 0.0), 4.0, 0.0, 3.14159265358979323846,
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, 5.0, 0.0)));

    const MyVoxel::Wire rectangle = MyVoxel::Modeling::makeRectangle(
        8.0, 5.0, MyMath::Matrix4::fromTranslation(MyMath::Vector3(-8.0, -5.0, 0.0)));

    const MyVoxel::Wire circle = MyVoxel::Modeling::makeCircle(
        4.0, MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -5.0, 0.0)));

    const MyVoxel::Display_LineObjectSnapshot lineSnapshot =
        MyVoxel::Display_CurveAdapter::buildSnapshot(101, line, MyVoxel::Display_Color::yellow(), 2.0f);
    const MyVoxel::Display_LineObjectSnapshot arcSnapshot =
        MyVoxel::Display_CurveAdapter::buildSnapshot(102, arc, MyVoxel::Display_Color::cyan(), 2.0f);
    const MyVoxel::Display_LineObjectSnapshot rectangleSnapshot =
        MyVoxel::Display_WireAdapter::buildSnapshot(103, rectangle, MyVoxel::Display_Color::magenta(), 2.0f);
    const MyVoxel::Display_LineObjectSnapshot circleSnapshot =
        MyVoxel::Display_WireAdapter::buildSnapshot(104, circle, MyVoxel::Display_Color(0.25, 0.85, 0.35, 1.0), 2.0f);

    bool success = true;
    success = window.submitDisplaySnapshot(lineSnapshot) && success;
    success = window.submitDisplaySnapshot(arcSnapshot) && success;
    success = window.submitDisplaySnapshot(rectangleSnapshot) && success;
    success = window.submitDisplaySnapshot(circleSnapshot) && success;

    if (!success)
    {
        std::cerr << "Curve/Wire viewer snapshot submission failed." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Line segments: " << window.viewer()->displayObjectSegmentCount(101) << std::endl;
    std::cout << "Arc segments: " << window.viewer()->displayObjectSegmentCount(102) << std::endl;
    std::cout << "Rectangle parts/segments: " << window.viewer()->displayObjectPartCount(103)
              << "/" << window.viewer()->displayObjectSegmentCount(103) << std::endl;
    std::cout << "Circle parts/segments: " << window.viewer()->displayObjectPartCount(104)
              << "/" << window.viewer()->displayObjectSegmentCount(104) << std::endl;

    window.statusBar()->showMessage(QStringLiteral(
        "Top-left: Line  Top-right: Arc  Bottom-left: Rectangle Wire  Bottom-right: Circle Wire"));
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}
