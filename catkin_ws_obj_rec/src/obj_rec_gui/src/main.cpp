#include <QtGui/QApplication>
#include "MainWindow.h"

#include <QApplication>
#include <QObject>

#include "QtRosNode.h"

#include <sstream>

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    // Create qt gui


    QApplication a(argc, argv);
    MainWindow window;
    QtRosNode qtRosNode(argc, argv,"obj_rec_gui", &window);
    qtRosNode.start();
    window.show();

    // Create ros node in seperate thread

	window.setQtRosNode(qtRosNode);
    //window.setNodeHandle(qtRosNode.getNodeHandle());

    a.exec();
}
