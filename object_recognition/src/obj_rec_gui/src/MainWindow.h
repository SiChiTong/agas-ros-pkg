#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QHash>

#include "talker.h"
//#include "listener.h"
#include "ButtonProcessAdministrator.h"

#include <vector>
#include <iostream>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "Containers/ObjectRecognitionTab/ObjectRecognitionTab.h"
#include "Containers/ObjectLearningTab/ObjectLearningTab.h"

#include <robot_platform/MoveRobot.h>
#include <robot_platform/TurnRobot.h>
#include <robot_platform/StopRobot.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ptu/SetPanTilt.h>


#include "QtRosNode.h"


namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace cv {
    class Mat;
}

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool eventFilter(QObject *object, QEvent *e);
    void setQtRosNode(QtRosNode& qtRosNode);


    ros::Publisher move_robot_pub_;
    ros::Publisher turn_robot_pub_;
    ros::Publisher stop_robot_pub_;
    ros::Publisher start_game_pub_;
    ros::Publisher fake_user_input_pub_;
    ros::Publisher speech_out_pub_;
    ros::Publisher set_pan_tilt_pub_;
    //    void updateData();
    //
    //

private slots:
    void on_butStartGame_clicked();
    void on_butFakeUserInput_clicked();

    void on_butSpeak_clicked();

    void on_pushButtonPanTiltAngle_clicked();

private:
    Ui::MainWindow *ui;
    Talker* m_Talker;
    ObjectRecognitionTab* m_ObjectRecognitionTab;
    ObjectLearningTab* m_ObjectLearningTab;

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* laser_;



};

#endif // MAINWINDOW_H
