#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QComboBox>
#include <QtCore>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <ros/package.h>
#include "speech_rec_messages/SpeechRecCommandM.h"
//#include "door_detection_messages/DetectDoorControl.h"
//#include "door_detection_messages/DetectHandleControl.h"



#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/yaml_config_reader.h"
#include "rviz/yaml_config_writer.h"

using namespace std;


bool MainWindow::eventFilter(QObject *object, QEvent *event)
 {
    if (object == this && event->type() == QEvent::KeyPress) {
        robot_platform::MoveRobot msgMove;
        robot_platform::TurnRobot msgTurn;
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        switch (keyEvent->key()) {
        case Qt::Key_W:
            msgMove.m_Distance = 0.1;
            msgMove.m_Speed = 0.05;
            msgMove.m_Permanent = false;
            move_robot_pub_.publish(msgMove);
            ROS_INFO_STREAM("MOVE FORWARD");
            //cout<<"Move Forward"<<endl;
            break;
        case Qt::Key_S:
            msgMove.m_Distance = -0.1;
            msgMove.m_Speed = 0.05;
            msgMove.m_Permanent = false;
            move_robot_pub_.publish(msgMove);
            ROS_INFO("Move Back");
            break;
        case Qt::Key_A:
            msgTurn.m_Theta = 15;
            msgTurn.m_Speed = 0.1;
            msgTurn.m_Permanent = false;
            turn_robot_pub_.publish(msgTurn);
            ROS_INFO("Turn Left");
            break;

        case Qt::Key_D:
            msgTurn.m_Theta = -15;
            msgTurn.m_Speed = 0.1;
            msgTurn.m_Permanent = false;
            turn_robot_pub_.publish(msgTurn);
            ROS_INFO("Turn Right");
            break;
        default:
            break;
        }
//        if (keyEvent->            cout<<"Turn Left"<<endl;
//key() == Qt::Key_Tab) {
//            // Special tab handling
//            std::cout<<"Hello Keys"<<std::endl;
//            return true;
//        } else
//            return false;
    }
    return false;
 }


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_Talker(0)
{
    ui->setupUi(this);

    qApp->installEventFilter(this);



//	qtRosNode.getNodeHandle()



}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::setQtRosNode(QtRosNode& qtRosNode)
{
    m_Talker = new Talker(*qtRosNode.getNodeHandle(),"chatter");

    m_ObjectRecognitionTab = new ObjectRecognitionTab ( qtRosNode.getNodeHandle(), ui->centralWidget);
    ui->mainTabs->addTab ( m_ObjectRecognitionTab, tr ( "&Object Recognition" ) );

    m_ObjectLearningTab = new ObjectLearningTab ( qtRosNode.getNodeHandle(), ui->centralWidget);
    ui->mainTabs->addTab ( m_ObjectLearningTab, tr ( "&Object Learning" ) );


    qRegisterMetaType<std::vector<map_messages::PointOfInterest> >("std::vector<map_messages::PointOfInterest>");
    qRegisterMetaType<std::vector<geometry_msgs::PoseStamped> >("std::vector<geometry_msgs::PoseStamped>");
    qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");

    connect(&qtRosNode, SIGNAL(learningStatus(std::vector<std::string>,std::string)), m_ObjectLearningTab, SLOT(processLearningStatus(std::vector<std::string>,std::string)));
    connect(&qtRosNode, SIGNAL(objectNames(std::vector<std::string>,std::vector<std::string>)), m_ObjectRecognitionTab, SLOT(processObjectNames(std::vector<std::string>,std::vector<std::string>)));



    connect(&qtRosNode, SIGNAL(rosShutdown()), this, SLOT(close()));


    move_robot_pub_ = qtRosNode.getNodeHandle()->advertise<robot_platform::MoveRobot>("/robot_platform/MoveRobot", 1);
    turn_robot_pub_ = qtRosNode.getNodeHandle()->advertise<robot_platform::TurnRobot>("/robot_platform/TurnRobot", 1);
    stop_robot_pub_ = qtRosNode.getNodeHandle()->advertise<robot_platform::StopRobot>("/robot_platform/StopRobot", 1);    stop_robot_pub_ = qtRosNode.getNodeHandle()->advertise<robot_platform::StopRobot>("/robot_platform/StopRobot", 1);


    start_game_pub_ = qtRosNode.getNodeHandle()->advertise<std_msgs::Empty>("/start_game", 1);
    speech_out_pub_ = qtRosNode.getNodeHandle()->advertise<std_msgs::String>("/robot_face/text_out", 1);
    fake_user_input_pub_ = qtRosNode.getNodeHandle()->advertise<std_msgs::String>("/robot_face/user_input", 1);

    set_pan_tilt_pub_ = qtRosNode.getNodeHandle()->advertise<ptu::SetPanTilt>("/ptu/set_pan_tilt", 1);

    //m_HandleDetectionPub = m_NodeHandle->advertise<door_detection_messages::DetectHandleControl>("/handle_detection/detect_handle_control", 10);

    //m_DoorDetectionPub = m_NodeHandle->advertise<door_detection_messages::DetectDoorControl>("/door_detection/detect_door_control", 10);




    render_panel_ = new rviz::RenderPanel();

    ui->frame->layout()->addWidget(render_panel_);

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc. It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    //manager_->setFixedFrame("/map");

    rviz::YamlConfigReader reader;
    rviz::Config config;
    std::string filename = ros::package::getPath("homer_bringup")+"/homer_display_or_final.rviz" ;
    reader.readFile( config, QString::fromStdString( filename ));
    if( !reader.error() )
    {
        manager_->load( config.mapGetChild( "Visualization Manager" ));
    }


    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create a Grid display.
    //grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    //laser_ = manager_->createDisplay( "rviz/Map", "sick laser data", true );
    //ROS_ASSERT( grid_ != NULL );
   // ROS_ASSERT( laser_ != NULL );

    // Configure the GridDisplay the way we like it.
    //grid_->subProp( "Line Style" )->setValue( "Billboards" );
    //grid_->subProp( "Color" )->setValue( Qt::yellow );


    //laser_->subProp( "Topic" )->setValue( "/map");
}

void MainWindow::on_butStartGame_clicked()
{
    std_msgs::Empty msg;
    start_game_pub_.publish(msg);
}

void MainWindow::on_butFakeUserInput_clicked()
{

}

void MainWindow::on_butSpeak_clicked()
{

}


void MainWindow::on_pushButtonPanTiltAngle_clicked()
{

}
