#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <QtGui/QMainWindow>
#include <QFileSystemModel>
#include "ui_main_window.h"
#include "roboy_managing_node/myoMaster.hpp"
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/JointCommand.h>
#include <roboy_communication_middleware/JointAngle.h>
#include <roboy_communication_middleware/ArucoPose.h>
#include <roboy_communication_middleware/DanceCommand.h>
#include <roboy_communication_middleware/DanceTrajectory.h>
#include <roboy_communication_middleware/DarkRoomSensor.h>
#include <roboy_communication_middleware/InverseKinematics.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <fstream>
#include <thread>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <common_utilities/rviz_visualization.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <roboy_interface/json.hpp>

#endif

#define RUN_IN_THREAD
#define NUMBER_OF_FPGAS 5
#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_JOINT_SENSORS 4

enum PLOTDATA{ MOTOR, JOINT};

using namespace std;
using namespace Eigen;
using namespace cv;
using json = nlohmann::json;

namespace interface {

class MainWindow : public QMainWindow, rviz_visualization {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    MyoMaster *myoMaster;
private:
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    void MotorRecordPack(const roboy_communication_middleware::MotorRecord::ConstPtr &msg);
    void receiveImage(const sensor_msgs::ImageConstPtr &msg);
    void JointCommand(const roboy_communication_middleware::JointCommand::ConstPtr& msg);
    void ArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr& msg);
    float calculateAngleBetween(int aruco0, int aruco1, int aruco2, int aruco3);
	double calculateAngleBetween(Vector3d &sensor0, Vector3d &sensor1, Vector3d &axis);
	void DanceCommand(const roboy_communication_middleware::DanceCommand::ConstPtr &msg);
	void DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensor::ConstPtr &msg);
	void InteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg);
	std::pair<Vector3d, Vector3d> best_plane_from_points(map<int, Vector3d> & c, vector<int> &ids);
    bool danceTrajectory(roboy_communication_middleware::DanceTrajectory::Request &req,
                           roboy_communication_middleware::DanceTrajectory::Response &res);
	double phi = 0;// angle left lower leg to world
public Q_SLOTS:
	void VisualServoing();
	void on_actionAbout_triggered();
    void updateSetPointsMotorControl(int percent);
    void updateSetPointsMotorControlAll(int percent);
    void updateSetPointsJointControl(int percent);
    void updateSetPointsJointControlAll(int percent);
	void updateControllerParams();
    void movementPathChanged();
    void recordMovement();
    void plotData(int id);
    bool playMovement();
    void stopMovement();
    void rewindMovement();
    void pauseMovement();
    void loopMovement();
    void stopButtonClicked();
    void danceToggle();
    void displayImage();
	void resetPose();
	void danceController();
Q_SIGNALS:
    void newData(int id);
    void drawImage();
	void DarkRoomSensorDataReady();
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig, motorRecordConfig, motorTrajectory, motorTrajectoryControl, hipCenter_pub,
            jointCommand_pub, jointAnglesOffset_pub, visualization_pub, danceCommand_pub;
    ros::Subscriber motorStatus, motorRecord, jointStatus, jointCommand, realsense, arucoPose, danceCommand, darkroom_sub,
            interactive_marker_sub;
    ros::ServiceServer danceService;
    QVector<double> time;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
	QVector<double> jointData[NUMBER_OF_FPGAS][NUMBER_OF_JOINT_SENSORS][4];
	QVector<double> jointSetpointData[NUMBER_OF_FPGAS][NUMBER_OF_JOINT_SENSORS];
	float sign[NUMBER_OF_JOINT_SENSORS] = {1.0f,1.0f,1.0f,1.0f};
	bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA], jointControl = false, motorControl = false,
            dance = false, initializeJointAngles = true, visualServoing = false;
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
	QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
							   Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    QFileSystemModel *model;
    int numberOfRecordsToWaitFor = 0;
    map<int, vector<int32_t>[NUMBER_OF_MOTORS_PER_FPGA]> records;

    Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    QImage imdisplay;
	Vector3d targetPosition;
    vector<float> angle, jointAngleOffset, setPointAngle;
    map<int, Vector3f> arucoMarkerPosition;
    map<int, Vector2f> arucoMarkerCenter;
	map<int, Vector3d> sensor_position;
    QTimer* Timer;   // A timer is needed in GUI application
    QPixmap pixmap;
    const float image_scale = 0.5f;
    roboy_communication_middleware::JointCommand joint_command_msg;
    ros::ServiceClient ik_srv;
	Vector2d errorVisualServoing, errorVisualServoing_prev, resultVisualServoing, integralVisualServoing,
			integralVisualServoingMax, integralVisualServoingMin;
	tf::Transform relativeFrame;
	tf::TransformBroadcaster tf_broadcaster;
	map<int,int> sensor_map{
			{5,0},
			{1,1},
			{2,2},
			{3,3},
			{0,4},
			{7,5},
			{8,6},
			{4,7},
			{6,8},
	};
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H
