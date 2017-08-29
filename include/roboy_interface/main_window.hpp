#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QtGui/QMainWindow>
#include <QFileSystemModel>
#include "ui_main_window.h"
#include <roboy_communication_middleware/DarkRoomSensor.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfig.h>
#include <roboy_communication_middleware/MotorRecord.h>
#include <roboy_communication_middleware/MotorRecordConfig.h>
#include <roboy_communication_middleware/MotorTrajectoryControl.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <fstream>
#include <thread>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_interface/json.hpp>

#endif

#define RUN_IN_THREAD
#define NUMBER_OF_FPGAS 6
#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_JOINT_SENSORS 4

enum PLOTDATA{ MOTOR, JOINT};
enum MOTOR_CONTROL{POSITION,VELOCITY,DISPLACEMENT};

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

using namespace Qt;

class MainWindow : public QMainWindow, rviz_visualization {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
private:
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void MotorRecordPack(const roboy_communication_middleware::MotorRecord::ConstPtr &msg);
	void DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensor::ConstPtr &msg);
public Q_SLOTS:
    void updateSetPointsMotorControl(int percent);
    void updateSetPointsMotorControlAll(int percent);
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
Q_SIGNALS:
    void newData(int id);
	void DarkRoomSensorDataReady();
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig, motorRecordConfig, visualization_pub, motorTrajectory_pub, motorTrajectoryControl_pub;
    ros::Subscriber motorStatus, motorRecord, darkroom_sub;
    ros::ServiceClient motorConfig_srv;
    QVector<double> time;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
	bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA];
    map<int, Vector3d> sensor_position;
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
	QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
							   Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    QFileSystemModel *model;
    int numberOfRecordsToWaitFor = 0;
    map<int, vector<int32_t>[NUMBER_OF_MOTORS_PER_FPGA]> records;

    QTimer* Timer;   // A timer is needed in GUI application
};

#endif // interface_MAIN_WINDOW_H
