#include <roboy_interface/main_window.hpp>

namespace interface {

    using namespace Qt;

    MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
            : QMainWindow(parent) {
        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                         SLOT(aboutQt())); // qApp is a global variable for the application

        ReadSettings();
        setWindowIcon(QIcon(":/images/icon.png"));
        ui.tab_manager->setCurrentIndex(
                0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

        QObject::connect(this, SIGNAL(newData(int)), this, SLOT(plotData(int)));
        QObject::connect(this, SIGNAL(drawImage()), this, SLOT(displayImage()));
        QObject::connect(this, SIGNAL(DarkRoomSensorDataReady()), this, SLOT(VisualServoing()));

        QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor6, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor7, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor8, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor9, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor10, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor11, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor12, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.motor13, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
        QObject::connect(ui.allMotors, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControlAll(int)));
        QObject::connect(ui.joint0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
        QObject::connect(ui.joint1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
        QObject::connect(ui.joint2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
        QObject::connect(ui.joint3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
        QObject::connect(ui.allJoints, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControlAll(int)));
        QObject::connect(ui.updateController, SIGNAL(clicked()), this, SLOT(updateControllerParams()));
        QObject::connect(ui.record, SIGNAL(clicked()), this, SLOT(recordMovement()));
        QObject::connect(ui.play, SIGNAL(clicked()), this, SLOT(playMovement()));
        QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stopMovement()));
        QObject::connect(ui.rewind, SIGNAL(clicked()), this, SLOT(rewindMovement()));
        QObject::connect(ui.pause, SIGNAL(clicked()), this, SLOT(pauseMovement()));
        QObject::connect(ui.loop, SIGNAL(clicked()), this, SLOT(loopMovement()));
        QObject::connect(ui.stop_button_motorControl, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
        QObject::connect(ui.stop_button_jointControl, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
        QObject::connect(ui.dance, SIGNAL(clicked()), this, SLOT(danceToggle()));
        QObject::connect(ui.reset_pose, SIGNAL(clicked()), this, SLOT(resetPose()));
        QObject::connect(ui.visual_servoing, SIGNAL(clicked()), this, SLOT(danceController()));
        ui.stop_button_motorControl->setStyleSheet("background-color: red");
        ui.stop_button_jointControl->setStyleSheet("background-color: red");
        ui.dance->setStyleSheet("background-color: green");

        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "interface",
                      ros::init_options::NoSigintHandler |
                      ros::init_options::AnonymousName |
                      ros::init_options::NoRosout);
        }
        motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &MainWindow::MotorStatus, this);
        jointStatus = nh->subscribe("/roboy/middleware/JointStatus", 1, &MainWindow::JointStatus, this);
//    jointCommand = nh->subscribe("/roboy/middleware/JointCommand", 1, &MainWindow::JointCommand, this);
        danceCommand = nh->subscribe("/roboy/middleware/DanceCommand", 1, &MainWindow::DanceCommand, this);
        danceCommand_pub = nh->advertise<roboy_communication_middleware::DanceCommand>("/roboy/middleware/DanceCommand",
                                                                                       1);
        realsense = nh->subscribe("/roboy/middleware/realsense", 1, &MainWindow::receiveImage, this);
        arucoPose = nh->subscribe("/roboy/middleware/ArucoPose", 1, &MainWindow::ArucoPose, this);
        motorConfig = nh->advertise<roboy_communication_middleware::MotorConfig>("/roboy/middleware/MotorConfig", 1);
        motorRecordConfig = nh->advertise<roboy_communication_middleware::MotorRecordConfig>(
                "/roboy/middleware/MotorRecordConfig", 1);
        motorRecord = nh->subscribe("/roboy/middleware/MotorRecord", 100, &MainWindow::MotorRecordPack, this);
        motorTrajectory = nh->advertise<roboy_communication_middleware::MotorRecord>(
                "/roboy/middleware/MotorTrajectory", 1);
        motorTrajectoryControl = nh->advertise<roboy_communication_middleware::MotorTrajectoryControl>(
                "/roboy/middleware/MotorTrajectoryControl", 1);
        hipCenter_pub = nh->advertise<geometry_msgs::Pose>("/roboy/middleware/HipCenter", 1);
        jointCommand_pub = nh->advertise<roboy_communication_middleware::JointCommand>("/roboy/middleware/JointCommand",
                                                                                       1);
        jointAnglesOffset_pub = nh->advertise<roboy_communication_middleware::JointAngle>(
                "/roboy/middleware/JointAngleOffset", 1);
        ik_srv = nh->serviceClient<roboy_communication_middleware::InverseKinematics>(
                "/roboy/middleware/PaBiRoboy/inverseKinematics");
        darkroom_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensor_location", 1, &MainWindow::DarkRoomSensor,
                                     this);
        visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

        interactive_marker_sub = nh->subscribe("/hip_position/feedback", 1, &MainWindow::InteractiveMarkerFeedback,
                                               this);

        danceService = nh->advertiseService("/roboy/middleware/PaBiRoboy/danceTrajectory",
                                            &MainWindow::danceTrajectory, this);

        spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(5));
        spinner->start();

        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            ui.position_plot->addGraph();
            ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
            ui.velocity_plot->addGraph();
            ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
            ui.displacement_plot->addGraph();
            ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
            ui.current_plot->addGraph();
            ui.current_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        }
        ui.position_plot->xAxis->setLabel("x");
        ui.position_plot->yAxis->setLabel("ticks");
        ui.position_plot->replot();

        ui.velocity_plot->xAxis->setLabel("x");
        ui.velocity_plot->yAxis->setLabel("ticks/s");
        ui.velocity_plot->replot();

        ui.displacement_plot->xAxis->setLabel("x");
        ui.displacement_plot->yAxis->setLabel("ticks");
        ui.displacement_plot->replot();

        ui.current_plot->xAxis->setLabel("x");
        ui.current_plot->yAxis->setLabel("mA");
        ui.current_plot->replot();

        ui.relAngle0_plot->addGraph();
        ui.relAngle0_plot->graph(0)->setPen(QPen(color_pallette[0]));
        ui.relAngle0_plot->addGraph();
        ui.relAngle0_plot->graph(1)->setPen(QPen(Qt::gray));
        ui.relAngle0_plot->xAxis->setLabel("x");
        ui.relAngle0_plot->yAxis->setLabel("degrees");
        ui.relAngle0_plot->yAxis->setRange(-100, 100);

        ui.relAngle1_plot->addGraph();
        ui.relAngle1_plot->graph(0)->setPen(QPen(color_pallette[1]));
        ui.relAngle1_plot->addGraph();
        ui.relAngle1_plot->graph(1)->setPen(QPen(Qt::gray));
        ui.relAngle1_plot->xAxis->setLabel("x");
        ui.relAngle1_plot->yAxis->setLabel("degrees");
        ui.relAngle1_plot->yAxis->setRange(-100, 100);

        ui.relAngle2_plot->addGraph();
        ui.relAngle2_plot->graph(0)->setPen(QPen(color_pallette[2]));
        ui.relAngle2_plot->addGraph();
        ui.relAngle2_plot->graph(1)->setPen(QPen(Qt::gray));
        ui.relAngle2_plot->xAxis->setLabel("x");
        ui.relAngle2_plot->yAxis->setLabel("degrees");
        ui.relAngle2_plot->yAxis->setRange(-100, 100);

        ui.relAngle3_plot->addGraph();
        ui.relAngle3_plot->graph(0)->setPen(QPen(color_pallette[3]));
        ui.relAngle3_plot->addGraph();
        ui.relAngle3_plot->graph(1)->setPen(QPen(Qt::gray));
        ui.relAngle3_plot->xAxis->setLabel("x");
        ui.relAngle3_plot->yAxis->setLabel("degrees");
        ui.relAngle3_plot->yAxis->setRange(-100, 100);

        ui.relAngle0_plot->replot();
        ui.relAngle1_plot->replot();
        ui.relAngle2_plot->replot();
        ui.relAngle3_plot->replot();

        updateControllerParams();

        model = new QFileSystemModel;
        movementPathChanged();

        angle.resize(4);
        jointAngleOffset.resize(4, -180);

        setPointAngle.resize(4);

        joint_command_msg.link_name.push_back("Groin_right");
        joint_command_msg.link_name.push_back("Knee_right");
        joint_command_msg.link_name.push_back("Groin_left");
        joint_command_msg.link_name.push_back("Knee_left");

        joint_command_msg.angle.push_back(0);
        joint_command_msg.angle.push_back(0);
        joint_command_msg.angle.push_back(0);
        joint_command_msg.angle.push_back(0);

        targetPosition = Vector3d(0.5, 0.5, 0);

        errorVisualServoing = Vector2d(0, 0);
        errorVisualServoing_prev = Vector2d(0, 0);
        resultVisualServoing = Vector2d(0, 0);
        integralVisualServoing = Vector2d(0, 0);
        integralVisualServoingMax = Vector2d(0, 0);
        integralVisualServoingMin = Vector2d(0, 0);
    }

    MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

    void MainWindow::showNoMasterMessage() {
        QMessageBox msgBox;
        msgBox.setText("Couldn't find the ros master.");
        msgBox.exec();
        close();
    }

    void MainWindow::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
        ROS_INFO_THROTTLE(5, "receiving motor status");
        time.push_back(counter++);
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            motorData[msg->id][motor][0].push_back(msg->position[motor]);
            motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
            motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
            motorData[msg->id][motor][3].push_back(msg->pwmRef[motor]);
            if (motorData[msg->id][motor][0].size() > samples_per_plot) {
                motorData[msg->id][motor][0].pop_front();
                motorData[msg->id][motor][1].pop_front();
                motorData[msg->id][motor][2].pop_front();
                motorData[msg->id][motor][3].pop_front();
            }
            if (msg->current[motor] != 0) {
                motorConnected[msg->id][motor] = true;
            } else {
                motorConnected[msg->id][motor] = false;
                switch (motor) {
                    case 0:
                        ui.setPoint_motor0->setText("dead");
                        break;
                    case 1:
                        ui.setPoint_motor1->setText("dead");
                        break;
                    case 2:
                        ui.setPoint_motor2->setText("dead");
                        break;
                    case 3:
                        ui.setPoint_motor3->setText("dead");
                        break;
                    case 4:
                        ui.setPoint_motor4->setText("dead");
                        break;
                    case 5:
                        ui.setPoint_motor5->setText("dead");
                        break;
                    case 6:
                        ui.setPoint_motor6->setText("dead");
                        break;
                    case 7:
                        ui.setPoint_motor7->setText("dead");
                        break;
                    case 8:
                        ui.setPoint_motor8->setText("dead");
                        break;
                    case 9:
                        ui.setPoint_motor9->setText("dead");
                        break;
                    case 10:
                        ui.setPoint_motor10->setText("dead");
                        break;
                    case 11:
                        ui.setPoint_motor11->setText("dead");
                        break;
                    case 12:
                        ui.setPoint_motor12->setText("dead");
                        break;
                    case 13:
                        ui.setPoint_motor13->setText("dead");
                        break;
                }
            }
        }
        if (time.size() > samples_per_plot)
            time.pop_front();
        static int counter = 0;
        if ((counter++) % 3 == 0)
                Q_EMIT newData(MOTOR);
    }

    void MainWindow::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
        ROS_INFO_THROTTLE(5, "receiving joint status");
        float jointAngles[NUMBER_OF_JOINT_SENSORS];
        for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {
            jointAngles[joint] = msg->relAngles[joint] / 4096.0 * 360.0;
            jointData[msg->id][joint][1].push_back(sign[joint] * (jointAngles[joint] + jointAngleOffset[joint]));
            if (jointData[msg->id][joint][1].size() > samples_per_plot) {
                jointData[msg->id][joint][1].pop_front();
            }
        }

        if (jointControl) {
            ROS_INFO_THROTTLE(5, "joint control active");
            std::lock_guard<std::mutex> lock(myoMaster->mux);
            float error[NUMBER_OF_JOINT_SENSORS];
            float integral[NUMBER_OF_JOINT_SENSORS];
            float integral_max = 360;
            const float smooth_distance = 50;
            const float offset = 20;
            static float error_previous[NUMBER_OF_JOINT_SENSORS] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {
                switch (joint) {
                    case 0:
                        setPointAngle[joint] = ui.joint0->value();
                        break;
                    case 1:
                        setPointAngle[joint] = ui.joint1->value();
                        break;
                    case 2:
                        setPointAngle[joint] = ui.joint2->value();
                        break;
                    case 3:
                        setPointAngle[joint] = ui.joint3->value();
                        break;
                }
                error[joint] = setPointAngle[joint] - jointData[msg->id][joint][1].back();
                switch (joint) {
                    case 0: {
                        float pterm = atoi(ui.Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(6, offset - result);
                            myoMaster->changeSetPoint(4, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(6, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(4, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(6, offset);
                            myoMaster->changeSetPoint(4, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 1: {
                        float pterm = atoi(ui.Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(2, offset - result);
                            myoMaster->changeSetPoint(0, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(2, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(0, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(2, offset);
                            myoMaster->changeSetPoint(0, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 2: {
                        float pterm = atoi(ui.Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(11, offset - result);
                            myoMaster->changeSetPoint(13, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(11, offset + powf(result - smooth_distance, 2.0f) /
                                                                   (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(13, offset + powf(result + smooth_distance, 2.0f) /
                                                                   (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(11, offset);
                            myoMaster->changeSetPoint(13, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 3: {
                        float pterm = atoi(ui.Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] +=
                                (float) atoi(ui.Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
//                    ROS_INFO_THROTTLE(1,"\npterm %f\ndterm %f\niterm %f\nresult %f", pterm, dterm, integral[joint], result);
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(9, offset - result);
                            myoMaster->changeSetPoint(7, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(9, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(7, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(9, offset);
                            myoMaster->changeSetPoint(7, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                }
                jointSetpointData[msg->id][joint].push_back(setPointAngle[joint]);
                if (jointSetpointData[msg->id][joint].size() > samples_per_plot) {
                    jointSetpointData[msg->id][joint].pop_front();
                }
            }
            QString str;
            str.sprintf("\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f",
                        setPointAngle[0], jointData[msg->id][0][1].back(), error[0],
                        setPointAngle[1], jointData[msg->id][1][1].back(), error[1],
                        setPointAngle[2], jointData[msg->id][2][1].back(), error[2],
                        setPointAngle[3], jointData[msg->id][3][1].back(), error[3]);
            ui.joint_console->append(str);
            ui.joint_console->verticalScrollBar()->setValue(ui.joint_console->verticalScrollBar()->maximum());
        }
        if (dance) {
            ROS_INFO_THROTTLE(5, "dance control active");
            std::lock_guard<std::mutex> lock(myoMaster->mux);
            float error[NUMBER_OF_JOINT_SENSORS];
            float integral[NUMBER_OF_JOINT_SENSORS];
            float integral_max = 360;
            const float smooth_distance = 50;
            const float offset = 200;
            static float error_previous[NUMBER_OF_JOINT_SENSORS] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {
                error[joint] = setPointAngle[joint] - jointData[msg->id][joint][1].back();
                switch (joint) {
                    case 0: {
                        float pterm = atoi(ui.Kp_danceControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_danceControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_danceControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(6, offset - result);
                            myoMaster->changeSetPoint(4, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(6, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(4, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(6, offset);
                            myoMaster->changeSetPoint(4, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 1: {
                        float pterm = atoi(ui.Kp_danceControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_danceControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_danceControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(2, offset - result);
                            myoMaster->changeSetPoint(0, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(2, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(0, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(2, offset);
                            myoMaster->changeSetPoint(0, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 2: {
                        float pterm = atoi(ui.Kp_danceControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_danceControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(ui.Ki_danceControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(11, offset - result);
                            myoMaster->changeSetPoint(13, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(11, offset + powf(result - smooth_distance, 2.0f) /
                                                                   (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(13, offset + powf(result + smooth_distance, 2.0f) /
                                                                   (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(11, offset);
                            myoMaster->changeSetPoint(13, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 3: {
                        float pterm = atoi(ui.Kp_danceControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(ui.Kd_danceControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] +=
                                (float) atoi(ui.Ki_danceControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
//                    ROS_INFO_THROTTLE(1,"\npterm %f\ndterm %f\niterm %f\nresult %f", pterm, dterm, integral[joint], result);
                        if (result <= -smooth_distance) {
                            myoMaster->changeSetPoint(9, offset - result);
                            myoMaster->changeSetPoint(7, offset);
                        } else if (result < smooth_distance) {
                            myoMaster->changeSetPoint(9, offset + powf(result - smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                            myoMaster->changeSetPoint(7, offset + powf(result + smooth_distance, 2.0f) /
                                                                  (4.0f * smooth_distance));
                        } else {
                            myoMaster->changeSetPoint(9, offset);
                            myoMaster->changeSetPoint(7, offset + result);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                }
                jointSetpointData[msg->id][joint].push_back(setPointAngle[joint]);
                if (jointSetpointData[msg->id][joint].size() > samples_per_plot) {
                    jointSetpointData[msg->id][joint].pop_front();
                }
            }
            QString str;
            str.sprintf("\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f"
                                "\nsetpoint %f\tjointAngle %f\terror %f",
                        setPointAngle[0], jointData[msg->id][0][1].back(), error[0],
                        setPointAngle[1], jointData[msg->id][1][1].back(), error[1],
                        setPointAngle[2], jointData[msg->id][2][1].back(), error[2],
                        setPointAngle[3], jointData[msg->id][3][1].back(), error[3]);
            ui.dance_console->append(str);
            ui.dance_console->verticalScrollBar()->setValue(ui.dance_console->verticalScrollBar()->maximum());
        }
        static int counter = 0;
        if ((counter++) % 3 == 0)
                Q_EMIT newData(JOINT);

        if (!initializeJointAngles) { // if we are initialized publish the corrected joint angles
            ROS_INFO_THROTTLE(10, "joint angles corrected: \n%f\t%f\t%f\t%f", jointAngles[0] + jointAngleOffset[0],
                              jointAngles[1] + jointAngleOffset[1], jointAngles[2] + jointAngleOffset[2],
                              jointAngles[3] + jointAngleOffset[3]);
            joint_command_msg.angle[0] = degreesToRadians(jointData[0][0][1].back());
            joint_command_msg.angle[1] = degreesToRadians(jointData[0][1][1].back());
            joint_command_msg.angle[2] = degreesToRadians(jointData[0][2][1].back());
            joint_command_msg.angle[3] = degreesToRadians(jointData[0][3][1].back());
            jointCommand_pub.publish(joint_command_msg);
        } else {
            float anglebetween[4];
//        anglebetween[0] = 90.0f -calculateAngleBetween(868, 576, 282, 754);
//        anglebetween[1] = -calculateAngleBetween(429, 260, 868, 576);
//        anglebetween[2] = -90.0f + calculateAngleBetween(282, 754, 1007, 422);
//        anglebetween[3] = calculateAngleBetween(1007, 422, 120, 100);

//        anglebetween[0] = -80.0f;
//        anglebetween[1] = 80.0f;
//        anglebetween[2] = -80.0f;
//        anglebetween[3] = 80.0f;

//        anglebetween[0] = 0;
//        anglebetween[1] = 0;
//        anglebetween[2] = 0;
//        anglebetween[3] = 0;

            sign[0] = 1.0f;
            sign[1] = 1.0f;
            sign[2] = 1.0f;
            sign[3] = 1.0f;

            joint_command_msg.angle[0] = degreesToRadians(anglebetween[0]);
            joint_command_msg.angle[1] = degreesToRadians(anglebetween[1]);
            joint_command_msg.angle[2] = degreesToRadians(anglebetween[2]);
            joint_command_msg.angle[3] = degreesToRadians(anglebetween[3]);
            jointCommand_pub.publish(joint_command_msg);

//        jointAngleOffset[0] = anglebetween[0] - jointAngles[0];
//        jointAngleOffset[1] = anglebetween[1] - jointAngles[1];
//        jointAngleOffset[2] = anglebetween[2] - jointAngles[2];
//        jointAngleOffset[3] = anglebetween[3] - jointAngles[3];
            ROS_WARN_THROTTLE(1, "angle between 868, 576, 282, 754:    %f \t\tsensor measures %f", anglebetween[0],
                              jointData[0][0][1].back());
            ROS_WARN_THROTTLE(1, "angle between 429, 260, 868, 576:    %f \t\tsensor measures %f", anglebetween[1],
                              jointData[0][1][1].back());
            ROS_WARN_THROTTLE(1, "angle between 282, 754, 1007, 422:   %f \t\tsensor measures %f", anglebetween[2],
                              jointData[0][2][1].back());
            ROS_WARN_THROTTLE(1, "angle between 1007, 422, 120, 100:   %f \t\tsensor measures %f", anglebetween[3],
                              jointData[0][3][1].back());

            roboy_communication_middleware::JointAngle angleOffset_msg;
            for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {
                angleOffset_msg.angle.push_back(jointAngleOffset[joint]);
            }
            jointAnglesOffset_pub.publish(angleOffset_msg);
            initializeJointAngles = false;
        }
    }

    void MainWindow::MotorRecordPack(const roboy_communication_middleware::MotorRecord::ConstPtr &msg) {
        numberOfRecordsToWaitFor--;
        records[msg->id][0] = msg->motor0;
        records[msg->id][1] = msg->motor1;
        records[msg->id][2] = msg->motor2;
        records[msg->id][3] = msg->motor3;
        records[msg->id][4] = msg->motor4;
        records[msg->id][5] = msg->motor5;
        records[msg->id][6] = msg->motor6;
        records[msg->id][7] = msg->motor7;
        records[msg->id][8] = msg->motor8;
        records[msg->id][9] = msg->motor9;
        records[msg->id][10] = msg->motor10;
        records[msg->id][11] = msg->motor11;
        records[msg->id][12] = msg->motor12;
        records[msg->id][13] = msg->motor13;
        ROS_INFO("received record from %d of length %ld with average sampling time %f ms",
                 msg->id, msg->motor0.size(), msg->recordTime / msg->motor0.size() * 1000.0f);
        if (numberOfRecordsToWaitFor == 0) {
            ROS_INFO("all records received");
            for (auto const &rec:records) {
                std::ofstream outfile;
                outfile.open(ui.movementName->text().toStdString().c_str());
                if (outfile.is_open()) {
                    outfile << "<?xml version=\"1.0\" ?>"
                            << std::endl;
                    outfile << "<roboy_trajectory control_mode=\""
                            << ui.control_mode->text().toStdString() << "\" samplingTime=\""
                            << atoi(ui.samplingTime->text().toStdString().c_str()) << "\">" << endl;
                    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                        outfile << "<trajectory motorid=\"" << motor << "\">"
                                << std::endl;
                        for (uint i = 0; i < rec.second[motor].size(); i++)
                            outfile << rec.second[motor][i] << " ";
                        outfile << "</trajectory>" << std::endl;
                    }
                    outfile << "</roboy_trajectory>" << endl;
                    outfile.close();
                }
            }
        }
    }

    void MainWindow::plotData(int type) {
        switch (type) {
            case MOTOR:
                for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                    ui.position_plot->graph(motor)->setData(time, motorData[0][motor][0]);
                    ui.velocity_plot->graph(motor)->setData(time, motorData[0][motor][1]);
                    ui.displacement_plot->graph(motor)->setData(time, motorData[0][motor][2]);
                    ui.current_plot->graph(motor)->setData(time, motorData[0][motor][3]);

                    if (motor == 0) {
                        ui.position_plot->graph(motor)->rescaleAxes();
                        ui.velocity_plot->graph(motor)->rescaleAxes();
                        ui.displacement_plot->graph(motor)->rescaleAxes();
                        ui.current_plot->graph(motor)->rescaleAxes();
                    } else {
                        ui.position_plot->graph(motor)->rescaleAxes(true);
                        ui.velocity_plot->graph(motor)->rescaleAxes(true);
                        ui.displacement_plot->graph(motor)->rescaleAxes(true);
                        ui.current_plot->graph(motor)->rescaleAxes(true);
                    }
                }
                ui.position_plot->replot();
                ui.velocity_plot->replot();
                ui.displacement_plot->replot();
                ui.current_plot->replot();
                break;
            case JOINT:
                ui.relAngle0_plot->graph(0)->setData(time, jointData[0][0][1]);
                ui.relAngle0_plot->graph(1)->setData(time, jointSetpointData[0][0]);
                ui.relAngle0_plot->xAxis->rescale();
                ui.relAngle0_plot->replot();

                ui.relAngle1_plot->graph(0)->setData(time, jointData[0][1][1]);
                ui.relAngle1_plot->graph(1)->setData(time, jointSetpointData[0][1]);
                ui.relAngle1_plot->xAxis->rescale();
                ui.relAngle1_plot->replot();

                ui.relAngle2_plot->graph(0)->setData(time, jointData[0][2][1]);
                ui.relAngle2_plot->graph(1)->setData(time, jointSetpointData[0][2]);
                ui.relAngle2_plot->xAxis->rescale();
                ui.relAngle2_plot->replot();

                ui.relAngle3_plot->graph(0)->setData(time, jointData[0][3][1]);
                ui.relAngle3_plot->graph(1)->setData(time, jointSetpointData[0][3]);
                ui.relAngle3_plot->xAxis->rescale();
                ui.relAngle3_plot->replot();
                break;
        }

    }

    bool MainWindow::playMovement() {
        // initialize TiXmlDocument doc with a string
        QModelIndexList indexList = ui.movementFolder->selectionModel()->selectedIndexes();
        ROS_INFO_STREAM("loading trajectory " << model->filePath(indexList[0]).toStdString().c_str());
        TiXmlDocument doc(model->filePath(indexList[0]).toStdString().c_str());
        if (!doc.LoadFile()) {
            return false;
        }

        TiXmlElement *root = doc.RootElement();

        int numberOfSamples, samplingTime, control_mode;

        root->QueryIntAttribute("control_mode", &control_mode);
        root->QueryIntAttribute("samplingTime", &samplingTime);
        ROS_INFO("recognized roboy_trajectory in control_mode %d with samplingTime %d", control_mode, samplingTime);

        roboy_communication_middleware::MotorRecord msg;

        // Constructs the myoMuscles by parsing custom xml.
        TiXmlElement *trajectory_it = NULL;
        for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;
             trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {
            int motor;
            if (trajectory_it->QueryIntAttribute("motorid", &motor) == TIXML_SUCCESS) {

                stringstream stream(trajectory_it->GetText());
                while (1) {
                    int n;
                    stream >> n;
                    switch (motor) {
                        case 0:
                            msg.motor0.push_back(n);
                            break;
                        case 1:
                            msg.motor1.push_back(n);
                            break;
                        case 2:
                            msg.motor2.push_back(n);
                            break;
                        case 3:
                            msg.motor3.push_back(n);
                            break;
                        case 4:
                            msg.motor4.push_back(n);
                            break;
                        case 5:
                            msg.motor5.push_back(n);
                            break;
                        case 6:
                            msg.motor6.push_back(n);
                            break;
                        case 7:
                            msg.motor7.push_back(n);
                            break;
                        case 8:
                            msg.motor8.push_back(n);
                            break;
                        case 9:
                            msg.motor9.push_back(n);
                            break;
                        case 10:
                            msg.motor10.push_back(n);
                            break;
                        case 11:
                            msg.motor11.push_back(n);
                            break;
                        case 12:
                            msg.motor12.push_back(n);
                            break;
                        case 13:
                            msg.motor13.push_back(n);
                            break;
                        default:
                            ROS_ERROR("motorid %d is not available, aborting. check you recorded trajectory", motor);
                            return false;
                    }

                    if (!stream) {
                        numberOfSamples = msg.motor0.size();
                        break;
                    }
                }
            } else {
                ROS_ERROR("trajectory without motorid, aborting. check you trajectory file");
                return false;
            }
        }

        msg.samplingTime = samplingTime;
        msg.control_mode = control_mode;

        motorTrajectory.publish(msg);
        return true;
    }

    void MainWindow::stopMovement() {
        ROS_INFO("stop movement");
        roboy_communication_middleware::MotorTrajectoryControl msg;
        msg.play = false;
        msg.pause = ui.pause->isChecked();
        msg.rewind = false;
        msg.loop = ui.loop->isChecked();
        motorTrajectoryControl.publish(msg);
    }

    void MainWindow::rewindMovement() {
        ROS_INFO("rewind movement");
        roboy_communication_middleware::MotorTrajectoryControl msg;
        msg.play = true;
        msg.pause = ui.pause->isChecked();
        msg.rewind = true;
        msg.loop = ui.loop->isChecked();
        motorTrajectoryControl.publish(msg);
    }

    void MainWindow::pauseMovement() {
        ROS_INFO("pause movement");
        roboy_communication_middleware::MotorTrajectoryControl msg;
        msg.play = true;
        msg.pause = ui.pause->isChecked();
        msg.rewind = false;
        msg.loop = ui.loop->isChecked();
        motorTrajectoryControl.publish(msg);
    }

    void MainWindow::loopMovement() {
        ROS_INFO("loop movement");
        roboy_communication_middleware::MotorTrajectoryControl msg;
        msg.play = true;
        msg.pause = ui.pause->isChecked();
        msg.rewind = false;
        msg.loop = ui.loop->isChecked();
        motorTrajectoryControl.publish(msg);
    }

    void MainWindow::stopButtonClicked() {
        ROS_INFO("stop button clicked");
        if (!ui.stop_button_motorControl->isChecked() && !ui.stop_button_jointControl->isChecked()) {
            updateControllerParams();
        } else { // set controller gains to zero
            jointControl = false;
            motorControl = false;
            roboy_communication_middleware::MotorConfig msg;
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                msg.motors.push_back(motor);
                msg.control_mode.push_back(2);
                msg.outputPosMax.push_back(1000); // pwm max
                msg.outputNegMax.push_back(-1000); // pwm min
                msg.spPosMax.push_back(100000000);
                msg.spNegMax.push_back(-100000000);
                msg.IntegralPosMax.push_back(100);
                msg.IntegralNegMax.push_back(-100);
                msg.Kp.push_back(0);
                msg.Ki.push_back(0);
                msg.Kd.push_back(0);
                msg.forwardGain.push_back(0);
                msg.deadBand.push_back(0);
            }
            motorConfig.publish(msg);
        }
    }

    void MainWindow::danceToggle() {
        ROS_INFO("dance button clicked");
        dance = ui.dance->isChecked();
        if (dance) {
            jointControl = false;
            motorControl = false;

            setPointAngle[0] = -80;
            setPointAngle[1] = 80;
            setPointAngle[2] = 80;
            setPointAngle[3] = -80;
            tf::Vector3 pos(0.5, 0.5, 0);
            make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, pos, false, 0.1,
                           "ankle_left", "hip_position", "");
        }
    }

    void MainWindow::receiveImage(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv_ptr->image.copyTo(img);
            cv::resize(img, img, Size(), image_scale, image_scale, INTER_CUBIC);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        {
            Point pt1(arucoMarkerCenter[260][0], arucoMarkerCenter[260][1]);
            Point pt2(arucoMarkerCenter[429][0], arucoMarkerCenter[429][1]);
            cv::line(img, pt1, pt2, CV_RGB(0, 0, 255), 5);
        }
        {
            Point pt1(arucoMarkerCenter[868][0], arucoMarkerCenter[868][1]);
            Point pt2(arucoMarkerCenter[576][0], arucoMarkerCenter[576][1]);
            cv::line(img, pt1, pt2, CV_RGB(0, 0, 255), 5);
        }
        {
            Point pt1(arucoMarkerCenter[282][0], arucoMarkerCenter[282][1]);
            Point pt2(arucoMarkerCenter[754][0], arucoMarkerCenter[754][1]);
            cv::line(img, pt1, pt2, CV_RGB(0, 0, 255), 5);
        }
        {
            Point pt1(arucoMarkerCenter[1007][0], arucoMarkerCenter[1007][1]);
            Point pt2(arucoMarkerCenter[422][0], arucoMarkerCenter[422][1]);
            cv::line(img, pt1, pt2, CV_RGB(0, 0, 255), 5);
        }
        {
            Point pt1(arucoMarkerCenter[120][0], arucoMarkerCenter[120][1]);
            Point pt2(arucoMarkerCenter[100][0], arucoMarkerCenter[100][1]);
            cv::line(img, pt1, pt2, CV_RGB(0, 0, 255), 5);
        }
        Q_EMIT drawImage();
    }

    void MainWindow::displayImage() {
        int w = img.cols;
        int h = img.rows;
        QImage qim(w, h, QImage::Format_RGB32);
        QRgb pixel;
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                cv::Vec3b rgb = img.at<cv::Vec3b>(j, i);
                pixel = qRgb(rgb[2], rgb[1], rgb[0]);
                qim.setPixel(i, j, pixel);
            }
        }
        pixmap = QPixmap::fromImage(qim);
        ui.camera_image->setPixmap(pixmap);
        ui.camera_image->repaint();
    }

    void MainWindow::resetPose() {
        setPointAngle[0] = -80;
        setPointAngle[1] = 80;
        setPointAngle[2] = 80;
        setPointAngle[3] = -80;
        errorVisualServoing = Vector2d(0, 0);
        errorVisualServoing_prev = Vector2d(0, 0);
        resultVisualServoing = Vector2d(0, 0);
        integralVisualServoing = Vector2d(0, 0);
        integralVisualServoingMax = Vector2d(0, 0);
        integralVisualServoingMin = Vector2d(0, 0);
    }

    void MainWindow::danceController() {
        visualServoing = ui.visual_servoing->isChecked();
        ROS_INFO("visual servoing %s", (visualServoing ? "activated" : "deactivated"));
    }

    void MainWindow::JointCommand(const roboy_communication_middleware::JointCommand::ConstPtr &msg) {
        for (uint joint = 0; joint < msg->angle.size(); joint++)
            setPointAngle[joint] = msg->angle[joint];
    }

    float MainWindow::calculateAngleBetween(int aruco0, int aruco1, int aruco2, int aruco3) {
        Vector3f line0 = arucoMarkerPosition[aruco1] - arucoMarkerPosition[aruco0];
        Vector3f line1 = arucoMarkerPosition[aruco3] - arucoMarkerPosition[aruco2];

        float len1 = sqrtf(line0[0] * line0[0] + line0[1] * line0[1] + line0[2] * line0[2]);
        float len2 = sqrtf(line1[0] * line1[0] + line1[1] * line1[1] + line1[2] * line1[2]);

        float dot = line0[0] * line1[0] + line0[1] * line1[1] + line0[2] * line1[2];

        float a = dot / (len1 * len2);

        return acos(a) * 180.0f / (float) M_PI; // 0..PI
    }

    double MainWindow::calculateAngleBetween(Vector3d &sensor0, Vector3d &sensor1, Vector3d &axis) {
        Vector3d line0 = sensor1 - sensor0;
        line0.normalize();
        axis.normalize();
        float len1 = sqrtf(line0[0] * line0[0] + line0[1] * line0[1] + line0[2] * line0[2]);

        float dot = line0[0] * axis[0] + line0[1] * axis[1] + line0[2] * axis[2];

        float a = dot / (len1);

        return acos(a) * 180.0f / M_PI; // 0..PI
    }

    void MainWindow::DanceCommand(const roboy_communication_middleware::DanceCommand::ConstPtr &msg) {
        QString str;
        str.sprintf("target position %lf\t%lf\t%lf", msg->pos.x, msg->pos.y, msg->pos.z);
        ui.target_position->append(str);
        ui.target_position->verticalScrollBar()->setValue(ui.target_position->verticalScrollBar()->maximum());
    }

    void MainWindow::DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensor::ConstPtr &msg) {
        uint i = 0;
        for (auto position:msg->position) {
            sensor_position[sensor_map[msg->ids[i]]] = Vector3d(position.x, position.y, position.z);
            char str[10];
            sprintf(str, "%d", sensor_map[msg->ids[i]]);
            publishText(sensor_position[sensor_map[msg->ids[i]]], str, "world", "sensor_names",
                        i + 9000/*random message id*/,
                        COLOR(0, 1, 0, 0.1), 1, 0.05);
            i++;
        }
        if (sensor_position.find(3) == sensor_position.end() && sensor_position.find(5) == sensor_position.end()) {
            ROS_WARN("sensor 3 and/or 5 not active");
            return;
        }
        vector<int> ids = {0, 3, 5, 8};
        std::pair<Vector3d, Vector3d> plane = best_plane_from_points(sensor_position, ids);
//            ROS_INFO_STREAM_THROTTLE(1,"centroid\n" << plane.first << "\nnormal:\n" << plane.second);

        if (plane.second[1] < 0)
            plane.second = -plane.second;

        Vector3d hip_level = sensor_position[5] - sensor_position[3];
        Vector3d y = plane.second.cross(hip_level);
        Vector3d x = y.cross(plane.second);
        x.normalize();
        y.normalize();
        plane.second.normalize();
//        publishRay(plane.first,x,"world","pabi X", 666, COLOR(1,0,0,1), 1);
//        publishRay(plane.first,y,"world","pabi Y", 666, COLOR(0,1,0,1), 1);
//        publishRay(plane.first,plane.second,"world","pabi Z", 666, COLOR(0,0,1,1), 1);

        //calculate angle to world from hip orientation (sensor 3 and 5) via kinematic chain
        double angle = calculateAngleBetween(sensor_position[3], sensor_position[5], x);
        char str[20];
        sprintf(str, "%lf", angle);
        publishText(sensor_position[3], str, "world", "angle_to_world_hip", 667, COLOR(1, 0, 0, 1), 1, 0.05);
        phi = angle + jointData[0][2][1].back() + jointData[0][3][1].back();
        sprintf(str, "%lf", phi);
        publishText(sensor_position[0], str, "world", "angle_to_world_ankle_left", 668, COLOR(1, 0, 0, 1), 1, 0.05);

        tf::Matrix3x3 rot(x[0], y[0], plane.second[0], x[1], y[1], plane.second[1], x[2], y[2], plane.second[2]);
        relativeFrame.setOrigin(tf::Vector3(sensor_position[0][0], sensor_position[0][1], sensor_position[0][2]));
//        tf::Quaternion quat;
//        quat.setRPY(M_PI_2, 0, M_PI);
        relativeFrame.setBasis(rot);
        tf_broadcaster.sendTransform(tf::StampedTransform(relativeFrame, ros::Time::now(), "world", "ankle_left"));
        Q_EMIT DarkRoomSensorDataReady();
    }

    void MainWindow::VisualServoing() {
        static ros::Time control_prev = ros::Time::now();
        ros::Time control_cur = ros::Time::now();
        if (visualServoing && (control_cur - control_prev).nsec > 100000) {
            Vector4d hip_sensor(sensor_position[4][0], sensor_position[4][1], sensor_position[4][2],
                                1), hip_sensor_ankle_frame,
                    ankle_right(sensor_position[8][0], sensor_position[8][1], sensor_position[8][2], 1),
                    ankle_right_ankle_left_frame;
            Eigen::Affine3d trans_;
            tf::transformTFToEigen(relativeFrame, trans_);
            Matrix4d transform;
            transform = trans_.matrix();

            hip_sensor_ankle_frame = transform.inverse() * hip_sensor;
            ankle_right_ankle_left_frame = transform.inverse() * ankle_right;
            ROS_INFO_THROTTLE(1, "hip sensor in ankle frame %f\t%f\t%f", hip_sensor_ankle_frame[0],
                              hip_sensor_ankle_frame[1], hip_sensor_ankle_frame[2]);

            int messageId = 88888;

            roboy_communication_middleware::InverseKinematics service_msg;
            service_msg.request.targetPosition.x = targetPosition[0];
            service_msg.request.targetPosition.y = targetPosition[1];
            service_msg.request.targetPosition.z = targetPosition[2];
            service_msg.request.lighthouse_sensor_id = 4; // hip center lighthouse sensor
            service_msg.request.ankle_right_sensor.x = ankle_right_ankle_left_frame[0];
            service_msg.request.ankle_right_sensor.y = ankle_right_ankle_left_frame[1];
            service_msg.request.ankle_right_sensor.z = 0;
            service_msg.request.initial_angles.push_back(jointData[0][0][1].back());
            service_msg.request.initial_angles.push_back(jointData[0][1][1].back());
            service_msg.request.initial_angles.push_back(jointData[0][2][1].back());
            service_msg.request.initial_angles.push_back(jointData[0][3][1].back());
            service_msg.request.initial_angles.push_back(phi);
            if (!ik_srv.call(service_msg)) {
                ROS_ERROR_THROTTLE(1, "ik of result pose failed");
                return;
            }

            for (uint j = 1; j < 9; j++) {
                Vector3d pos0, pos1;
                pos0 = convertGeometryToEigen(service_msg.response.resultPosition[j - 1]);
                pos1 = convertGeometryToEigen(service_msg.response.resultPosition[j]);
                Vector3d dir = pos1 - pos0;
                publishRay(pos0, dir, "ankle_left", "ik_solution_result", messageId++, COLOR(0, 1, 0, 0.3), 0.1);
            }

            ROS_INFO_THROTTLE(1, "targetPosition:\n%lf\t%lf\t%lf\n"
                    "resultPosition:\n%lf\t%lf\t%lf",
                              targetPosition[0], targetPosition[1], targetPosition[2],
                              service_msg.response.resultPosition[4].x, service_msg.response.resultPosition[4].y,
                              service_msg.response.resultPosition[4].z);

            setPointAngle[0] = service_msg.response.angles[0];
            setPointAngle[1] = service_msg.response.angles[1];
            setPointAngle[2] = service_msg.response.angles[2];
            setPointAngle[3] = service_msg.response.angles[3];

//            Vector2d resultPosition(service_msg.response.resultPosition[4].x, service_msg.response.resultPosition[4].y);
//            Vector2d targetPosition(targetPosition[0] + sensor_position[0][0],
//                                    targetPosition[1] + sensor_position[0][1]);
//            errorVisualServoing = targetPosition - resultPosition;
//            errorVisualServoing_prev = errorVisualServoing;
//
//            Vector2d pterm = atof(ui.Kp_danceControl->text().toStdString().c_str()) * errorVisualServoing;
//            Vector2d dterm = atof(ui.Kd_danceControl->text().toStdString().c_str()) *
//                             (errorVisualServoing - errorVisualServoing_prev);
//            integralVisualServoing += atof(ui.Ki_danceControl->text().toStdString().c_str()) * errorVisualServoing;
//            if (integralVisualServoing[0] >= atof(ui.integralMaxX->text().toStdString().c_str())) {
//                integralVisualServoing[0] = integralVisualServoingMax[0];
//            } else if (integralVisualServoing[0] <= atof(ui.integralMinX->text().toStdString().c_str())) {
//                integralVisualServoing[0] = integralVisualServoingMin[0];
//            }
//            if (integralVisualServoing[1] >= atof(ui.integralMaxY->text().toStdString().c_str())) {
//                integralVisualServoing[1] = integralVisualServoingMax[1];
//            } else if (integralVisualServoing[1] <= atof(ui.integralMinY->text().toStdString().c_str())) {
//                integralVisualServoing[1] = integralVisualServoingMin[1];
//            }
//            resultVisualServoing = pterm + dterm + integralVisualServoing;
        }
    }

    void MainWindow::InteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
        QString str;
        str.sprintf("target position %lf\t%lf\t%lf", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        targetPosition[0] = msg->pose.position.x;
        targetPosition[1] = msg->pose.position.y;
        targetPosition[2] = msg->pose.position.z;
        ui.target_position->append(str);
        ui.target_position->verticalScrollBar()->setValue(ui.target_position->verticalScrollBar()->maximum());
    }

    std::pair<Vector3d, Vector3d> MainWindow::best_plane_from_points(map<int, Vector3d> &c, vector<int> &ids) {
        // copy coordinates to  matrix in Eigen format
        size_t num_atoms = ids.size();
        Eigen::Matrix<Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
        uint i = 0;
        for (auto &id:ids) {
            coord.col(i) = c[id];
            i++;
        }

        // calculate centroid
        Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

        // subtract centroid
        coord.row(0).array() -= centroid(0);
        coord.row(1).array() -= centroid(1);
        coord.row(2).array() -= centroid(2);

        // we only need the left-singular matrix here
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        Vector3d plane_normal = svd.matrixU().rightCols<1>();
        return std::make_pair(centroid, plane_normal);
    }

    bool MainWindow::danceTrajectory(roboy_communication_middleware::DanceTrajectory::Request &req,
                                     roboy_communication_middleware::DanceTrajectory::Response &res) {
        std::ifstream i(req.trajectory_name);
        if (!i.is_open()) {
            ROS_ERROR("could not find trajectory %s", req.trajectory_name.c_str());
            return false;
        }
        json j;
        i >> j;
        jointControl = false;
        motorControl = false,
                dance = true;
        for (uint k = 1; k < j["setpoints"].size(); k++) {
            ros::Duration d(((double)j["setpoints"][k][0] - (double)j["setpoints"][k - 1][0]));
            setPointAngle[0] = j["setpoints"][k - 1][1];
            setPointAngle[1] = j["setpoints"][k - 1][2];
            setPointAngle[2] = j["setpoints"][k - 1][3];
            setPointAngle[3] = j["setpoints"][k - 1][4];
            ROS_INFO_THROTTLE(1,"setPoint at %lf: %f\t%f\t%f\t%f", j["setpoints"][k][0], setPointAngle[0], setPointAngle[1],
                     setPointAngle[2], setPointAngle[3]);
            d.sleep();
        }
        return true;
    }

    void MainWindow::ArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr &msg) {
        ROS_INFO_THROTTLE(10, "receiving aruco pose");
        for (uint i = 0; i < msg->id.size(); i++) {
            arucoMarkerPosition[msg->id[i]] = Vector3f(msg->pose[i].position.x, msg->pose[i].position.y,
                                                       msg->pose[i].position.z);
            arucoMarkerCenter[msg->id[i]] = Vector2f(msg->center[i].x, msg->center[i].y) * image_scale;
        }

        Vector3f hipCenter = (arucoMarkerPosition[282] + (arucoMarkerPosition[754] - arucoMarkerPosition[282]) / 2.0f) -
                             arucoMarkerPosition[429];
        geometry_msgs::Pose hipCenterPose;
        hipCenterPose.position.x = hipCenter[0];
        hipCenterPose.position.y = -hipCenter[1];
        hipCenterPose.position.z = hipCenter[2];
        hipCenter_pub.publish(hipCenterPose);
    }

    void MainWindow::updateSetPointsMotorControl(int percent) {
        std::lock_guard<std::mutex> lock(myoMaster->mux);
        motorControl = true;
        jointControl = false;
        int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
        switch (ui.control_mode->value()) {
            case POSITION:
                setpoints[0] = ui.motor0->value() * 3000;
                setpoints[1] = ui.motor1->value() * 3000;
                setpoints[2] = ui.motor2->value() * 3000;
                setpoints[3] = ui.motor3->value() * 3000;
                setpoints[4] = ui.motor4->value() * 3000;
                setpoints[5] = ui.motor5->value() * 3000;
                setpoints[6] = ui.motor6->value() * 3000;
                setpoints[7] = ui.motor7->value() * 3000;
                setpoints[8] = ui.motor8->value() * 3000;
                setpoints[9] = ui.motor9->value() * 3000;
                setpoints[10] = ui.motor10->value() * 3000;
                setpoints[11] = ui.motor11->value() * 3000;
                setpoints[12] = ui.motor12->value() * 3000;
                setpoints[13] = ui.motor13->value() * 3000;
                break;
            case VELOCITY:
                setpoints[0] = ui.motor0->value();
                setpoints[1] = ui.motor1->value();
                setpoints[2] = ui.motor2->value();
                setpoints[3] = ui.motor3->value();
                setpoints[4] = ui.motor4->value();
                setpoints[5] = ui.motor5->value();
                setpoints[6] = ui.motor6->value();
                setpoints[7] = ui.motor7->value();
                setpoints[8] = ui.motor8->value();
                setpoints[9] = ui.motor9->value();
                setpoints[10] = ui.motor10->value();
                setpoints[11] = ui.motor11->value();
                setpoints[12] = ui.motor12->value();
                setpoints[13] = ui.motor13->value();
                break;
            case DISPLACEMENT:
                setpoints[0] = (ui.motor0->value() + 50) * 20;
                setpoints[1] = (ui.motor1->value() + 50) * 20;
                setpoints[2] = (ui.motor2->value() + 50) * 20;
                setpoints[3] = (ui.motor3->value() + 50) * 20;
                setpoints[4] = (ui.motor4->value() + 50) * 20;
                setpoints[5] = (ui.motor5->value() + 50) * 20;
                setpoints[6] = (ui.motor6->value() + 50) * 20;
                setpoints[7] = (ui.motor7->value() + 50) * 20;
                setpoints[8] = (ui.motor8->value() + 50) * 20;
                setpoints[9] = (ui.motor9->value() + 50) * 20;
                setpoints[10] = (ui.motor10->value() + 50) * 20;
                setpoints[11] = (ui.motor11->value() + 50) * 20;
                setpoints[12] = (ui.motor12->value() + 50) * 20;
                setpoints[13] = (ui.motor13->value() + 50) * 20;
                break;
        }
        for (int motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            myoMaster->changeSetPoint(motor, setpoints[motor]);
        }
        ui.setPoint_motor0->setText(QString::number(setpoints[0]));
        ui.setPoint_motor1->setText(QString::number(setpoints[1]));
        ui.setPoint_motor2->setText(QString::number(setpoints[2]));
        ui.setPoint_motor3->setText(QString::number(setpoints[3]));
        ui.setPoint_motor4->setText(QString::number(setpoints[4]));
        ui.setPoint_motor5->setText(QString::number(setpoints[5]));
        ui.setPoint_motor6->setText(QString::number(setpoints[6]));
        ui.setPoint_motor7->setText(QString::number(setpoints[7]));
        ui.setPoint_motor8->setText(QString::number(setpoints[8]));
        ui.setPoint_motor9->setText(QString::number(setpoints[9]));
        ui.setPoint_motor10->setText(QString::number(setpoints[10]));
        ui.setPoint_motor11->setText(QString::number(setpoints[11]));
        ui.setPoint_motor12->setText(QString::number(setpoints[12]));
        ui.setPoint_motor13->setText(QString::number(setpoints[13]));
    }

    void MainWindow::updateSetPointsMotorControlAll(int percent) {
        std::lock_guard<std::mutex> lock(myoMaster->mux);
        jointControl = false;
        motorControl = true;
        int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
        switch (ui.control_mode->value()) {
            case POSITION:
                setpoints[0] = ui.allMotors->value() * 3000;
                setpoints[1] = ui.allMotors->value() * 3000;
                setpoints[2] = ui.allMotors->value() * 3000;
                setpoints[3] = ui.allMotors->value() * 3000;
                setpoints[4] = ui.allMotors->value() * 3000;
                setpoints[5] = ui.allMotors->value() * 3000;
                setpoints[6] = ui.allMotors->value() * 3000;
                setpoints[7] = ui.allMotors->value() * 3000;
                setpoints[8] = ui.allMotors->value() * 3000;
                setpoints[9] = ui.allMotors->value() * 3000;
                setpoints[10] = ui.allMotors->value() * 3000;
                setpoints[11] = ui.allMotors->value() * 3000;
                setpoints[12] = ui.allMotors->value() * 3000;
                setpoints[13] = ui.allMotors->value() * 3000;
                break;
            case VELOCITY:
                setpoints[0] = ui.allMotors->value();
                setpoints[1] = ui.allMotors->value();
                setpoints[2] = ui.allMotors->value();
                setpoints[3] = ui.allMotors->value();
                setpoints[4] = ui.allMotors->value();
                setpoints[5] = ui.allMotors->value();
                setpoints[6] = ui.allMotors->value();
                setpoints[7] = ui.allMotors->value();
                setpoints[8] = ui.allMotors->value();
                setpoints[9] = ui.allMotors->value();
                setpoints[10] = ui.allMotors->value();
                setpoints[11] = ui.allMotors->value();
                setpoints[12] = ui.allMotors->value();
                setpoints[13] = ui.allMotors->value();
                break;
            case DISPLACEMENT:
                setpoints[0] = (ui.allMotors->value() + 50) * 20;
                setpoints[1] = (ui.allMotors->value() + 50) * 20;
                setpoints[2] = (ui.allMotors->value() + 50) * 20;
                setpoints[3] = (ui.allMotors->value() + 50) * 20;
                setpoints[4] = (ui.allMotors->value() + 50) * 20;
                setpoints[5] = (ui.allMotors->value() + 50) * 20;
                setpoints[6] = (ui.allMotors->value() + 50) * 20;
                setpoints[7] = (ui.allMotors->value() + 50) * 20;
                setpoints[8] = (ui.allMotors->value() + 50) * 20;
                setpoints[9] = (ui.allMotors->value() + 50) * 20;
                setpoints[10] = (ui.allMotors->value() + 50) * 20;
                setpoints[11] = (ui.allMotors->value() + 50) * 20;
                setpoints[12] = (ui.allMotors->value() + 50) * 20;
                setpoints[13] = (ui.allMotors->value() + 50) * 20;
                break;
        }
        for (int motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            myoMaster->changeSetPoint(motor, setpoints[motor]);
        }
        ui.setPoint_motor0->setText(QString::number(setpoints[0]));
        ui.setPoint_motor1->setText(QString::number(setpoints[1]));
        ui.setPoint_motor2->setText(QString::number(setpoints[2]));
        ui.setPoint_motor3->setText(QString::number(setpoints[3]));
        ui.setPoint_motor4->setText(QString::number(setpoints[4]));
        ui.setPoint_motor5->setText(QString::number(setpoints[5]));
        ui.setPoint_motor6->setText(QString::number(setpoints[6]));
        ui.setPoint_motor7->setText(QString::number(setpoints[7]));
        ui.setPoint_motor8->setText(QString::number(setpoints[8]));
        ui.setPoint_motor9->setText(QString::number(setpoints[9]));
        ui.setPoint_motor10->setText(QString::number(setpoints[10]));
        ui.setPoint_motor11->setText(QString::number(setpoints[11]));
        ui.setPoint_motor12->setText(QString::number(setpoints[12]));
        ui.setPoint_motor13->setText(QString::number(setpoints[13]));
    }

    void MainWindow::updateSetPointsJointControl(int val) {
        jointControl = true;
        int setpoints[NUMBER_OF_JOINT_SENSORS];

        setpoints[0] = ui.joint0->value();
        setpoints[1] = ui.joint1->value();
        setpoints[2] = ui.joint2->value();
        setpoints[3] = ui.joint3->value();

//    for(int motor=0;motor<NUMBER_OF_JOINT_SENSORS;motor++){
//        myoMaster->changeSetPoint(motor,setpoints[motor]);
//    }
        ui.setPoint_joint0->setText(QString::number(setpoints[0]));
        ui.setPoint_joint1->setText(QString::number(setpoints[1]));
        ui.setPoint_joint2->setText(QString::number(setpoints[2]));
        ui.setPoint_joint3->setText(QString::number(setpoints[3]));
    }

    void MainWindow::updateSetPointsJointControlAll(int val) {
        jointControl = true;
        int setpoints[NUMBER_OF_JOINT_SENSORS];

        setpoints[0] = ui.allJoints->value();
        setpoints[1] = ui.allJoints->value();
        setpoints[2] = ui.allJoints->value();
        setpoints[3] = ui.allJoints->value();

//    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
//        myoMaster->changeSetPoint(motor,setpoints[motor]);
//    }
        ui.setPoint_joint0->setText(QString::number(setpoints[0]));
        ui.setPoint_joint1->setText(QString::number(setpoints[1]));
        ui.setPoint_joint2->setText(QString::number(setpoints[2]));
        ui.setPoint_joint3->setText(QString::number(setpoints[3]));
    }

    void MainWindow::updateControllerParams() {
        ui.stop_button_motorControl->setChecked(false);
        roboy_communication_middleware::MotorConfig msg;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            msg.motors.push_back(motor);
            msg.control_mode.push_back(ui.control_mode->value());
            int outputMax = atoi(ui.outputMax->text().toStdString().c_str());
            if (outputMax >= 0 && outputMax <= 4000) {
                msg.outputPosMax.push_back(outputMax); // pwm max
                msg.outputNegMax.push_back(-outputMax); // pwm min
            } else {
                msg.outputPosMax.push_back(1000); // pwm max
                msg.outputNegMax.push_back(-1000); // pwm min
            }
            msg.spPosMax.push_back(100000000);
            msg.spNegMax.push_back(-100000000);
            msg.IntegralPosMax.push_back(100);
            msg.IntegralNegMax.push_back(-100);
            msg.Kp.push_back(atoi(ui.Kp->text().toStdString().c_str()));
            msg.Ki.push_back(atoi(ui.Ki->text().toStdString().c_str()));
            msg.Kd.push_back(atoi(ui.Kd->text().toStdString().c_str()));
            msg.forwardGain.push_back(0);
            msg.deadBand.push_back(atoi(ui.deadBand->text().toStdString().c_str()));
        }
        motorConfig.publish(msg);
    }

    void MainWindow::movementPathChanged() {
        model->setRootPath(QDir::currentPath());
        ui.movementFolder->setModel(model);
    }

    void MainWindow::recordMovement() {
        ROS_INFO("start recording");
        roboy_communication_middleware::MotorRecordConfig msg;
        msg.ids = 0;
        numberOfRecordsToWaitFor = 0;
        if (ui.record_fpga0->isChecked()) {
            msg.ids |= (1 << 0);
            numberOfRecordsToWaitFor++;
        }
        if (ui.record_fpga1->isChecked()) {
            msg.ids |= (1 << 1);
            numberOfRecordsToWaitFor++;
        }
        if (ui.record_fpga2->isChecked()) {
            msg.ids |= (1 << 2);
            numberOfRecordsToWaitFor++;
        }
        if (ui.record_fpga3->isChecked()) {
            msg.ids |= (1 << 3);
            numberOfRecordsToWaitFor++;
        }
        if (ui.record_fpga4->isChecked()) {
            msg.ids |= (1 << 4);
            numberOfRecordsToWaitFor++;
        }

        msg.control_mode = ui.control_mode->value();
        msg.samplingTime = atoi(ui.samplingTime->text().toStdString().c_str());
        msg.recordTime = atoi(ui.recordTime->text().toStdString().c_str());
        motorRecordConfig.publish(msg);
    }

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

    void MainWindow::on_actionAbout_triggered() {
        QMessageBox::about(this, tr("About ..."),
                           tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
    }

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

    void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
    }

    void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    }

    void MainWindow::closeEvent(QCloseEvent *event) {
        WriteSettings();
        QMainWindow::closeEvent(event);
    }

}  // namespace interface

