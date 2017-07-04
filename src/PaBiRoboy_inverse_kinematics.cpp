/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *               This file is part of 'PaBiRoboy_DanceControl'                *
 ******************************************************************************/
#include "interface/PaBiRoboy_inverse_kinematics.hpp"
#include <math.h>

PaBiRoboyInverseKinematics::PaBiRoboyInverseKinematics() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "PaBiRoboy_inverseKinematis");
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ik_srv = nh->advertiseService("/roboy/middleware/PaBiRoboy/inverseKinematics", &PaBiRoboyInverseKinematics::inverseKinematics, this);
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(1.5,0.5,-1.0,0,1,0);
}

bool PaBiRoboyInverseKinematics::inverseKinematics(roboy_communication_middleware::InverseKinematics::Request &req,
                                                    roboy_communication_middleware::InverseKinematics::Response &res){
    vector<double> q = {degreesToRadians(req.initial_angles[0]),
                        degreesToRadians(req.initial_angles[1]),
                        degreesToRadians(req.initial_angles[2]),
                        degreesToRadians(req.initial_angles[3]),
                        0};
    double data[3];
    ankle_left(initial_position["ankle_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    knee_left(initial_position["knee_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_left(initial_position["hip_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_center(initial_position["hip_center"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_right(initial_position["hip_right"].data(),q[0],q[1],q[2],q[3],q[4]);
    knee_right(initial_position["knee_right"].data(),q[0],q[1],q[2],q[3],q[4]);
    ankle_right(initial_position["ankle_right"].data(),q[0],q[1],q[2],q[3],q[4]);

    // we are trying to keep the ankle_right where it is
    Vector4d setPoint(initial_position["ankle_right"][0], 0,req.targetPosition.x, req.targetPosition.y);

    Matrix4d kp;
    kp << 10, 0, 0,  0,
          0, 10, 0,  0,
          0, 0, 10, 0,
          0, 0, 0,  10;

    double error;

    boost::numeric::odeint::runge_kutta4<vector<double>> stepper;
    uint iter = 0;
    do{
        // do 1 step of integration of DiffModel() at current time
        stepper.do_step([this, &setPoint, kp, &error](const vector<double> &q, vector<double> &dq, const double) {
            // This lambda function implements the inverse kinematics for PaBiLegs
            // q - joint angles
            // dq - joint angles derivatives
            Matrix<double,4,5,RowMajor> J;
            Matrix<double,5,4,RowMajor> Jpinv;
            double data[20];
            Jacobian(data,q[0],q[1],q[2],q[3],q[4]);
            J = Eigen::Map<Matrix<double,4,5,RowMajor>>(data, 4,5);
            Vector4d angles(q.data());
            Vector4d x_current;
            ankle_right_hip_center(x_current.data(),q[0],q[1],q[2],q[3],q[4]);
            Jpinv = pseudoInverse<Matrix<double,4,5,RowMajor>, Matrix<double,5,4,RowMajor>>(J);
            VectorXd dangles(5);
            dangles = Jpinv * (kp * (setPoint - x_current));
            memcpy(dq.data(), dangles.data(), 5 * sizeof(double));
            error = sqrt(pow(x_current[2]-setPoint[2],2.0)+pow(x_current[3]-setPoint[3],2.0));
        }, q, 0.01, 0.01);
    }while(iter++<max_number_iterations && error>error_threshold);

    Vector4d x_current;
    ankle_right_hip_center(x_current.data(),q[0],q[1],q[2],q[3],q[4]);
    ROS_INFO("\ninitial_angles: %lf\t%lf\t%lf\t%lf"
            "\ntarget_point: ankle_right(%lf\t%lf)\thip_center(%lf\t%lf)"
             "\nresult:       ankle_right(%lf\t%lf)\thip_center(%lf\t%lf)\nerror: %f[m]\niterations: %d",
             req.initial_angles[0], req.initial_angles[1], req.initial_angles[2], req.initial_angles[3],
             setPoint[0],setPoint[1],setPoint[2],setPoint[3],
             x_current[0],x_current[1],x_current[2],x_current[3], error, iter);

    ankle_left(result_position["ankle_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    knee_left(result_position["knee_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_left(result_position["hip_left"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_center(result_position["hip_center"].data(),q[0],q[1],q[2],q[3],q[4]);
    hip_right(result_position["hip_right"].data(),q[0],q[1],q[2],q[3],q[4]);
    knee_right(result_position["knee_right"].data(),q[0],q[1],q[2],q[3],q[4]);
    ankle_right(result_position["ankle_right"].data(),q[0],q[1],q[2],q[3],q[4]);

    if(req.inspect)
        visualize();

    if(error > error_threshold)
        return false;

    // check joint limits
    if(abs(radiansToDegrees(q[0]))>80.0 || abs(radiansToDegrees(q[1]))>80.0 ||
            abs(radiansToDegrees(q[2]))>80.0 || abs(radiansToDegrees(q[3]))>80.0)
        return false;

    res.angles.push_back(radiansToDegrees(q[0]));
    res.angles.push_back(radiansToDegrees(q[1]));
    res.angles.push_back(radiansToDegrees(q[2]));
    res.angles.push_back(radiansToDegrees(q[3]));
    return true;
}

void PaBiRoboyInverseKinematics::Jacobian(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) - l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
    out[1] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
    out[2] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l2*cos(phi + theta0 + theta1 + theta2);
    out[3] = l1*cos(phi + theta0 + theta1 + theta2 + theta3);
    out[4] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) - l1*cos(phi) - l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
    out[5] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[6] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[7] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) + l2*sin(phi + theta0 + theta1 + theta2);
    out[8] = l1*sin(phi + theta0 + theta1 + theta2 + theta3);
    out[9] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[10] = -l2*cos(phi + theta0) - 1.0L/2.0L*l3*sin(phi + theta0 + theta1);
    out[11] = -1.0L/2.0L*l3*sin(phi + theta0 + theta1);
    out[12] = 0;
    out[13] = 0;
    out[14] = -l1*cos(phi) - l2*cos(phi + theta0) - 1.0L/2.0L*l3*sin(phi + theta0 + theta1);
    out[15] = -l2*sin(phi + theta0) + (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
    out[16] = (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
    out[17] = 0;
    out[18] = 0;
    out[19] = -l1*sin(phi) - l2*sin(phi + theta0) + (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
}
void PaBiRoboyInverseKinematics::ankle_right_hip_center(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[1] = -l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1);
    out[2] = -l1*sin(phi) - l2*sin(phi + theta0) + (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
    out[3] = l1*cos(phi) + l2*cos(phi + theta0) + (1.0L/2.0L)*l3*sin(phi + theta0 + theta1);
}
void PaBiRoboyInverseKinematics::ankle_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::knee_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = -l1*sin(phi);
    out[1] = l1*cos(phi);
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::hip_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = -l1*sin(phi) - l2*sin(phi + theta0);
    out[1] = l1*cos(phi) + l2*cos(phi + theta0);
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::hip_center(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = -l1*sin(phi) - l2*sin(phi + theta0) + (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
    out[1] = l1*cos(phi) + l2*cos(phi + theta0) + (1.0L/2.0L)*l3*sin(phi + theta0 + theta1);
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::hip_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = -l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1);
    out[1] = l1*cos(phi) + l2*cos(phi + theta0) + l3*sin(phi + theta0 + theta1);
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::knee_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = -l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[1] = l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1);
    out[2] = 0;
}
void PaBiRoboyInverseKinematics::ankle_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi) {
    out[0] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
    out[1] = -l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1);
    out[2] = 0;
}

void PaBiRoboyInverseKinematics::visualize(){
    viewer->removeAllShapes();

    viewer->addSphere (pcl::PointXYZ(initial_position["ankle_left"][0],
                                     initial_position["ankle_left"][1],
                                     initial_position["ankle_left"][2]), 0.01, 0, 0, 1.0, "ankle_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["knee_left"][0],
                                     initial_position["knee_left"][1],
                                     initial_position["knee_left"][2]), 0.01, 0, 0, 1.0, "knee_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_left"][0],
                                     initial_position["hip_left"][1],
                                     initial_position["hip_left"][2]), 0.01, 0, 0, 1.0, "hip_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_center"][0],
                                     initial_position["hip_center"][1],
                                     initial_position["hip_center"][2]), 0.01, 0, 0, 1.0, "hip_center_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_right"][0],
                                     initial_position["hip_right"][1],
                                     initial_position["hip_right"][2]), 0.01, 0, 0, 1.0, "hip_right_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["knee_right"][0],
                                     initial_position["knee_right"][1],
                                     initial_position["knee_right"][2]), 0.01, 0, 0, 1.0, "knee_right_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["ankle_right"][0],
                                     initial_position["ankle_right"][1],
                                     initial_position["ankle_right"][2]), 0.01, 0, 0, 1.0, "ankle_right_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["ankle_left"][0],
                                                  initial_position["ankle_left"][1],
                                                  initial_position["ankle_left"][2]),
                                    pcl::PointXYZ(initial_position["knee_left"][0],
                                                  initial_position["knee_left"][1],
                                                  initial_position["knee_left"][2]), "lower_leg_left_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["knee_left"][0],
                                                  initial_position["knee_left"][1],
                                                  initial_position["knee_left"][2]),
                                    pcl::PointXYZ(initial_position["hip_left"][0],
                                                  initial_position["hip_left"][1],
                                                  initial_position["hip_left"][2]), "upper_leg_left_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["hip_left"][0],
                                                  initial_position["hip_left"][1],
                                                  initial_position["hip_left"][2]),
                                    pcl::PointXYZ(initial_position["hip_right"][0],
                                                  initial_position["hip_right"][1],
                                                  initial_position["hip_right"][2]), "hip_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["hip_right"][0],
                                                  initial_position["hip_right"][1],
                                                  initial_position["hip_right"][2]),
                                    pcl::PointXYZ(initial_position["knee_right"][0],
                                                  initial_position["knee_right"][1],
                                                  initial_position["knee_right"][2]), "upper_leg_right_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["knee_right"][0],
                                                  initial_position["knee_right"][1],
                                                  initial_position["knee_right"][2]),
                                    pcl::PointXYZ(initial_position["ankle_right"][0],
                                                  initial_position["ankle_right"][1],
                                                  initial_position["ankle_right"][2]), "lower_leg_right_initial");

    viewer->addSphere (pcl::PointXYZ(result_position["ankle_left"][0],
                                     result_position["ankle_left"][1],
                                     result_position["ankle_left"][2]), 0.01, 0, 1.0, 0, "ankle_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["knee_left"][0],
                                     result_position["knee_left"][1],
                                     result_position["knee_left"][2]), 0.01, 0, 1.0, 0, "knee_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_left"][0],
                                     result_position["hip_left"][1],
                                     result_position["hip_left"][2]), 0.01, 0, 1.0, 0, "hip_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_center"][0],
                                     result_position["hip_center"][1],
                                     result_position["hip_center"][2]), 0.01, 0, 1.0, 0, "hip_center_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_right"][0],
                                     result_position["hip_right"][1],
                                     result_position["hip_right"][2]), 0.01, 0, 1.0, 0, "hip_right_result");
    viewer->addSphere (pcl::PointXYZ(result_position["knee_right"][0],
                                     result_position["knee_right"][1],
                                     result_position["knee_right"][2]), 0.01, 0, 1.0, 0, "knee_right_result");
    viewer->addSphere (pcl::PointXYZ(result_position["ankle_right"][0],
                                     result_position["ankle_right"][1],
                                     result_position["ankle_right"][2]), 0.01, 0, 1.0, 0, "ankle_right_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["ankle_left"][0],
                                                  result_position["ankle_left"][1],
                                                  result_position["ankle_left"][2]),
                                    pcl::PointXYZ(result_position["knee_left"][0],
                                                  result_position["knee_left"][1],
                                                  result_position["knee_left"][2]), "lower_leg_left_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["knee_left"][0],
                                                  result_position["knee_left"][1],
                                                  result_position["knee_left"][2]),
                                    pcl::PointXYZ(result_position["hip_left"][0],
                                                  result_position["hip_left"][1],
                                                  result_position["hip_left"][2]), "upper_leg_left_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["hip_left"][0],
                                                  result_position["hip_left"][1],
                                                  result_position["hip_left"][2]),
                                    pcl::PointXYZ(result_position["hip_right"][0],
                                                  result_position["hip_right"][1],
                                                  result_position["hip_right"][2]), "hip_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["hip_right"][0],
                                                  result_position["hip_right"][1],
                                                  result_position["hip_right"][2]),
                                    pcl::PointXYZ(result_position["knee_right"][0],
                                                  result_position["knee_right"][1],
                                                  result_position["knee_right"][2]), "upper_leg_right_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["knee_right"][0],
                                                  result_position["knee_right"][1],
                                                  result_position["knee_right"][2]),
                                    pcl::PointXYZ(result_position["ankle_right"][0],
                                                  result_position["ankle_right"][1],
                                                  result_position["ankle_right"][2]), "lower_leg_right_result");
    viewer->spinOnce();
}

int main(int argc, char *argv[]){
    PaBiRoboyInverseKinematics ik;
    ros::spin();
}