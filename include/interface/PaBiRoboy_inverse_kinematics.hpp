#pragma once

#include <ros/ros.h>
#include "roboy_communication_middleware/InverseKinematics.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "common_utilities/CommonDefinitions.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <thread>
#include <mutex>

using namespace Eigen;
using namespace std;

class PaBiRoboyInverseKinematics{
public:
    PaBiRoboyInverseKinematics();
    ~PaBiRoboyInverseKinematics(){};

    template<typename MatrixTypeIN, typename MatrixTypeOUT>
    MatrixTypeOUT pseudoInverse(const MatrixTypeIN &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< MatrixTypeIN > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    bool inverseKinematics(roboy_communication_middleware::InverseKinematics::Request &req,
                               roboy_communication_middleware::InverseKinematics::Response &res);
    /**
     * Calculates the Jacobian of ankle_right and hip_center
     * @param out 4x4 Matrix will be filled with the values
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void Jacobian(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of ankle right and hip center (ankle_right_x, ankle_right_y, hip_center_x, hip_center_y)
     * @param out 4x1 vector
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void ankle_right_hip_center(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of ankle_left
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void ankle_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of knee_left
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void knee_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of hip_left
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void hip_left(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of hip_center
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void hip_center(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of hip_right
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void hip_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of knee_right
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void knee_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of ankle_right
     * @param out
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle between ankle_left and origin frame
     */
    void ankle_right(double *out, double theta0, double theta1, double theta2, double theta3, double phi);

    void visualize();

private:
    ros::NodeHandlePtr nh;
    ros::ServiceServer ik_srv;
    const double l1 = 0.32;
    const double l2 = 0.39;
    const double l3 = 0.18;
    const double error_threshold = 0.005, max_number_iterations = 1000;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    map<string,Vector3d> initial_position, result_position;
};

int main(int argc, char* argv[]);

