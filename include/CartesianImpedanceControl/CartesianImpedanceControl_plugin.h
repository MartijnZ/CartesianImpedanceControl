/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef CartesianImpedanceControl_PLUGIN_H_
#define CartesianImpedanceControl_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames_io.hpp>
#include <cmath>
#include <eigen_conversions/eigen_kdl.h>
#include "ros/ros.h"

#define PI 3.14159265

using namespace Eigen;

namespace XBotPlugin {

enum EControlMode{
    Idle,
    GravityCompensate,
    Impedance
};



/**
 * @brief CartesianImpedanceControl XBot RT Plugin
 *
 **/
class CartesianImpedanceControl : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~CartesianImpedanceControl();

protected:

    virtual void control_loop(double time, double period);

private:

    void compute_error(Matrix<double, 12,1> &err);

    XBot::RobotInterface::Ptr _robot; 
    XBot::ModelInterface::Ptr _model;  
    XBot::MatLogger::Ptr _logger;

    double _start_time;


    // Gains and setpoints
    Matrix<double, 12, 12> _K;
    Matrix<double, 12, 1>  _setpoint;
    EControlMode ControlMode = EControlMode::Idle;

    // Robot interface:

    // Joint velocity
    VectorXd _q;
    VectorXd _q0;

    Eigen::Affine3d _dPoseLeft;
    Eigen::Affine3d _dPoseRight;
    Eigen::Affine3d _cPoseLeft;
    Eigen::Affine3d _cPoseRight;

    // Error values:
    Eigen::Matrix<double, 3,1> _wErrLeft;
    Eigen::Matrix<double, 3,1> _pErrLeft;
    Eigen::Matrix<double, 3,1> _wErrRight;
    Eigen::Matrix<double, 3,1> _pErrRight;
    Eigen::Matrix<double,12,1> _err;

    KDL::Jacobian   _J;   // Jacobian
    KDL::Jacobian   _Jinv;// Jacobian Inverse
    Eigen::MatrixXd _M;   // Mass Matrix
    Eigen::VectorXd _n;   // Non-linear term including centrifugal/coriolis/gravity
    Eigen::VectorXd _u;   // Joint Effort 


};

// Define Quaternion log:
double sc = 0.0;
double acos_val = 0.0;
Quaterniond qtmp;
Matrix<double, 3,1> qvec;
void arccos_star(double& res, const double& rho);
void Quaternionlog(Matrix<double, 3,1>& w, const Quaterniond& q, const Quaterniond& base);


}

#endif // CartesianImpedanceControl_PLUGIN_H_
