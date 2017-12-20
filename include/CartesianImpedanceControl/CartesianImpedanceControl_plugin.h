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
#include <eigen_conversions/eigen_kdl.h>
#include "ros/ros.h"

using namespace Eigen;

namespace XBotPlugin {
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

    void compute_error(MatrixXd &err);
    void Rtoq(const KDL::Rotation &R, Eigen::Quaterniond& q);

    XBot::RobotInterface::Ptr _robot; 
    XBot::ModelInterface::Ptr _model;  

    double _start_time;

    XBot::MatLogger::Ptr _logger;

    // Gains and setpoints
    Matrix<double, 12, 12> _K;
    Matrix<double, 12, 1>  _setpoint;

    // Robot interface:

    // Joint velocity
    VectorXd _q;
    VectorXd _q0;

    double xtmp, ytmp, ztmp, wtmp; // Temporal variables for copying of quaternion information
    KDL::Frame _dPoseLeft;
    KDL::Frame _dPoseRight;
    Eigen::Quaterniond _dqLeft;
    Eigen::Quaterniond _dqRight;
    KDL::Frame _cPoseLeft;
    KDL::Frame _cPoseRight;
    Eigen::Quaterniond _cqLeft;
    Eigen::Quaterniond _cqRight;

    // Error values:
    Eigen::AngleAxisd _qerrLeft;
    Eigen::AngleAxisd _qerrRight;
    KDL::Vector _PerrLeft;
    KDL::Vector _PerrRight;
    Eigen::VectorXd _err;

    KDL::Jacobian   _J;   // Jacobian
    KDL::Jacobian   _Jinv;// Jacobian Inverse
    Eigen::MatrixXd _M;   // Mass Matrix
    Eigen::VectorXd _n;   // Non-linear term including centrifugal/coriolis/gravity
    Eigen::VectorXd _u;   // Joint Effort 

    bool is_activated;

};

}

#endif // CartesianImpedanceControl_PLUGIN_H_
