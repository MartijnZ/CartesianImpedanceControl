/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Martijn Zeestraten
 * email: martijnzeestraten@gmail.com
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

#include <CartesianImpedanceControl_plugin.h>

/* Specify that the class XBotPlugin::CartesianImpedanceControl is a XBot RT plugin with name "CartesianImpedanceControl" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::CartesianImpedanceControl)

namespace XBotPlugin {

bool CartesianImpedanceControl::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */

    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/CartesianImpedanceControl_log");


    // Set controller values:
    _K.setIdentity();
    _err.setZero();
    _wErrLeft.setZero();
    _pErrLeft.setZero();
    _wErrRight.setZero();
    _pErrRight.setZero();

    return true;


}

void CartesianImpedanceControl::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /CartesianImpedanceControl_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
    
    /* Start rosnode
     */
}

void CartesianImpedanceControl::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /CartesianImpedanceControl_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void CartesianImpedanceControl::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /CartesianImpedanceControl_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    if(!current_command.str().empty()){

        if (current_command.str() == "Impedance"){
            ControlMode = EControlMode::Impedance;
        }
        else if (current_command.str() == "GravityCompensate"){
            ControlMode = EControlMode::GravityCompensate;
        }
        else {
            ControlMode = EControlMode::Idle;
        }

    }

    // Get values from model:
    _model->syncFrom(*_robot);
    _model->computeNonlinearTerm( _n);
    _model->getInertiaMatrix( _M);
    _model->getJacobian( "ft_arm1", _J1); // Determine which Jacobian this is, and which one we need
    _model->getJacobian( "ft_arm2", _J2); // Determine which Jacobian this is, and which one we need

    switch (ControlMode)
    {
        case EControlMode::Idle:
            // What is the best idle mode?
            break;

        case EControlMode::GravityCompensate:
            _u = _n;
            _model->setJointEffort(_u);

            _robot->setReferenceFrom(*_model, XBot::Sync::Effort);
            _robot->move();
            break;

        case EControlMode::Impedance:
            // Compute error:
            compute_error(_err);

            // Compute control command:
            //_u = _M*_Jinv.data*(_K*_err) + _n;
            _model->setJointEffort(_u);

            _robot->setReferenceFrom(*_model, XBot::Sync::Effort);
            _robot->move();
            break;
    }

}

bool CartesianImpedanceControl::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

CartesianImpedanceControl::~CartesianImpedanceControl()
{
  
}


void CartesianImpedanceControl::compute_error(Matrix<double, 12,1>& err)
{
    /* Compute control error
     */ 

    _model->getPose("torso_2", "ft_arm1", _cPoseLeft);  // left end-effector
    _model->getPose("torso_2", "ft_arm2", _cPoseRight); // right right-endeffector

    // Compute errors (are these operations thread safe?)
    // e.g. Quaterniond() will initialize a new object when called, or is this handled by the compiler?
    _pErrLeft = _cPoseLeft.translation() - _dPoseLeft.translation(); 
    Quaternionlog(_wErrLeft,  Quaterniond(_cPoseLeft.rotation()), 
                              Quaterniond(_dPoseLeft.rotation()));

    Quaternionlog(_wErrRight,  Quaterniond(_cPoseRight.rotation()), 
                              Quaterniond(_dPoseRight.rotation()));
    _pErrRight = _cPoseRight.translation() - _dPoseRight.translation(); 
    
    // Copy elements:
    for (int i=0;i++;i<3)
    {
        err(i,0)  = _pErrLeft[i];
        err(i+3,0)= _wErrLeft[i];
        err(i+6,0)= _pErrRight[i];
        err(i+9,0)= _wErrRight[i];

    }
}


void Quaternionlog(Matrix<double, 3,1>& w, const Quaterniond& q, const Quaterniond& base)
{
    /* Compute Quaternion logarithm
     */
    qtmp = base.inverse()*q; // Is this thread safe?

    qvec(0) = qtmp.x();
    qvec(1) = qtmp.y();
    qvec(2) = qtmp.z();
    sc      = qtmp.w();

    if (std::abs(sc - 1.0) < 1e-6)
    {
        arccos_star(acos_val, sc);
        w = acos_val*qvec.normalized();
    }
    else
    {
        w.setZero();
    }
}

void arccos_star(double& res, const double& rho)
{
    /* Compute modified arccos, to ensure measuring minimum distance between 
     * two orientations expressed as quaternions
     */
    if (-1.0 <= rho && rho < 0.0)
    {
        res = acos(rho) - PI;
    }
    else
    {
        res = acos(rho);
    }
}

} // Close XBot Namespace
