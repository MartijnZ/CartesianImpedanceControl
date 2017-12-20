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

        if(current_command.str() == "Activate"){
            this->is_activated = true;
        }

        if(current_command.str() == "Deactivate"){
            this->is_activated = false;
        }

    }

    if (this->is_activated)
    {

        // Update robot state
        
        // Get values from model:
        _model->syncFrom(*_robot);
        _model->computeNonlinearTerm( this->_n);
        _model->getInertiaMatrix( this->_M);
        _model->getJacobian( "SomeLink", this->_J); // Determine which Jacobian this is, and which one we need


        // Compute error:


        // Compute control command:
        _u = _M*_Jinv.data*(_K*_err) + _n;
        _model->setJointEffort(_u);


        // Update robot state:
        _robot->setReferenceFrom(*_model, XBot::Sync::Effort);
        _robot->move();


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

void CartesianImpedanceControl::Rtoq(const KDL::Rotation &R, Eigen::Quaterniond& q)
{
    //R.GetQuaternion(q.x, q.y, q.z, q.w);
    R.GetQuaternion(xtmp, ytmp, ztmp, wtmp);
    q = Eigen::Quaterniond(xtmp, ytmp, ztmp, wtmp);
}

void CartesianImpedanceControl::compute_error(MatrixXd & err)
{
    // Compute 
    _model->getPose("source", "target", this->_cPoseLeft);  // left end-effector
    _model->getPose("source", "target", this->_cPoseRight); // right right-endeffector

    // Get quaternions:
    Rtoq(this->_cPoseLeft.M, this->_cqLeft);
    Rtoq(this->_dPoseLeft.M, this->_dqLeft);
    Rtoq(this->_cPoseRight.M, this->_cqRight);
    Rtoq(this->_dPoseRight.M, this->_dqRight);

    // Compute errors:
    _qerrLeft = Eigen::AngleAxisd(_cqLeft.inverse()*_dqLeft);
    _PerrLeft = _cPoseLeft.p - _dPoseLeft.p; 

    _qerrRight = Eigen::AngleAxisd(_cqRight.inverse()*_dqRight);
    _PerrRight = _cPoseRight.p - _dPoseRight.p; 
    
    // Copy elements:
    for (int i=0;i++;i<3)
    {
        err[i]   = _PerrLeft[i];
        err[i+3] = _PerrLeft[i];
        err[i+6] = _PerrRight[i];
        err[i+9] = _PerrLeft[i];

    }








}

}
