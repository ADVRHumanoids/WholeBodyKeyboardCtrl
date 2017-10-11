/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
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

#ifndef __WBC_ACTION_H__
#define __WBC_ACTION_H__

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <wholebody_keyboard_ctrl/LegMovementAction.h>
#include <actionlib/server/simple_action_server.h>

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Contact.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <OpenSoT/tasks/velocity/RigidRotation.h>
#include <qpOASES/Options.hpp>
#include <OpenSoT/SubTask.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/functional.hpp>


namespace centauro {
    
    namespace VelocityTask = OpenSoT::tasks::velocity;
    namespace VelocityConstraint = OpenSoT::constraints::velocity;
    
    class LegMovementAction {
        
    public:
        
        LegMovementAction(XBot::ModelInterface::Ptr model,
                          std::shared_ptr<std::mutex> model_mtx,
                          std::vector<std::string> feet_links);
        
    private:
        
        typedef actionlib::SimpleActionServer<wholebody_keyboard_ctrl::LegMovementAction> ActionServer;
        
        void execute_cb(const wholebody_keyboard_ctrl::LegMovementGoalConstPtr& goal);
        
        bool check_tol(double tol);
        
        ros::NodeHandle _nh;
        
        std::shared_ptr< ActionServer > _action_server;
        
        double _loop_rate;
        
        std::vector<Eigen::Affine3d> _goal_pos;
        std::vector<bool> _wheel_enabled;
        
        XBot::ModelInterface::Ptr _model;
        std::shared_ptr<std::mutex> _model_mtx;
        
        std::vector<std::string> _feet_links;
        int _num_feet;
        
        Eigen::VectorXd _q, _qmin, _qmax, _qdotmax, _dq;
        
        std::vector<VelocityTask::Cartesian::Ptr> _feet_world_cartesian_tasks;
        std::vector<VelocityTask::Cartesian::Ptr> _feet_waist_cartesian_tasks;
        VelocityTask::Cartesian::Ptr _left_arm_cartesian, _right_arm_cartesian;
        
        std::vector<VelocityTask::PureRolling::Ptr> rolling_tasks;
        std::vector<VelocityTask::RigidRotation::Ptr> icr_tasks;
        
        VelocityConstraint::JointLimits::Ptr _joint_pos_lims;
        VelocityConstraint::VelocityLimits::Ptr _joint_vel_lims;
        
        OpenSoT::AutoStack::Ptr _autostack;
        OpenSoT::solvers::QPOases_sot::Ptr _solver;
        
    };

}


centauro::LegMovementAction::LegMovementAction(XBot::ModelInterface::Ptr model, 
                                               std::shared_ptr<std::mutex> model_mtx,
                                               std::vector< std::string > feet_links):
    _model(model),
    _feet_links(feet_links),
    _num_feet(feet_links.size()),
    _loop_rate(100),
    _goal_pos(feet_links.size(), Eigen::Affine3d::Identity()),
    _wheel_enabled(feet_links.size(), false),
    _model_mtx(model_mtx)
    
{
    
    _model->getJointPosition(_q);
    _model->getJointLimits(_qmin, _qmax);
    _model->getVelocityLimits(_qdotmax);
    
    _dq = _q*0;
    
    std::vector<OpenSoT::SubTask::Ptr> feet_waist_pos_tasks, feet_world_orientation_tasks;
    
    for(int i = 0; i < _num_feet; i++){

        _feet_waist_cartesian_tasks.push_back( boost::make_shared<VelocityTask::Cartesian>("CARTESIAN_" + std::to_string(i),
                                                                           _q,
                                                                            *_model,
                                                                            _feet_links[i],
                                                                            "pelvis"
                                                                            )
                                       );
        
        _feet_waist_cartesian_tasks[i]->setLambda(0.05);
        
        _feet_world_cartesian_tasks.push_back( boost::make_shared<VelocityTask::Cartesian>("CARTESIAN_" + std::to_string(i),
                                                                           _q,
                                                                            *_model,
                                                                            _feet_links[i],
                                                                            "world"
                                                                            )
                                       );
        
        
        _feet_world_cartesian_tasks[i]->setLambda(0.05);
        
        std::list<uint> position_idx = {0, 1, 2}, orientation_idx = {3, 4};
        
        feet_waist_pos_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_waist_cartesian_tasks[i], position_idx) );
        feet_world_orientation_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_world_cartesian_tasks[i], orientation_idx) );

        _feet_waist_cartesian_tasks[i]->setWeight(Eigen::MatrixXd::Identity(6,6));

    }
    
    /* Upperbody cartesian */
    
    _left_arm_cartesian = boost::make_shared<VelocityTask::Cartesian>("CARTESIAN_LA",
                                                            _q,
                                                            *_model,
                                                            "arm1_7",
                                                            "pelvis"
                                                            );
    _left_arm_cartesian->setLambda(.05);
    
    _right_arm_cartesian = boost::make_shared<VelocityTask::Cartesian>("CARTESIAN_RA",
                                                            _q,
                                                            *_model,
                                                            "arm2_7",
                                                            "pelvis"
                                                            );
    _right_arm_cartesian->setLambda(.05);
    
    /* Joint limits */
    
    _joint_pos_lims = boost::make_shared<VelocityConstraint::JointLimits>(_q, _qmax, _qmin);
    
    _joint_vel_lims = boost::make_shared<VelocityConstraint::VelocityLimits>(_qdotmax, 0.01);
    
    /* Create autostack and solver */
    
    _autostack = (  ( feet_world_orientation_tasks[0] + feet_world_orientation_tasks[1] + feet_world_orientation_tasks[2] + feet_world_orientation_tasks[3] ) /
                    ( feet_waist_pos_tasks[0] + feet_waist_pos_tasks[1] + feet_waist_pos_tasks[2] + feet_waist_pos_tasks[3] ) /
                    ( _left_arm_cartesian + _right_arm_cartesian ) 
                 )  << _joint_pos_lims << _joint_vel_lims;
                 
    _autostack->update(_q);
    
    _autostack->getStack();
    
    _solver.reset( new OpenSoT::solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds(), 1e9) );
    
    /* Open action server */
    _action_server = std::make_shared<ActionServer>(_nh, 
                                                    "leg_movement_action", 
                                                    boost::bind(&centauro::LegMovementAction::execute_cb, this, _1), 
                                                    false);
    
    _action_server->start(); // Thread spawned!!
    
 
}


void centauro::LegMovementAction::execute_cb(const wholebody_keyboard_ctrl::LegMovementGoalConstPtr& goal)
{
    
    std::cout << __func__ << std::endl;
    
    ros::Rate loop_rate(_loop_rate);
    
    
    auto t0 = ros::Time::now();
    
    /* Update internal references */
    
    _goal_pos[0].translation() << goal->goal_pos_fl_x, goal->goal_pos_fl_y, goal->goal_pos_fl_z;
    _goal_pos[1].translation() << goal->goal_pos_fr_x, goal->goal_pos_fr_y, goal->goal_pos_fr_z;
    _goal_pos[2].translation() << goal->goal_pos_br_x, goal->goal_pos_br_y, goal->goal_pos_br_z;
    _goal_pos[3].translation() << goal->goal_pos_bl_x, goal->goal_pos_bl_y, goal->goal_pos_bl_z;
    
    _wheel_enabled[0] = goal->wheel_rotation_enabled_fl;
    _wheel_enabled[1] = goal->wheel_rotation_enabled_fr;
    _wheel_enabled[2] = goal->wheel_rotation_enabled_br;
    _wheel_enabled[3] = goal->wheel_rotation_enabled_bl;
    
    double TIMEOUT_TIME = 5.0;
    
    bool success = false;
    
    while( ((ros::Time::now() - t0).toSec() <= TIMEOUT_TIME) && !success ){
        
        /* Check for action preemption */
        if(!ros::ok() || _action_server->isPreemptRequested()){
            _action_server->setPreempted();
            break;
        }
    
        /* Update cartesians */
        for(int i = 0; i < _num_feet; i++){
            _feet_waist_cartesian_tasks[i]->setReference(_goal_pos[i].matrix());
        }
        
        {
            std::lock_guard<std::mutex>(*_model_mtx);
            _autostack->update(_q);
        }
        
        if(!_solver->solve(_dq)){
            _dq.setZero(_q.size());
            ROS_ERROR("UNABLE TO SOLVE IK");
        }
        
        _q += _dq;
        
        std::cout << _q.transpose().format(Eigen::IOFormat(1)) << std::endl;
        
        {
            std::lock_guard<std::mutex>(*_model_mtx);
            _model->setJointPosition(_q);
            _model->setJointVelocity(_dq/_loop_rate);
            _model->update();
        }
        
        success = check_tol(0.001);
        
        loop_rate.sleep();
    
    }
    
    if(success){
        wholebody_keyboard_ctrl::LegMovementResult result;
        _action_server->setSucceeded(result);
    }
    
}

bool centauro::LegMovementAction::check_tol(double tol)
{
    double err = 0;
    
    for(VelocityTask::Cartesian::Ptr task : _feet_waist_cartesian_tasks){
        err += task->getError().head<3>().norm();
    }
    
    err /= std::sqrt(3 * _num_feet);
    
    return err <= tol;
}



#endif