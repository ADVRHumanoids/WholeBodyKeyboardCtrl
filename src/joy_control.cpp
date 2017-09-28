#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_state_publisher/robot_state_publisher.h>


#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Contact.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <qpOASES/Options.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>

typedef OpenSoT::tasks::velocity::Contact ContactTask;
typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
typedef OpenSoT::tasks::velocity::CoM ComTask;
typedef OpenSoT::constraints::velocity::JointLimits JointLimits;
typedef OpenSoT::constraints::velocity::VelocityLimits JointVelocityLimits;
typedef OpenSoT::tasks::velocity::Postural PosturalTask;

Eigen::VectorXd twist_com_desired;
Eigen::VectorXd twist_pelvis_desired;

XBot::ModelInterface::Ptr _model;
ComTask::Ptr _com_task;

double v = 0.5;
double w = 0.5;
double dT = 0.01;

double joy_th = 0.5;

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy)
{
    twist_com_desired.setZero(3);


    if(fabs(joy->axes[2]+1.0) <= 1e-3)
    {
        if(joy->axes[1] > joy_th)
            twist_com_desired[2] = v;
        else if(joy->axes[1] < -joy_th)
            twist_com_desired[2] = -v;
    }
    else
    {
        if(joy->axes[1] > joy_th)
            twist_com_desired[0] = v;
        else if(joy->axes[1] < -joy_th)
            twist_com_desired[0] = -v;

        if(joy->axes[0] > joy_th)
            twist_com_desired[1] = v;
        else if(joy->axes[0] < -joy_th)
            twist_com_desired[1] = -v;
    }

    twist_pelvis_desired.setZero(6);
    if(fabs(joy->axes[5]+1.0) <= 1e-3)
    {
        if(joy->axes[3] > joy_th)
            twist_pelvis_desired[3] = w;
        else if(joy->axes[3] < -joy_th)
            twist_pelvis_desired[3] = -w;
    }
    else
    {
        if(joy->axes[4] > joy_th)
            twist_pelvis_desired[4] = w;
        else if(joy->axes[4] < -joy_th)
            twist_pelvis_desired[4] = -w;

        if(joy->axes[3] > joy_th)
            twist_pelvis_desired[5] = w;
        else if(joy->axes[3] < -joy_th)
            twist_pelvis_desired[5] = -w;
    }



}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh("~");

    twist_com_desired.setZero(3);
    twist_pelvis_desired.setZero(6);

    ros::Subscriber joystick_feedback = nh.subscribe("/joy", 1000, joy_cb);

    _model = XBot::ModelInterface::getModel(argv[1]);

    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(_model->getUrdf(), kdl_tree);

    robot_state_publisher::RobotStatePublisher rspub(kdl_tree);

    std::string _urdf_param_name = "/xbotcore/" + _model->getUrdf().getName() + "/robot_description";
    std::string _tf_prefix = "/xbotcore/" + _model->getUrdf().getName();
    nh.setParam(_urdf_param_name, _model->getUrdfString());

    int _num_feet = _model->legs();
    std::vector<std::string> _feet_links = {"foot_fl", "foot_fr", "foot_hr", "foot_hl"};

    Eigen::MatrixXd _contact_matrix;
    _contact_matrix.setZero(5,6);
    _contact_matrix << 1, 0, 0, 0, 0, 0,
                       0, 1, 0, 0, 0, 0,
                       0, 0, 1, 0, 0, 0,
                       0, 0, 0, 1, 0, 0,
                       0, 0, 0, 0, 0, 1;

    std::vector<Eigen::MatrixXd> cm_vector(4, _contact_matrix);

    Eigen::VectorXd _qhome;
    _model->getRobotState("home", _qhome);
    _model->setJointPosition(_qhome);
    _model->update();

    std::vector<ContactTask::Ptr> _contact_tasks;
    std::vector<CartesianTask::Ptr> _feet_cartesian_tasks;
    CartesianTask::Ptr _pelvis_cartesian_task, _left_arm_cartesian, _right_arm_cartesian;
    PosturalTask::Ptr _postural_task;

    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_pos_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_vel_lims;

    OpenSoT::AutoStack::Ptr _autostack;
    OpenSoT::solvers::QPOases_sot::Ptr _solver;

    Eigen::VectorXd _q, _qmin, _qmax, _qdotmax, _dq;

    _model->getJointPosition(_q);
    _model->getJointLimits(_qmin, _qmax);
    _model->getVelocityLimits(_qdotmax);


    std::vector<OpenSoT::SubTask::Ptr> feet_pos_tasks, feet_orientation_tasks;

    for(int i = 0; i < _num_feet; i++){

        _contact_tasks.push_back( boost::make_shared<ContactTask>("CONTACT_" + std::to_string(i),
                                                                  *_model,
                                                                  _feet_links[i],
                                                                  cm_vector[i]
                                                                  ) );


        _feet_cartesian_tasks.push_back( boost::make_shared<CartesianTask>("CARTESIAN_" + std::to_string(i),
                                                                           _q,
                                                                            *_model,
                                                                            _feet_links[i],
                                                                            "world"
                                                                            )
                                       );

        _feet_cartesian_tasks[i]->setLambda(0.1);

        std::list<uint> position_idx = {0, 1, 2}, orientation_idx = {3, 4, 5};

        feet_pos_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_cartesian_tasks[i], position_idx) );
        feet_orientation_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_cartesian_tasks[i], orientation_idx) );

        _feet_cartesian_tasks[i]->setWeight(Eigen::MatrixXd::Identity(6,6));
        _feet_cartesian_tasks[i]->setActive(true);

    }

    _postural_task = boost::make_shared<PosturalTask>(_qhome);
    _postural_task->setLambda(.01);

    _com_task = boost::make_shared<ComTask>(_q, *_model);
    //_com_task->setLambda(.02);
    _com_task->setLambda(0.0);

    _pelvis_cartesian_task = boost::make_shared<CartesianTask>("CARTESIAN_PELVIS",
                                                                _q,
                                                                *_model,
                                                                "pelvis",
                                                                "world"
                                                                );
    //_pelvis_cartesian_task->setLambda(.01);
    _pelvis_cartesian_task->setLambda(0.0);

    _left_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_LA",
                                                            _q,
                                                            *_model,
                                                            "arm1_7",
                                                            "pelvis"
                                                            );
    _left_arm_cartesian->setLambda(.01);

    _right_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_RA",
                                                            _q,
                                                            *_model,
                                                            "arm2_7",
                                                            "pelvis"
                                                            );
    _right_arm_cartesian->setLambda(.01);

    std::list<uint> orientation_idx = {3,4,5};
    auto pelvis_orientation_task = boost::make_shared<OpenSoT::SubTask>(_pelvis_cartesian_task, orientation_idx);

    _joint_pos_lims = boost::make_shared<JointLimits>(_q, _qmax, _qmin);

    _joint_vel_lims = boost::make_shared<JointVelocityLimits>(_qdotmax, 0.01);



    _autostack = (  ( _contact_tasks[0] + _contact_tasks[1] + _contact_tasks[2] + _contact_tasks[3] ) /
                    ( _left_arm_cartesian + _right_arm_cartesian ) /
                    ( _com_task + pelvis_orientation_task ) /
                    ( feet_pos_tasks[0] + feet_pos_tasks[1] + feet_pos_tasks[2] + feet_pos_tasks[3] ) /
                    ( feet_orientation_tasks[0] + feet_orientation_tasks[1] + feet_orientation_tasks[2] + feet_orientation_tasks[3] ) /
                    _postural_task
                 )  << _joint_pos_lims << _joint_vel_lims;

    _autostack->update(_q);

    _autostack->getStack();

    _solver.reset( new OpenSoT::solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds(), 1e10) );

    _solver->setActiveStack(3, false);
    _solver->setActiveStack(4, false);

    /* TF broadcaster for publishing floating base tf */
    tf::TransformBroadcaster tf_broadcaster;

    XBot::JointNameMap _joint_name_map;

    ROS_INFO("Running joy_control_node");
    while(ros::ok())
    {

        _com_task->setReference(Eigen::VectorXd::Zero(3), twist_com_desired*dT);
        _pelvis_cartesian_task->setReference(Eigen::MatrixXd::Zero(4,4), twist_pelvis_desired*dT);

        /* Update stack */
        _autostack->update(_q);

        /* Solve QP */
        if(!_solver->solve(_dq)){
            std::cout << "SOLVER ERROR" << std::endl;
            continue;
        }

        /* Integrate model */
        _q += _dq;

       _model->setJointPosition(_q);
       _model->update();

       /* Publish TF */
       _model->getJointPosition(_joint_name_map);
        std::map<std::string, double> _joint_name_std_map(_joint_name_map.begin(), _joint_name_map.end());

        rspub.publishTransforms(_joint_name_std_map, ros::Time::now(), "");
        rspub.publishFixedTransforms("");

       /* Publish floating base pose to TF */
        Eigen::Affine3d w_T_pelvis;
        _model->getFloatingBasePose(w_T_pelvis);
        tf::Transform transform;
        tf::transformEigenToTF(w_T_pelvis, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "pelvis", "world_odom"));

        ros::spinOnce();

        ros::Duration(dT).sleep();
    }
}
