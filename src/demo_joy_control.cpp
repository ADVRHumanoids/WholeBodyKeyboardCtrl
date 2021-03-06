#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Contact.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <qpOASES/Options.hpp>
#include <OpenSoT/SubTask.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>

#include <functional>

typedef OpenSoT::tasks::velocity::Contact ContactTask;
typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
typedef OpenSoT::tasks::velocity::CoM ComTask;
typedef OpenSoT::constraints::velocity::JointLimits JointLimits;
typedef OpenSoT::constraints::velocity::VelocityLimits JointVelocityLimits;
typedef OpenSoT::tasks::velocity::Postural PosturalTask;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using XBot::Logger;

const std::vector<std::string> ee_name_list = {"COM", "LEFT_ARM", "RIGHT_ARM", "LEFT_FRONT_FOOT", "RIGHT_FRONT_FOOT", "RIGHT_HIND_FOOT", "LEFT_HIND_FOOT"};

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy, 
                  Eigen::Vector6d& ref_twist, 
                  int& ee_id,
                  bool& base_frame_is_world
                  );

bool is_foot(const std::string& ee_name);


int main(int argc, char** argv){
    
    /* Command line parsing */
    
    std::string path_to_cfg;
    bool visual_mode = false;
    
    {
        po::options_description desc("Whole body joystick constrol. Available options:");
        desc.add_options()
            ("visual,V","Visual mode does not send references to the robot. \
                         An internal robot state publisher provides TF for visualization in rviz")
            ("config,C", po::value<std::string>(), "Path to config file")
        ;

        
        po::positional_options_description p;
        p.add("config", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(desc).positional(p).run(), vm);
        po::notify(vm);

        if (vm.count("config")) {
            path_to_cfg = fs::absolute(vm["config"].as<std::string>()).string();
        }
        else{
            path_to_cfg = XBot::Utils::getXBotConfig();
            if(path_to_cfg == ""){
                std::cout << desc << std::endl;
                return 1;
            }
        }
        
        if (vm.count("visual")) {
            visual_mode = true;
        }
    }
    
    ros::init(argc, argv, "demo_joy_control");
    
    
    ros::NodeHandle nh;
    
    Eigen::Vector6d ref_twist;
    ref_twist.setZero();
    int ee_id = 0;
    bool base_frame_is_world = true;

    auto joy_sub_callback = std::bind(joy_callback, std::placeholders::_1, 
                                             std::ref(ref_twist),
                                             std::ref(ee_id),
                                             std::ref(base_frame_is_world)
                                            );
    ros::Subscriber joystick_feedback = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_sub_callback);
    
    auto _model = XBot::ModelInterface::getModel(path_to_cfg);
    auto _robot = XBot::RobotInterface::getRobot(path_to_cfg);
    
    _robot->sense();
    
    
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
    ComTask::Ptr _com_task;
    PosturalTask::Ptr _postural_task;

    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_pos_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_vel_lims;

    OpenSoT::AutoStack::Ptr _autostack;
    OpenSoT::solvers::iHQP::Ptr _solver;

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
        
        _feet_cartesian_tasks[i]->setLambda(0.0);
        
        std::list<uint> position_idx = {0, 1, 2}, orientation_idx = {3, 4, 5};
        
        feet_pos_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_cartesian_tasks[i], position_idx) );
        feet_orientation_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(_feet_cartesian_tasks[i], orientation_idx) );

        _feet_cartesian_tasks[i]->setWeight(Eigen::MatrixXd::Identity(6,6));
        _feet_cartesian_tasks[i]->setActive(true);

    }
    
    _postural_task = boost::make_shared<PosturalTask>(_qhome);
    _postural_task->setLambda(.01);

    _com_task = boost::make_shared<ComTask>(_q, *_model);
    _com_task->setLambda(.00);

    _pelvis_cartesian_task = boost::make_shared<CartesianTask>("CARTESIAN_PELVIS",
                                                                _q,
                                                                *_model,
                                                                "pelvis",
                                                                "world"
                                                                );
    _pelvis_cartesian_task->setLambda(.00);
    
    _left_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_LA",
                                                            _q,
                                                            *_model,
                                                            "arm1_7",
                                                            "pelvis"
                                                            );
    _left_arm_cartesian->setLambda(.00);
    
    _right_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_RA",
                                                            _q,
                                                            *_model,
                                                            "arm2_7",
                                                            "pelvis"
                                                            );
    _right_arm_cartesian->setLambda(.00);

    std::list<uint> orientation_idx = {3,4,5};
    auto pelvis_orientation_task = boost::make_shared<OpenSoT::SubTask>(_pelvis_cartesian_task, orientation_idx);
    
    _joint_pos_lims = boost::make_shared<JointLimits>(_q, _qmax, _qmin);
    
    _joint_vel_lims = boost::make_shared<JointVelocityLimits>(_qdotmax, 0.01);
    
    std::map<std::string, CartesianTask::Ptr> _tasks_map;
    std::map<std::string, ContactTask::Ptr> _contact_map;
    
    _tasks_map["RIGHT_ARM"] = _right_arm_cartesian;
    _tasks_map["LEFT_ARM"] = _left_arm_cartesian;
    _tasks_map["LEFT_FRONT_FOOT"] = _feet_cartesian_tasks[0];
    _tasks_map["RIGHT_FRONT_FOOT"] = _feet_cartesian_tasks[1];
    _tasks_map["RIGHT_HIND_FOOT"] = _feet_cartesian_tasks[2];
    _tasks_map["LEFT_HIND_FOOT"] = _feet_cartesian_tasks[3];
    
    _contact_map["LEFT_FRONT_FOOT"] = _contact_tasks[0];
    _contact_map["RIGHT_FRONT_FOOT"] = _contact_tasks[1];
    _contact_map["RIGHT_HIND_FOOT"] = _contact_tasks[2];
    _contact_map["LEFT_HIND_FOOT"] = _contact_tasks[3];

    
    
    _autostack = (  ( _contact_tasks[0] + _contact_tasks[1] + _contact_tasks[2] + _contact_tasks[3] ) /
                    ( _left_arm_cartesian + _right_arm_cartesian ) /
                    ( _com_task + pelvis_orientation_task ) /
                    ( feet_pos_tasks[0] + feet_pos_tasks[1] + feet_pos_tasks[2] + feet_pos_tasks[3] ) /
                    ( feet_orientation_tasks[0] + feet_orientation_tasks[1] + feet_orientation_tasks[2] + feet_orientation_tasks[3] ) /
                    _postural_task
                 )  << _joint_pos_lims << _joint_vel_lims;
                 
    _autostack->update(_q);
    
    _autostack->getStack();

    _solver.reset( new OpenSoT::solvers::iHQP(_autostack->getStack(), _autostack->getBounds(), 1e9) );
    
    int contact_stack_id = 0;
    int foot_pos_stack_id = 3;
    int foot_or_stack_id = 4;

    _solver->setActiveStack(foot_pos_stack_id, false);
    _solver->setActiveStack(foot_or_stack_id,  false);
    
    
    /* TF broadcaster for publishing floating base tf */
    tf::TransformBroadcaster tf_broadcaster;
   
    
    ros::Rate loop_rate(100);
    double dt = loop_rate.expectedCycleTime().toSec();
    
    XBot::JointNameMap _joint_name_map;
    

    Logger::success(Logger::Severity::HIGH, "Started looping.. \n");
    
    while(ros::ok()){
        
        /* Update reference from joypad */
        ros::spinOnce();
        
        /* Set references from joypad */

        if(ee_name_list[ee_id] == "COM"){
            
            _solver->setActiveStack(foot_pos_stack_id, false);
            _solver->setActiveStack(foot_or_stack_id, false);
            for(auto c : _contact_tasks) c->setActive(true);
            
            Eigen::Affine3d T;
            _model->getPose(_pelvis_cartesian_task->getDistalLink(), T);
            Eigen::MatrixXd I; I.setZero(6,6);
            I.block(0,0,3,3) = T.linear().transpose();
            I.block(3,3,3,3) = T.linear().transpose();
            
            Eigen::Vector6d local_twist = ref_twist;
            
            if(!base_frame_is_world){
                local_twist = I*local_twist;
            }
            
            _com_task->setReference(Eigen::Vector3d::Zero(), local_twist.head<3>()*dt);
            _pelvis_cartesian_task->setReference(T.matrix(), local_twist*dt);
        }
        else{
            
            if(is_foot(ee_name_list[ee_id])){
                _solver->setActiveStack(foot_pos_stack_id, true);
                _solver->setActiveStack(foot_or_stack_id, true);
                for(auto c : _contact_tasks) c->setActive(true);
                _contact_map.at(ee_name_list[ee_id])->setActive(false);
            }
            else{
                _solver->setActiveStack(foot_pos_stack_id, false);
                _solver->setActiveStack(foot_or_stack_id, false);
                for(auto c : _contact_tasks) c->setActive(true);
            }
            
            CartesianTask::Ptr current_task = _tasks_map.at(ee_name_list[ee_id]);
            
            Eigen::Affine3d T;
            _model->getPose(current_task->getDistalLink(), current_task->getBaseLink(), T);
            Eigen::MatrixXd I; I.setZero(6,6);
            I.block(0,0,3,3) = T.linear();
            I.block(3,3,3,3) = T.linear();
            
            Eigen::Vector6d local_twist = ref_twist*3;
            
            if(!base_frame_is_world){
                local_twist = I*local_twist;
            }
            
            current_task->setReference(T.matrix(), local_twist*dt);
            
        }
        
        
        /* Update stack */
        _autostack->update(_q);
        
        /* Solve QP */
        if(!_solver->solve(_dq)){
            Logger::error("SOLVER ERROR \n");
            continue;
        }
        
        /* Integrate model */
        _q += _dq;
        
       _model->setJointPosition(_q);
       _model->update();
       

       /* Publish TF */
       if(visual_mode){
            _model->getJointPosition(_joint_name_map);
            std::map<std::string, double> _joint_name_std_map(_joint_name_map.begin(), _joint_name_map.end());

            rspub.publishTransforms(_joint_name_std_map, ros::Time::now(), "");
            rspub.publishFixedTransforms("");
       }
       else {
            /* Send reference to robot */
            _robot->setReferenceFrom(*_model, XBot::Sync::Position);
            _robot->move();
       }
        
       
       /* Publish floating base pose to TF */
        Eigen::Affine3d w_T_pelvis;
        _model->getFloatingBasePose(w_T_pelvis);
        tf::Transform transform;
        tf::transformEigenToTF(w_T_pelvis, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "pelvis", "world_odom"));
        
       /* Sleep a bit */
       loop_rate.sleep();
       
    }
    
    
        
    
}

bool is_foot(const string& ee_name)
{
    if(ee_name == "LEFT_FRONT_FOOT"){
        return true;
    }
    
    if(ee_name == "RIGHT_FRONT_FOOT"){
        return true;
    }
    
    if(ee_name == "LEFT_HIND_FOOT"){
        return true;
    }
    
    if(ee_name == "RIGHT_HIND_FOOT"){
        return true;
    }
    
    return false;
}


void joy_callback(const sensor_msgs::Joy::ConstPtr& joy, 
                  Eigen::Vector6d& ref_twist, 
                  int& ee_id,
                  bool& base_frame_is_world
                  )
{
    
    double v_max = 0.02;
    double thetadot_max = 0.1;
    
    int fwd_bck = 7;
    int l_r = 6;
    int up_down = 3;
    int yaw = 2;
    int roll = 0;
    int pitch = 1;
    
    int ee_selector_plus = 7;
    
    int base_link_selector = 5;

    ref_twist[0] = v_max * joy->axes[fwd_bck];
    ref_twist[1] = v_max * joy->axes[l_r];
    ref_twist[2] = v_max * joy->axes[up_down];
    
    ref_twist[3] = -1.0 * thetadot_max * joy->axes[roll];
    ref_twist[4] = thetadot_max * joy->axes[pitch];
    ref_twist[5] = thetadot_max * joy->axes[yaw];
    
    int old_ee_id = ee_id;
    
    ee_id = (ee_id + joy->buttons[ee_selector_plus]) % ee_name_list.size();
    
    if(ee_id != old_ee_id){
        Logger::info(Logger::Severity::HIGH) << "Controlling " << ee_name_list[ee_id] << Logger::endl();
    }
    
    if(joy->buttons[base_link_selector] == 1){
        base_frame_is_world = !base_frame_is_world;
        
        if(base_frame_is_world){
            Logger::info(Logger::Severity::HIGH) << "Controlling w.r.t. world frame" << Logger::endl();
        }
        else{
            Logger::info(Logger::Severity::HIGH) << "Controlling w.r.t. local frame" << Logger::endl();
        }
        
    }
    
    
}
