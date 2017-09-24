#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <boost/make_shared.hpp>

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

#include <mutex>
#include <functional>

typedef OpenSoT::tasks::velocity::Contact ContactTask;
typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
typedef OpenSoT::tasks::velocity::CoM ComTask;
typedef OpenSoT::constraints::velocity::JointLimits JointLimits;
typedef OpenSoT::constraints::velocity::VelocityLimits JointVelocityLimits;
typedef OpenSoT::tasks::velocity::Postural PosturalTask;

std::mutex com_ref_mutex;
std::mutex R_waist_ref_mutex;



void keyboard_callback(const ros::TimerEvent& ev, 
                       Eigen::Vector3d& com_ref, 
                       Eigen::Matrix3d& R_waist_ref);


int main(int argc, char** argv){
    
    if(argc != 2){
        std::cout << "Usage: " << argv[0] << " path_to_config_file" << std::endl;
        return 1;
    }
    
    ros::init(argc, argv, "keyboard_control");
    
    
    ros::NodeHandle nh;
    ros::CallbackQueue callback_queue;
    nh.setCallbackQueue(&callback_queue);
    
    Eigen::Vector3d com_ref;
    Eigen::Matrix3d R_waist_ref;

    auto keyboard_timer_callback = std::bind(keyboard_callback, std::placeholders::_1, 
                                             std::ref(com_ref),
                                             std::ref(R_waist_ref));
    ros::Timer keyboard_timer = nh.createTimer(ros::Duration(.1), keyboard_timer_callback);
    
    auto _robot = XBot::RobotInterface::getRobot(argv[1]);
    auto _model = XBot::ModelInterface::getModel(argv[1]);
    
    _robot->sense();
    
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(_robot->getUrdf(), kdl_tree);

    robot_state_publisher::RobotStatePublisher rspub(kdl_tree);

    std::string _urdf_param_name = "/xbotcore/" + _robot->getUrdf().getName() + "/robot_description";
    std::string _tf_prefix = "/xbotcore/" + _robot->getUrdf().getName();
    nh.setParam(_urdf_param_name, _robot->getUrdfString());
    
    int _num_feet = _robot->legs();
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
    
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();
    
    ros::Duration(1.0).sleep();
 
    
    
    std::vector<ContactTask::Ptr> _contact_tasks;
    std::vector<CartesianTask::Ptr> _feet_cartesian_tasks;
    CartesianTask::Ptr _pelvis_cartesian_task, _left_arm_cartesian, _right_arm_cartesian;
    ComTask::Ptr _com_task;
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
    _com_task->setLambda(.02);

    _pelvis_cartesian_task = boost::make_shared<CartesianTask>("CARTESIAN_PELVIS",
                                                                _q,
                                                                *_model,
                                                                "pelvis",
                                                                "world"
                                                                );
    _pelvis_cartesian_task->setLambda(.01);
    
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
                    ( _com_task + _left_arm_cartesian + _right_arm_cartesian ) /
                    ( pelvis_orientation_task ) /
                    ( feet_pos_tasks[0] + feet_pos_tasks[1] + feet_pos_tasks[2] + feet_pos_tasks[3] ) /
                    ( feet_orientation_tasks[0] + feet_orientation_tasks[1] + feet_orientation_tasks[2] + feet_orientation_tasks[3] ) /
                    _postural_task
                 )  << _joint_pos_lims << _joint_vel_lims;
                 
    _autostack->update(_q);
    
    _autostack->getStack();

    _solver.reset( new OpenSoT::solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds(), 1e9) );

    _solver->setActiveStack(3, false);
    _solver->setActiveStack(4, false);
    
    com_ref = _com_task->getReference();
    R_waist_ref = _pelvis_cartesian_task->getReference().block(0, 0, 3, 3);
    
    /* TF broadcaster for publishing floating base tf */
    tf::TransformBroadcaster tf_broadcaster;
    
    /* Spawn a separate thread for keyboard input */
    ros::AsyncSpinner async_spinner(1, &callback_queue);
    async_spinner.start();
    
    std::cout << "Usage: \n";
    std::cout << "\t W-A-S-D: com position on the XY plane \n";
    std::cout << "\t T-G: com height (up-down) \n";
    std::cout << "\t R-F: waist roll (pos-neg) \n";
    std::cout << "\t P-ò: waist pitch (pos-neg) \n";
    std::cout << "\t Y-H: waist yaw (pos-neg) \n";
    
    ros::Rate loop_rate(100);
    
    XBot::JointNameMap _joint_name_map;

    while(ros::ok()){
        
        /* Receive new reference for COM and waist orientation */
        {
            std::lock_guard<std::mutex> lkg(com_ref_mutex);
            _com_task->setReference(com_ref);
        }
        
        {
            std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
            Eigen::Matrix4d T;
            T.block(0,0,3,3) = R_waist_ref;
            _pelvis_cartesian_task->setReference(T);
        }
        
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
       
       /* Send config to robot */
       _robot->setReferenceFrom(*_model, XBot::Sync::Position);
       _robot->move();
       
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
        
       /* Sleep a bit */
       loop_rate.sleep();
       
    }
    
    
        
    
}


void keyboard_callback(const ros::TimerEvent& ev, Eigen::Vector3d& com_ref, Eigen::Matrix3d& R_waist_ref)
{
 
    std::string input;
    getline(std::cin, input);
    
    
    if(input == "w"){ // Forward
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.x() += 0.01;
    }
    else if(input == "a"){ // Left
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.y() += 0.01;
    }
    else if(input == "s"){ // Backward
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.x() -= 0.01;
    }
    else if(input == "d"){ // Right
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.y() -= 0.01;
    }
    else if(input == "t"){ // Up
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.z() += 0.01;
    }
    else if(input == "g"){ // Down
        std::lock_guard<std::mutex> lkg(com_ref_mutex);
        com_ref.z() -= 0.01;
    }
    else if(input == "r"){ // Roll positive
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX());
    }
    else if(input == "f"){ // Roll negative
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitX());
    }
    else if(input == "p"){ // Pitch positive
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY());
    }
    else if(input == "ò"){ // Pitch negative
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitY());
    }
    else if(input == "y"){ // Yaw positive
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ());
    }
    else if(input == "h"){ // Yaw negative
        std::lock_guard<std::mutex> lkg(R_waist_ref_mutex);
        R_waist_ref = R_waist_ref * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitZ());
    }
    else if(input == "QUIT"){
        ros::shutdown();
    }
    else{
        std::cout << "Char sequence not supported!" << std::endl;
        std::cout << "Usage: \n";
        std::cout << "\t W-A-S-D: com position on the XY plane \n";
        std::cout << "\t T-G: com height (up-down) \n";
        std::cout << "\t R-F: waist roll (pos-neg) \n";
        std::cout << "\t P-ò: waist pitch (pos-neg) \n";
        std::cout << "\t Y-H: waist yaw (pos-neg) \n";
    }
    
}
