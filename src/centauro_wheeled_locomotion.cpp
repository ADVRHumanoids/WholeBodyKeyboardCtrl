#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <OpenSoT/tasks/velocity/RigidRotation.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <boost/make_shared.hpp>
#include <qpOASES/Options.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/TaskToConstraint.h>


#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <functional>


void callback(geometry_msgs::Twist::ConstPtr msg, Eigen::Vector6d& vref){
 
    vref(0) = msg->linear.x;
    vref(1) = msg->linear.y;
    vref(5) = msg->angular.z;
    
}

void joy_callback(sensor_msgs::Joy::ConstPtr msg, Eigen::Vector6d& vref){
    
    double v_max = 0.1;
    double thetadot_max = 0.1;
    
    int fwd_bck = 7;
    int l_r = 6;
    int up_down = 3;
    int yaw = 2;

    vref[0] = v_max * msg->axes[fwd_bck];
    vref[1] = v_max * msg->axes[l_r];
    vref[2] = v_max * msg->axes[up_down];
    
    vref[5] = thetadot_max * msg->axes[yaw];
    
}



int main(int argc, char** argv){
 
    ros::init(argc, argv, "centauro_wheeled_locomotion");
    ros::NodeHandle nh;
    
    Eigen::Vector6d vref;
    vref.setZero();
    
    auto vref_cbk = std::bind(callback, std::placeholders::_1, std::ref(vref));
    ros::Subscriber vref_sub = nh.subscribe<geometry_msgs::Twist>("/centauro_wheeled_locomotion/vref", 1, vref_cbk);
    
    auto vref_joy_cbk = std::bind(joy_callback, std::placeholders::_1, std::ref(vref));
    ros::Subscriber joy_vref_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, vref_joy_cbk);
    
    auto logger = XBot::MatLogger::getLogger("/tmp/centauro_wheeled_locomotion_log");
    
    double ts = 0.01;
    
    auto robot = XBot::RobotInterface::getRobot(argv[1]);
    auto model = XBot::ModelInterface::getModel(argv[1]);
    
    std::vector<std::string> wheel_names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};

    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    Eigen::VectorXd qmin, qmax, qdotmax;
    model->getJointLimits(qmin, qmax);
    model->getVelocityLimits(qdotmax);
    
    std::cout << "******\n" << qmax - qmin << "******\n" << std::endl;
    std::cout << "******\n" << qdotmax << "******\n" << std::endl;
    
    
    typedef OpenSoT::tasks::velocity::PureRolling PureRollingTask;
    typedef OpenSoT::tasks::velocity::RigidRotation RigidRotationTask;
    typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
    typedef OpenSoT::tasks::velocity::CoM ComTask;
    typedef OpenSoT::constraints::velocity::JointLimits JointLimits;
    typedef OpenSoT::constraints::velocity::VelocityLimits JointVelocityLimits;
    typedef OpenSoT::tasks::velocity::Postural PosturalTask;


    std::vector<PureRollingTask::Ptr> rolling_tasks;
    std::vector<RigidRotationTask::Ptr> icr_tasks;
    std::vector<CartesianTask::Ptr> feet_cartesian_tasks;
    std::vector<OpenSoT::SubTask::Ptr> feet_cartesian_pos_tasks;
    std::vector<OpenSoT::SubTask::Ptr> rolling_pos_tasks;
    std::vector<OpenSoT::SubTask::Ptr> rolling_or_tasks;
    CartesianTask::Ptr pelvis_cartesian_task;
    ComTask::Ptr com_task;
    PosturalTask::Ptr postural_task;

    OpenSoT::constraints::velocity::JointLimits::Ptr joint_pos_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_vel_lims;

    OpenSoT::AutoStack::Ptr autostack;
    OpenSoT::solvers::QPOases_sot::Ptr solver;
    
    
    
    for(int i = 0; i < 4; i++){

        rolling_tasks.push_back( boost::make_shared<PureRollingTask>(wheel_names[i],
                                                                     0.075,
                                                                     *model
                                                                     )
                                 );
        
        rolling_tasks[i]->setWeight(Eigen::Vector4d(100,100,100,100).asDiagonal());
        
        icr_tasks.push_back( boost::make_shared<RigidRotationTask>(wheel_names[i],
                                                                   "pelvis",
                                                                     *model,
                                                                   0.01
                                                                     )
                                 );
        
        icr_tasks[i]->setLambda(.05);
        
        feet_cartesian_tasks.push_back( boost::make_shared<CartesianTask>("CARTESIAN_" + std::to_string(i),
                                                                           qhome,
                                                                           *model,
                                                                           wheel_names[i],
                                                                           "pelvis"
                                                                           )
                                       );
        
        feet_cartesian_tasks[i]->setLambda(0.1);
        
        std::list<uint> position_dofs = {0,1};
        feet_cartesian_pos_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(feet_cartesian_tasks[i], position_dofs) );
        
        rolling_pos_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(rolling_tasks[i], position_dofs) );
        
        std::list<uint> rolling_or_dofs = {2, 3};
        rolling_or_tasks.push_back( boost::make_shared<OpenSoT::SubTask>(rolling_tasks[i], rolling_or_dofs) );


    }
    
    postural_task = boost::make_shared<PosturalTask>(qhome);
    postural_task->setLambda(0.1);

    com_task = boost::make_shared<ComTask>(qhome, *model);
    com_task->setLambda(0.1);

    pelvis_cartesian_task = boost::make_shared<CartesianTask>("CARTESIAN_PELVIS",
                                                                           qhome,
                                                                            *model,
                                                                            "pelvis",
                                                                            "world"
                                                                            );
    pelvis_cartesian_task->setLambda(0.0);
    
    auto left_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_LA",
                                                                           qhome,
                                                                            *model,
                                                                            "arm1_7",
                                                                            "pelvis"
                                                                            );
    
    auto right_arm_cartesian = boost::make_shared<CartesianTask>("CARTESIAN_RA",
                                                                           qhome,
                                                                            *model,
                                                                            "arm2_7",
                                                                            "pelvis"
                                                                            );

    std::list<uint> height_orientation_dofs = {2,3,4,5};
    auto pelvis_h_orientation_task = boost::make_shared<OpenSoT::SubTask>(pelvis_cartesian_task, height_orientation_dofs);
    
    
    
    joint_pos_lims = boost::make_shared<JointLimits>(qhome, qmax, qmin);
    
    joint_vel_lims = boost::make_shared<JointVelocityLimits>(qdotmax/5, ts*2);

    autostack = (   
                    ( rolling_or_tasks[0] +
                        rolling_or_tasks[1] + 
                        rolling_or_tasks[2] +
                        rolling_or_tasks[3]
                    ) 
                    / ( icr_tasks[0] + 
                        icr_tasks[1] + 
                        icr_tasks[2] + 
                        icr_tasks[3] 
                        
                    ) 
                    / ( pelvis_cartesian_task + left_arm_cartesian + right_arm_cartesian) 
                    / ( rolling_pos_tasks[0] + 
                        rolling_pos_tasks[1] + 
                        rolling_pos_tasks[2] + 
                        rolling_pos_tasks[3] ) 
                    
//                     / postural_task
                    
                 )  << joint_pos_lims << joint_vel_lims;
//                     << boost::make_shared<OpenSoT::constraints::TaskToConstraint>(rolling_pos_tasks[0] + 
//                                                                                     rolling_pos_tasks[1] + 
//                                                                                     rolling_pos_tasks[2] + 
//                                                                                     rolling_pos_tasks[3], 
//                                                                                     -Eigen::VectorXd::Ones(12)*0.00001,
//                                                                                     Eigen::VectorXd::Ones(12)*0.00001
//                                                                                     );
                 
    autostack->update(qhome);
    autostack->log(logger);

    try{
        solver.reset( new OpenSoT::solvers::QPOases_sot(autostack->getStack(), autostack->getBounds(), 1) );
    }
    catch(...){
       return -1;
    }

//     for(int i = 0; i < solver->getNumberOfTasks(); i++){
//         qpOASES::Options opt;
//         opt.setToMPC();
//         opt.printLevel = qpOASES::PL_LOW;
// //         opt.epsRegularisation *= 1e5;
//         solver->setOptions(i, opt);
//     }
    
    ros::Rate loop_rate(1.0/ts);
    
    robot->sense();
    
    std::cout << *robot << std::endl;
    
    robot->setReferenceFrom(*model, XBot::Sync::Position);
    
    for(int i = 0; i < 100 ; i++){
        robot->move();
        ros::Duration(0.01).sleep();
    }
    
    
    /* TF broadcaster for publishing floating base tf */
    tf::TransformBroadcaster tf_broadcaster;
    
//     robot->sense();
//     model->syncFrom(*robot);
    
    Eigen::VectorXd q, dq;
    model->getJointPosition(q);
    
    model->initLog(logger, 10000);
    
    dq = q*0;
    
    Eigen::Affine3d waist_pose_ref;
    model->getPose("pelvis", waist_pose_ref);
    
    double time = 0.0;
    double theta = 0.0;
    
    while(ros::ok()){
        
        ros::spinOnce();
        Eigen::Vector3d cart_vref;
        cart_vref << vref(0), vref(1), vref(5);
        
        for(int i : {0, 1, 2, 3}){
            icr_tasks[i]->setReference(0, cart_vref, Eigen::Vector2d(0.40, 0.27));
        }
        
        pelvis_cartesian_task->setReference(waist_pose_ref.matrix(), vref*ts);
        
        autostack->update(q);
        
        if(!solver->solve(dq)){
            std::cerr << "Porcozio" << std::endl;
            autostack->log(logger);
            dq *= 0;
        }
        
        q += dq;
        
        model->setJointPosition(q);
        model->setJointVelocity(dq/ts);
        model->update();
        model->log(logger, ros::Time::now().toSec());
        autostack->log(logger);
        
        robot->setReferenceFrom(*model, XBot::Sync::Position);
        robot->move();
     
        std::cout << loop_rate.cycleTime().toSec() << std::endl;
        
        /* Publish floating base pose to TF */
        Eigen::Affine3d w_T_pelvis;
        model->getFloatingBasePose(w_T_pelvis);
        tf::Transform transform;
        tf::transformEigenToTF(w_T_pelvis, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "pelvis", "world_odom"));
       
        
        time += loop_rate.expectedCycleTime().toSec();
        loop_rate.sleep();
        
    }
    
    
}