#include <WholeBodyKeyboardCtrl/LegMovementAction.h>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <robot_state_publisher/robot_state_publisher.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int main(int argc, char **argv){
    
    ros::init(argc, argv, "wholebody_action_node");
    ros::NodeHandle nh;
    
    /* Command line parsing */
    
    std::string path_to_cfg;
    bool visual_mode = false;
    
    {
        po::options_description desc("Whole body leg movement action server. Available options:");
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
            std::cout << desc << std::endl;
        }
        
        if (vm.count("visual")) {
            visual_mode = true;
        }
    }
    
    
    auto robot = XBot::RobotInterface::getRobot(path_to_cfg);
    auto model = XBot::ModelInterface::getModel(path_to_cfg);
    
    if(!visual_mode){
        robot->sense();
        model->syncFrom(*robot);
    }
    else{
        Eigen::VectorXd _qhome;
        model->getRobotState("home", _qhome);
        model->setJointPosition(_qhome);
        model->update();
        std::string _urdf_param_name = "/xbotcore/" + model->getUrdf().getName() + "/robot_description";
        nh.setParam(_urdf_param_name, model->getUrdfString());
    
    }
    
    
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(model->getUrdf(), kdl_tree);
    robot_state_publisher::RobotStatePublisher rspub(kdl_tree);

    
    std::vector<std::string> feet_links = {"foot_fl", "foot_fr", "foot_hr", "foot_hl"};
    std::vector<std::string> wheels_links = {"wheel_1", "wheel_2", "wheel_4", "wheel_3"};
    
    auto model_mtx = std::make_shared<std::mutex>();
    centauro::LegMovementAction act(model, model_mtx, feet_links, wheels_links);
    
    ros::Rate loop_rate(100);
    
    XBot::JointNameMap joint_name_map;
    
    while(ros::ok()){
        
        ros::spinOnce();
        
        if(!visual_mode){
            
            { 
                std::lock_guard<std::mutex>(*model_mtx);
                robot->setReferenceFrom(*model, XBot::Sync::Position, XBot::Sync::Velocity);
            }
            
            robot->move();
        }
        else{
            
            {
                std::lock_guard<std::mutex>(*model_mtx);
                model->getJointPosition(joint_name_map);
            }
            
            std::map<std::string, double> _joint_name_std_map(joint_name_map.begin(), joint_name_map.end());

            rspub.publishTransforms(_joint_name_std_map, ros::Time::now(), "");
            rspub.publishFixedTransforms("");
        }
        
        loop_rate.sleep();
    }
    
    
    return 0;
    
    
}