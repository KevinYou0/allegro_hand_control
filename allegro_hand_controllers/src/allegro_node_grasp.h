#ifndef PROJECT_ALLEGRO_NODE_GRASP_H
#define PROJECT_ALLEGRO_NODE_GRASP_H

#include "allegro_node.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"

// Forward class declaration.
class BHand;

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp (e.g., three-finger ping, envelop, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position, but in constrast to the PD node
// you do not have any control over the controller gains.
//
// Author: Felix Duvallet
//
class AllegroNodeGrasp : public AllegroNode {

 public:

    AllegroNodeGrasp();

    ~AllegroNodeGrasp();

    void initController(const std::string &whichHand);

    void computeDesiredTorque();

    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    void libHoloCallback_l(const sensor_msgs::Joy::ConstPtr &msg);

    void libHoloCallback_r(const sensor_msgs::Joy::ConstPtr &msg);

    void setJointCallback(const sensor_msgs::JointState &msg);

    void envelopTorqueCallback(const std_msgs::Float32 &msg);

    void cuoCmdCallback(const sensor_msgs::Joy::ConstPtr &msg);

    void doIt(bool polling);

 protected:

    // Handles external joint command (sensor_msgs/JointState).
    ros::Subscriber joint_cmd_sub;
    ros::Subscriber joint_detect_sub_l;
    ros::Subscriber joint_detect_sub_r;

    // Handles defined grasp commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

    ros::Subscriber envelop_torque_sub;

    ros::Subscriber cuo_trigger_sub;

    // Initialize BHand
    BHand *pBHand_left = NULL;
    BHand *pBHand_right = NULL;
    

  int cuo_command = 0;
  int cuo_command_old = 0;
  int cuo_control = 0;
  double multi_pose = 1.0;
  double desired_position_l[DOF_JOINTS] = {0.0};
  double desired_position_r[DOF_JOINTS] = {0.0};
  
  //double kp[DOF_JOINTS] = {500, 800, 900, 500, 500, 800, 900, 500, 500, 800, 900, 500, 1000, 700, 600, 600};
  
  double kp[DOF_JOINTS] = {1000, 1600, 1800, 1000, 1000, 1600, 1800, 1000, 1000, 1600, 1800, 1000, 2000, 1400, 1200, 1200};
  double kd[DOF_JOINTS] = {25, 50, 55, 40, 25, 50, 55, 40, 25, 50, 55, 40, 50, 50, 50, 40};

  // double kp[DOF_JOINTS] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,};
  // double kp[DOF_JOINTS] = {-600, -600, -600, -1000, -600, -600, -600, -1000, -600, -600, -600, -1000, -1000, -1000, -1000, -600,};
  // double kp[DOF_JOINTS] = {500, 500, 500, 500, 500, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,};


  // double kd[DOF_JOINTS] = {-150.0, -200.0, -150.0, -150.0, -150.0, -200.0, -150.0, -150.0, -150.0, -200.0, -150.0, -150.0, -300.0, -200.0, -200.0, -150.0};
  // double kd[DOF_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  //double ros_desired_position[DOF_JOINTS] = {0.0};
  double finger_index[3] = {0.05, 0.05, 0.05};
  // double ros_desired_position[DOF_JOINTS] = {-0.008353378855596436, 1.5074594362663634, 0.20582140373696892, 0.2280764583203702, 0.03142835235513788, 1.7184879834016404, 0.18132367212967682, -0.25212529374205217, 0.029052174565912654, 1.4686810509277561, 0.07597764094188524, 0.0037414273089661112, 1.3684906414672684, -0.033256754779417086, 0.14645159930457247, 1.0819818411075923};

  // free position
//   double ros_desired_position_1[DOF_JOINTS] = {0.05016486161788896, -0.040153412958728756, 0.4905858588030592, -0.019793448452197562, 0.2916829866421884, -0.12656689880916988, 0.5976206351605217, -0.11041210751704329, 0.02523891129763445, 0.009492744825731576, 0.6236970245584349, -0.025600961698601116, 1.4254833523289419, -0.06323080002774033, 1.1362781975699292, 0.2456977769148539};
//   double ros_desired_position_3[DOF_JOINTS] = {0.05016486161788896, -0.040153412958728756, 0.4905858588030592, -0.019793448452197562, 0.2916829866421884, -0.12656689880916988, 0.5976206351605217, -0.11041210751704329, 0.02523891129763445, 0.009492744825731576, 0.6236970245584349, -0.025600961698601116, 1.4254833523289419, -0.06323080002774033, 1.1362781975699292, 0.6456977769148539};

  double damuzhi_rotation = 0.0;
  double damuzhi_cuo_1 = 0.0;
  double damuzhi_cuo_2 = 0.0;

  double shizhi_rotation = -0.2;
  double shizhi_nie_1 = 0.30;
  double shizhi_nie_2 = 0.30;

  double zhongzhi_rotation = 0.2;
  double zhongzhi_nie_1 = 0.30;
  double zhongzhi_nie_2 = 0.30;

  double ros_desired_position_l[DOF_JOINTS] = {0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0};
                                        
  double ros_desired_position_r[DOF_JOINTS] = {0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0};

  double ros_desired_position_test[DOF_JOINTS] = {-0.1404067662136229 - shizhi_rotation, 0.7746382193919801 + shizhi_nie_1, 1.2477836105266493 + shizhi_nie_2, -0.03101587735345266, 
                                                0.05763875100631766 - zhongzhi_rotation, 0.8133656701645516 + zhongzhi_nie_1, 1.2282749235385246 + zhongzhi_nie_2, -0.06676199651743961, 
                                                -0.22874673944724444, 0.13091614994745326, 0.4727221514944348, 0.03172295698576558, 
                                                1.4791509846107849 - damuzhi_rotation, 0.09539243495887054, 0.460704242306852 - damuzhi_cuo_1, 0.7112192374023807 - damuzhi_cuo_2};




};

#endif //PROJECT_ALLEGRO_NODE_GRASP_H