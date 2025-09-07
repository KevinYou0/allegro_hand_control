#include "allegro_node_grasp.h"

#include "bhand/BHand.h"

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME},
        {"ready",    eMotionType_READY},  // ready position
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
        // These ones do not appear to do anything useful (or anything at all):
        // {"pregrasp", eMotionType_PRE_SHAPE},  // not sure what this is supposed to do.
        // {"move_object", eMotionType_OBJECT_MOVING},
        // {"move_fingertip", eMotionType_FINGERTIP_MOVING}
};

AllegroNodeGrasp::AllegroNodeGrasp()
        : AllegroNode() {

  initController(whichHand);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &AllegroNodeGrasp::setJointCallback, this);

  joint_detect_sub_l = nh.subscribe(
          DESIRED_HOLO_STATE_TOPIC_L, 1, &AllegroNodeGrasp::libHoloCallback_l, this);

  joint_detect_sub_r = nh.subscribe(
          DESIRED_HOLO_STATE_TOPIC_R, 1, &AllegroNodeGrasp::libHoloCallback_r, this);
  
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeGrasp::libCmdCallback, this);

  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &AllegroNodeGrasp::envelopTorqueCallback,
          this);
  // cuo_trigger_sub = nh.subscribe(
  //         "gripper_control_panda1", 1, &AllegroNodeGrasp::cuoCmdCallback,
  //         this);
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand_left;
  delete pBHand_right;
}

// void AllegroNodeGrasp::cuoCmdCallback(const sensor_msgs::Joy::ConstPtr &msg) {

//   /////////////////////////
//   /////////////////////////
//   // cuo operation/////////
//   /////////////////////////
//   /////////////////////////
//   // cuo_command = msg->buttons[0];
//   // if (cuo_command != cuo_command_old)
//   // {
//   //   cuo_control = 1 - cuo_control;
//   // }

//   // root control
//   // pBHand->SetMotionType(itr->second);
//   // ROS_INFO("haha in cmd mode");
//   // if (cuo_control == 1)
//   // {
//   //   // set index
//   //   for (int i = 0; i <= 15; i++)
//   //   {
//   //     if (ros_desired_position_1[i] > 0.0)
//   //     {
//   //       desired_position[i] = multi_pose * ros_desired_position_1[i];
//   //     }
//   //   }
//   // }
//   // else if (cuo_control == 0)
//   // {
//   //   // set index
//   //   for (int i = 0; i <= 15; i++)
//   //   {
//   //     if (ros_desired_position_3[i] > 0.0)
//   //     {
//   //       desired_position[i] = multi_pose * ros_desired_position_3[i];
//   //     }
//   //   }
//   // }
//   // else
//   // {
//   //   for (int i = 0; i <= 15; i++)
//   //   {
//   //     if (ros_desired_position_2[i] > 0.0)
//   //     {
//   //       desired_position[i] = multi_pose * ros_desired_position_2[i];
//   //     }
//   //   }
//   // }
//   pBHand->SetJointDesiredPosition(desired_position);
//   pBHand->SetMotionType(eMotionType_JOINT_PD);
//   pBHand->SetGainsEx(kp,kd);

//   cuo_command_old = cuo_command;
// }


void AllegroNodeGrasp::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data;

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand_left->SetJointDesiredPosition(desired_position_l);
    pBHand_left->SetMotionType(eMotionType_JOINT_PD);


  } else if (lib_cmd.compare("pdControl") == 0) {
    
    ROS_INFO("haha in pd mode");
    // Desired position only necessary if in PD Control mode
    for (int i = 4; i < 16; i++)
    {
      desired_position_l[i] = ros_desired_position_l[i];
      desired_position_r[i] = ros_desired_position_r[i];
    }
    pBHand_left->SetJointDesiredPosition(ros_desired_position_test);
    pBHand_left->SetMotionType(eMotionType_JOINT_PD);

  } else if (lib_cmd.compare("save") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position_l[i] = current_position[i];
  } else {
    ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
  }  
}

void AllegroNodeGrasp::libHoloCallback_l(const sensor_msgs::Joy::ConstPtr &msg) {
  // get detected data
  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    ros_desired_position_l[i] = msg->axes[i];
  }
  // set index
  for (int i = 0; i <= 15; i++)
  {
    if (ros_desired_position_l[i] > 0.0)
    {
      desired_position_l[i] = multi_pose * ros_desired_position_l[i];
    }
    else
    {
      desired_position_l[i] = ros_desired_position_l[i];
    }
  }
  // if (whichHand.compare("left") == 0) 
  // {
  //   pBHand_left->SetJointDesiredPosition(ros_desired_position_l);
  //   pBHand_left->SetMotionType(eMotionType_JOINT_PD);
  // }
}

void AllegroNodeGrasp::libHoloCallback_r(const sensor_msgs::Joy::ConstPtr &msg) {
  // get detected data
  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    ros_desired_position_r[i] = msg->axes[i];
  }
  // set index
  for (int i = 0; i <= 15; i++)
  {
    if (ros_desired_position_r[i] > 0.0)
    {
      desired_position_r[i] = multi_pose * ros_desired_position_r[i];
    }
    else
    {
      desired_position_r[i] = ros_desired_position_r[i];
    }
  }
  // if (whichHand.compare("right") == 0)
  // {
  //   pBHand_left->SetJointDesiredPosition(ros_desired_position_r);
  //   pBHand_left->SetMotionType(eMotionType_JOINT_PD);
  // }
}


// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();
  for (int i = 0; i < DOF_JOINTS; i++)
  {
    desired_position_l[i] = msg.position[i];
    desired_position_r[i] = msg.position[i];  
  }
  mutex->unlock();

  pBHand_left->SetJointDesiredPosition(desired_position_l);
  pBHand_left->SetMotionType(eMotionType_JOINT_PD);

}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::Float32 &msg) {
  const double torque = msg.data;
  ROS_INFO("Setting envelop torque to %.3f.", torque);
  pBHand_left->SetEnvelopTorqueScalar(torque);
  // pBHand_right->SetEnvelopTorqueScalar(torque);
}

void AllegroNodeGrasp::computeDesiredTorque() {
  // compute control torque using Bhand library
  pBHand_left->SetJointPosition(current_position_filtered);
  // BHand lib control updated with time stamp
  pBHand_left->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);
  // Necessary torque obtained from Bhand libs
  pBHand_left->GetJointTorque(desired_torque);
}

void AllegroNodeGrasp::initController(const std::string &whichHand) {
  // Initialize BHand controller
  if (whichHand.compare("left") == 0) {
    pBHand_left = new BHand(eHandType_Left);
    ROS_WARN("CTRL: Left Allegro Hand controller initialized.");
  }
  else {
    pBHand_left = new BHand(eHandType_Right);
    ROS_WARN("CTRL: Right Allegro Hand controller initialized.");
  }
  pBHand_left->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand_left->SetMotionType(eMotionType_NONE);
  // pBHand_right->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  // pBHand_right->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
  {
    desired_position_l[i] = current_position[i];
    desired_position_r[i] = current_position[i];   
  }

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNodeGrasp::doIt(bool polling) {
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGrasp grasping;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  ROS_INFO("Start controller with polling = %d", polling);
  
  grasping.doIt(polling);
}

