#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

std::unique_ptr<ScoutRobot> robot;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // 설정된 파라미터를 로드합니다.
  bool is_scout_mini = true;  // 기본적으로 scout mini 사용
  static bool is_scout_omni = false;  // omni 기능은 사용하지 않음
  bool simulated_robot = false;  // Real 모드 사용
  // bool simulated_robot = true;  // 시뮬레이션 모드 사용
  bool pub_tf = true;  // TF 퍼블리시 활성화

  // ScoutROSMessenger를 초기화합니다.
  ScoutROSMessenger messenger(nullptr, &node, is_scout_omni);

  // 노드 파라미터를 가져옵니다.
  std::string odom_frame, base_frame, odom_topic_name;
  int control_rate;
  
  private_node.param<std::string>("odom_frame", odom_frame, std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame, std::string("base_link"));
  private_node.param<std::string>("odom_topic_name", odom_topic_name, std::string("odom"));
  private_node.param<int>("control_rate", control_rate, 50);
  private_node.param<bool>("pub_tf", pub_tf, true);

  // Messenger 설정
  messenger.odom_frame_ = odom_frame;
  messenger.base_frame_ = base_frame;
  messenger.odom_topic_name_ = odom_topic_name;
  messenger.simulated_robot_ = simulated_robot;
  messenger.sim_control_rate_ = control_rate;
  messenger.pub_tf = pub_tf;

  // 시뮬레이션 모드로 동작합니다.
  messenger.SetupSubscription();

  // 50Hz로 로봇 상태를 퍼블리시하고 Twist 명령을 수신합니다.
  ros::Rate rate(50);
  while (ros::ok()) {
    if (simulated_robot) {
      double linear = 0.0, angular = 0.0;

      // 시뮬레이션 모드에서 Twist 명령을 가져옵니다.
      messenger.GetCurrentMotionCmdForSim(linear, angular);

      // 시뮬레이션된 상태를 ROS에 퍼블리시합니다.
      messenger.PublishSimStateToROS(linear, angular);

    } else {
      messenger.PublishStateToROS();
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
