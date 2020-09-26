#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_ur.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#define robot_enable

using GraspPlanning = moveit_msgs::srv::GraspPlanning;

/* place position in [x, y, z, R, P, Y]*/
static std::vector<double> place_ = {-0.425, -0.286, 0.240, 0.7248, 3.055, 0.1050};
/* pre-pick position in joint values*/
static std::vector<double> prepick_joints = {-1.85, -1.43, -1.57, -1.99, 1.48, -0.26};
/* place position in joint values*/
static std::vector<double> preplace_joints = {-2.32, -1.43, -1.57, -1.99, 1.48, -0.26};

static double vel_ = 0.9, acc_ = 0.9, vscale_ = 0.9, appr_ = 0.1;
static std::shared_ptr<URControl> robot_ = nullptr;
static rclcpp::Node::SharedPtr node_ = nullptr;
static std::shared_ptr<GraspPlanning::Response> result_ = nullptr;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // init random pick node
  node_ = rclcpp::Node::make_shared("random_pick");
  tf2_ros::StaticTransformBroadcaster tfb(node_);

  // create client for grasp planning
  auto client = node_->create_client<GraspPlanning>("plan_grasps");
  // wait for service
  while (!client->wait_for_service(5s)) {
    RCLCPP_INFO(node_->get_logger(), "Wait for service");
  }
  RCLCPP_INFO(node_->get_logger(), "Got service");
  rclcpp::sleep_for(1s);

  // init robot control
  robot_ = std::make_shared<URControl>("robot_control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_->parseArgs();
  robot_->startLoop();
  rclcpp::sleep_for(1s);

#ifdef robot_enable
  // reset joint
  robot_->moveToJointValues(prepick_joints, vel_, acc_);
#endif

  while (rclcpp::ok()) {
    // request grasp poses
    auto request = std::make_shared<GraspPlanning::Request>();
    auto result_future = client->async_send_request(request);
    RCLCPP_WARN(node_->get_logger(), "Request sent");

    // wait for response
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
      continue;
    }

    // get response
    if (moveit_msgs::msg::MoveItErrorCodes::SUCCESS == result_future.get()->error_code.val) {
      result_ = result_future.get();
      RCLCPP_INFO(node_->get_logger(), "Response received %d", result_->error_code.val);
    }
    else
      continue;

    geometry_msgs::msg::PoseStamped p = result_->grasps[0].grasp_pose;
    // publish grasp pose
    tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 r;
    r.setRotation(q);
    r.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(node_->get_logger(), "**********pick pose [position %f %f %f, quat %f %f %f %f, RPY %f %f %f]",
        p.pose.position.x, p.pose.position.y, p.pose.position.z,
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,
        roll, pitch, yaw);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = p.header;
    tf_msg.child_frame_id = "grasp_pose";
    tf_msg.transform.translation.x = p.pose.position.x;
    tf_msg.transform.translation.y = p.pose.position.y;
    tf_msg.transform.translation.z = p.pose.position.z;
    tf_msg.transform.rotation = p.pose.orientation;
    tfb.sendTransform(tf_msg);

#ifdef robot_enable
    // pick
    robot_->moveToJointValues(prepick_joints, vel_, acc_);
    robot_->pick(p, vel_, acc_, vscale_, appr_);

    // place
    robot_->moveToJointValues(preplace_joints, vel_, acc_);
    // robot_->place(place_[0], place_[1], place_[2], place_[3], place_[4], place_[5], vel_, acc_, vscale_, appr_);

    // reset joint
    robot_->moveToJointValues(prepick_joints, 1., 1.);
#else
    RCLCPP_INFO(node_->get_logger(), "move to pick");
    RCLCPP_INFO(node_->get_logger(), "pick");
    RCLCPP_INFO(node_->get_logger(), "move to home");
#endif
  }

  rclcpp::shutdown();
  return 0;
}
