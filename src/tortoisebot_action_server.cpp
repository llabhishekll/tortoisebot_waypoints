#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

class WaypointActionClass : public rclcpp::Node {
private:
  // substitution
  using WaypointInterface = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<WaypointInterface>;

  // member variables
  double x;
  double y;
  double yaw;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp_action::Server<WaypointInterface>::SharedPtr action_server;

  // member method
  void subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;

    // convert quaternion into euler angles
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    this->yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  rclcpp_action::GoalResponse
  action_handle_goal(const rclcpp_action::GoalUUID &uuid,
                     std::shared_ptr<const WaypointInterface::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void action_handle_accepted(
      const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    callback_execute(goal_handle);
  }

  void callback_execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    // goal request
    auto target_position = goal_handle->get_goal()->position;
    RCLCPP_INFO(this->get_logger(), "goal (%f, %f) received", target_position.x,
                target_position.y);

    // flag variable
    bool goal_done = false;

    //  goal response and feedback
    auto result = std::make_shared<WaypointInterface::Result>();
    auto feedback = std::make_shared<WaypointInterface::Feedback>();

    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    // control loop
    rclcpp::Rate loop_rate(25);
    while (!goal_done && rclcpp::ok()) {
      // if goal canceled
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "The goal has been cancelled");

        // halt robot on current location
        message.linear.x = 0;
        message.angular.z = 0;
        publisher_cmd_vel->publish(message);

        // return result
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      // distance delta calculation
      double delta_x = target_position.x - this->x;
      double delta_y = target_position.y - this->y;
      double target_angle = std::atan2(delta_y, delta_x);
      double delta_dist = sqrt(delta_x * delta_x + delta_y * delta_y);
      double delta_yaw = target_angle - this->yaw;

      RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", this->yaw);
      RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", target_angle);
      RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", delta_yaw);
      RCLCPP_INFO(this->get_logger(), "Error Dist: %f", delta_dist);

      // control logic
      if (std::fabs(delta_yaw) > M_PI / 90) {
        feedback->state = "fix yaw";
        message.linear.x = 0.0;
        message.angular.z = 0.65 * (delta_yaw > 0 ? 1 : -1);
      } else {
        feedback->state = "go to point";
        message.linear.x = 0.6;
        message.angular.z = 0;
      }

      if (std::fabs(delta_dist) < 0.05) {
        feedback->state = "reached";
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        goal_done = true;
      }

      // publish velocity to publisher
      this->publisher_cmd_vel->publish(message);

      // publish feedback
      feedback->position.x = this->x;
      feedback->position.y = this->y;
      goal_handle->publish_feedback(feedback);

      // deep sleep
      loop_rate.sleep();
    }

    // on completion return result
    if (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached");
      result->success = true;
      goal_handle->succeed(result);
    }
  }

  rclcpp_action::CancelResponse
  action_handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

public:
  WaypointActionClass() : Node("tortoisebot_as") {
    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ros objects
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WaypointActionClass::subscriber_callback, this,
                  std::placeholders::_1),
        sub_callback_g1);
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->action_server = rclcpp_action::create_server<WaypointInterface>(
        this, "/tortoisebot_as",
        std::bind(&WaypointActionClass::action_handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointActionClass::action_handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&WaypointActionClass::action_handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), callback_g2);
    RCLCPP_INFO(this->get_logger(), "Action server started");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<WaypointActionClass>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}