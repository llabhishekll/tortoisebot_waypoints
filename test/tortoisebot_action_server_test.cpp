#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "gtest/gtest.h"

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class TestWaypointsActionServer : public ::testing::Test {
private:
  // node object
  std::shared_ptr<rclcpp::Node> node;

  // substitution
  using WaypointInterface = tortoisebot_waypoints::action::WaypointAction;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<WaypointInterface>;

  // member variables
  double x;
  double y;
  double yaw;

  // flag variables
  bool goal_done;

  // error variables
  double error_x;
  double error_y;
  double error_yaw;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp_action::Client<WaypointInterface>::SharedPtr action_client;

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

  void callback_goal_response(const ClientGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_INFO(this->node->get_logger(), "Goal Rejected");
    } else {
      RCLCPP_INFO(this->node->get_logger(), "Goal Accepted");
    }
  }

  void callback_goal_result(const ClientGoalHandle::WrappedResult &result) {
    (void)result;
    this->goal_done = true;
  }

  void callback_goal_feedback(
      const ClientGoalHandle::SharedPtr &goal_handle,
      const std::shared_ptr<const WaypointInterface::Feedback> feedback) {
    (void)goal_handle;
    (void)feedback;
  }

  void send_action_goal(double x, double y) {
    // target yaw
    double yaw = std::atan2(y - this->y, x - this->x);

    // define goal
    auto goal = WaypointInterface::Goal();
    goal.position.x = x;
    goal.position.y = y;

    // define goal options
    auto options = rclcpp_action::Client<WaypointInterface>::SendGoalOptions();
    options.goal_response_callback =
        std::bind(&TestWaypointsActionServer::callback_goal_response, this,
                  std::placeholders::_1);
    options.result_callback =
        std::bind(&TestWaypointsActionServer::callback_goal_result, this,
                  std::placeholders::_1);
    options.feedback_callback =
        std::bind(&TestWaypointsActionServer::callback_goal_feedback, this,
                  std::placeholders::_1, std::placeholders::_2);

    // send goal to server
    this->action_client->async_send_goal(goal, options);

    // loop until goal done
    while (!this->goal_done) {
      rclcpp::spin_some(this->node);
    }

    // calculate error
    this->error_x = x - this->x;
    this->error_y = y - this->y;
    this->error_yaw = yaw - this->yaw;
  }

public:
  TestWaypointsActionServer() : goal_done(false) {
    // node object
    this->node = rclcpp::Node::make_shared("tortoisebot_as_test");

    // ros objects
    this->action_client = rclcpp_action::create_client<WaypointInterface>(
        this->node, "/tortoisebot_as");
    this->subscriber_odom =
        this->node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TestWaypointsActionServer::subscriber_callback, this,
                      std::placeholders::_1));

    // send goal
    this->action_client->wait_for_action_server();
    this->send_action_goal(0.25, 0.25);
  }

  bool test_orientation() {
    // Test Case 1 : validate robot orientation error
    return (std::fabs(this->error_yaw) <= 6.50);
  }

  bool test_position() {
    // Test Case 2 : validate robot position error
    return (std::fabs(this->error_x) <= 0.25 &&
            std::fabs(this->error_y) <= 0.25);
  }
};

TEST_F(TestWaypointsActionServer, TestOrientation) {
  EXPECT_TRUE(test_orientation());
}
TEST_F(TestWaypointsActionServer, TestPosition) {
  EXPECT_TRUE(test_position());
}