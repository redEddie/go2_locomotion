#include <array>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "leg_odometry_ros2/leg_kinematics.hpp"
#include "leg_odometry_ros2/moving_window_filter.hpp"

namespace leg_odometry_ros2
{
class LegOdometryNode : public rclcpp::Node
{
public:
  LegOdometryNode()
  : rclcpp::Node("leg_odometry_node"),
    filter_x_(1),
    filter_y_(1),
    filter_z_(1)
  {
    const int window_size = declare_parameter<int>("moving_window_size", 120);
    filter_x_ = MovingWindowFilter(static_cast<std::size_t>(window_size));
    filter_y_ = MovingWindowFilter(static_cast<std::size_t>(window_size));
    filter_z_ = MovingWindowFilter(static_cast<std::size_t>(window_size));

    lowstate_topic_ = declare_parameter<std::string>("lowstate_topic", "/lowstate");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/go2/leg_odom/twist");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/go2/leg_odom/odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    contact_force_threshold_ = declare_parameter<int>("contact_force_threshold", 65);
    jacobian_eps_ = declare_parameter<double>("jacobian_eps", 1e-5);
    tick_in_ms_ = declare_parameter<bool>("tick_in_ms", true);
    default_dt_ = declare_parameter<double>("default_dt", 0.002);

    rclcpp::QoS state_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    state_sub_ = create_subscription<unitree_go::msg::LowState>(
      lowstate_topic_, state_qos,
      std::bind(&LegOdometryNode::on_lowstate, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    RCLCPP_INFO(get_logger(), "leg_odometry_node ready: subscribing to %s", lowstate_topic_.c_str());
  }

private:
  void on_lowstate(const unitree_go::msg::LowState::SharedPtr msg)
  {
    const Eigen::Vector3d omega_b(msg->imu_state.gyroscope[0],
                                  msg->imu_state.gyroscope[1],
                                  msg->imu_state.gyroscope[2]);

    Eigen::Vector3d v_sum = Eigen::Vector3d::Zero();
    int stance_count = 0;

    for (int leg = 0; leg < 4; ++leg)
    {
      if (!is_stance_leg(*msg, leg))
      {
        continue;
      }

      const Eigen::Vector3d q = leg_joint_vector(*msg, leg);
      const Eigen::Vector3d qd = leg_joint_velocity(*msg, leg);
      const LegId leg_id = static_cast<LegId>(leg);

      const Eigen::Vector3d foot_pos = kinematics_.compute_foot_position(q, leg_id);
      const Eigen::Matrix3d J = kinematics_.compute_foot_jacobian(q, leg_id, jacobian_eps_);
      const Eigen::Vector3d leg_vel = J * qd;
      const Eigen::Vector3d base_vel = -(leg_vel - omega_b.cross(foot_pos));

      v_sum += base_vel;
      stance_count += 1;
    }

    Eigen::Vector3d v_base = Eigen::Vector3d::Zero();
    if (stance_count > 0)
    {
      v_base = v_sum / static_cast<double>(stance_count);
    }

    const Eigen::Vector3d v_filt(filter_x_.update(v_base.x()),
                                 filter_y_.update(v_base.y()),
                                 filter_z_.update(v_base.z()));

    publish_twist(*msg, v_filt);
    publish_odom(*msg, v_filt);
  }

  bool is_stance_leg(const unitree_go::msg::LowState &msg, int leg_index) const
  {
    if (leg_index < 0 || leg_index >= 4)
    {
      return false;
    }
    // Unit from SDK (unitree_go LowState). Adjust if SDK unit differs.
    return msg.foot_force[leg_index] >= contact_force_threshold_;
  }

  Eigen::Vector3d leg_joint_vector(const unitree_go::msg::LowState &msg, int leg_index) const
  {
    const int base_idx = leg_motor_base_index(leg_index);
    return Eigen::Vector3d(msg.motor_state[base_idx + 0].q,
                           msg.motor_state[base_idx + 1].q,
                           msg.motor_state[base_idx + 2].q);
  }

  Eigen::Vector3d leg_joint_velocity(const unitree_go::msg::LowState &msg, int leg_index) const
  {
    const int base_idx = leg_motor_base_index(leg_index);
    return Eigen::Vector3d(msg.motor_state[base_idx + 0].dq,
                           msg.motor_state[base_idx + 1].dq,
                           msg.motor_state[base_idx + 2].dq);
  }

  int leg_motor_base_index(int leg_index) const
  {
    // Unitree order: FR(0-2), FL(3-5), RR(6-8), RL(9-11).
    switch (leg_index)
    {
      case 0:  // FR
        return 0;
      case 1:  // FL
        return 3;
      case 2:  // RR
        return 6;
      case 3:  // RL
        return 9;
      default:
        return 0;
    }
  }

  double compute_dt(const unitree_go::msg::LowState &msg)
  {
    if (!has_last_tick_)
    {
      last_tick_ = msg.tick;
      has_last_tick_ = true;
      return default_dt_;
    }
    const int64_t tick_delta = static_cast<int64_t>(msg.tick) - static_cast<int64_t>(last_tick_);
    last_tick_ = msg.tick;
    if (tick_delta <= 0)
    {
      return default_dt_;
    }
    const double scale = tick_in_ms_ ? 0.001 : 1.0;
    return static_cast<double>(tick_delta) * scale;
  }

  void publish_twist(const unitree_go::msg::LowState &msg, const Eigen::Vector3d &v_base)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now();
    twist_msg.header.frame_id = base_frame_id_;
    twist_msg.twist.linear.x = v_base.x();
    twist_msg.twist.linear.y = v_base.y();
    twist_msg.twist.linear.z = v_base.z();
    twist_pub_->publish(twist_msg);
  }

  void publish_odom(const unitree_go::msg::LowState &msg, const Eigen::Vector3d &v_base)
  {
    const double dt = compute_dt(msg);
    const Eigen::Quaterniond q_world_from_base(
      msg.imu_state.quaternion[0],
      msg.imu_state.quaternion[1],
      msg.imu_state.quaternion[2],
      msg.imu_state.quaternion[3]);

    const Eigen::Vector3d v_world = q_world_from_base * v_base;
    position_ += v_world * dt;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = position_.x();
    odom_msg.pose.pose.position.y = position_.y();
    odom_msg.pose.pose.position.z = position_.z();
    odom_msg.pose.pose.orientation.w = q_world_from_base.w();
    odom_msg.pose.pose.orientation.x = q_world_from_base.x();
    odom_msg.pose.pose.orientation.y = q_world_from_base.y();
    odom_msg.pose.pose.orientation.z = q_world_from_base.z();
    odom_msg.twist.twist.linear.x = v_base.x();
    odom_msg.twist.twist.linear.y = v_base.y();
    odom_msg.twist.twist.linear.z = v_base.z();
    odom_pub_->publish(odom_msg);
  }

  std::string lowstate_topic_;
  std::string twist_topic_;
  std::string odom_topic_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  int contact_force_threshold_ = 65;
  double jacobian_eps_ = 1e-5;
  bool tick_in_ms_ = true;
  double default_dt_ = 0.002;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  LegKinematics kinematics_;
  MovingWindowFilter filter_x_;
  MovingWindowFilter filter_y_;
  MovingWindowFilter filter_z_;

  bool has_last_tick_ = false;
  uint32_t last_tick_ = 0;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
};
}  // namespace leg_odometry_ros2

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<leg_odometry_ros2::LegOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
