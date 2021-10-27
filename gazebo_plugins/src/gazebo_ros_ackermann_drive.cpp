// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_ackermann_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosAckermannDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
      /// Use an encoder
      ENCODER = 0,

      /// Use ground truth from simulation world
      WORLD = 1,

      /// Use the parametric error model from Probabilistic Robotics (3rd Ed.) p. 136
      PARAMETRIC_ERROR_MODEL = 2,
  };

  /// Indicates which joint
  enum
  {
    /// Front right wheel
    FRONT_RIGHT,

    /// Front left wheel
    FRONT_LEFT,

    /// Rear right wheel
    REAR_RIGHT,

    /// Rear left wheel
    REAR_LEFT,

    /// Right steering
    STEER_RIGHT,

    /// Left steering
    STEER_LEFT,

    /// Steering wheel
    STEER_WHEEL
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Extracts radius of a cylinder or sphere collision shape
  /// \param[in] _coll Pointer to collision
  /// \return If the collision shape is valid, return radius
  /// \return If the collision shape is invalid, return 0
  double CollisionRadius(const gazebo::physics::CollisionPtr & _coll);

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Update ground truth according to world
  void UpdateGroundTruthWorld();

  /// Update distance
  void UpdateDistance();

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Update odometry according to the parametric error model
  void UpdateOdometryParametricErrorModel(const gazebo::common::Time & _current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish ground truth transforms
  /// \param[in] _current_time Current simulation time
  void PublishGroundTruthTf(const gazebo::common::Time & _current_time);

  /// Publish transforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// Publish ground truth messages
  /// \param[in] _current_time Current simulation time
  void PublishGroundTruthMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Ground truth publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;

  /// Distance publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Distance between the front wheels, in meters.
  double front_wheel_separation_;

  /// Distance between the rear wheels, in meters.
  double rear_wheel_separation_;

  /// Distance between front and rear axles, in meters.
  double wheel_base_;

  /// Radius of rear wheels, in meters.
  double wheel_radius_;

  /// Angle ratio between the steering wheel and the front wheels
  double steering_ratio_ = 0;

  // Max steering angle
  double max_speed_ = 0;

  // Max steering angle of tyre
  double max_steer_ = 0;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_linear_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Keep latest world odometry pose
  geometry_msgs::msg::Pose2D last_pose_;

  /// Keep odometry pose from error model
  geometry_msgs::msg::Pose2D pose_error_model_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Ground truth parent frame ID
  std::string ground_truth_parent_frame_;

  /// Ground truth robot base frame ID
  std::string ground_truth_robot_base_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update_;

  /// Last time the odometry error model was updated
  gazebo::common::Time last_odometry_error_model_update_;

  /// Either ENCODER, WORLD or PARAMETRIC_ERROR_MODEL
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Keep latest odometry message to compute the distance
  nav_msgs::msg::Odometry distance_odom_;

  /// Keep latest ground truth message
  nav_msgs::msg::Odometry ground_truth_odom_;

  /// Keep latest distance message
  std_msgs::msg::Float32 distance_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish ground truth messages.
  bool publish_ground_truth_;

  /// True to publish distance travelled
  bool publish_distance_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// True to publish ground truth transforms.
  bool publish_ground_truth_tf_;

  /// Covariance in odometry
  double covariance_[3];

  /// PID control for left steering control
  gazebo::common::PID pid_left_steering_;

  /// PID control for right steering control
  gazebo::common::PID pid_right_steering_;

  /// PID control for linear velocity control
  gazebo::common::PID pid_linear_vel_;

  /// Parameters of odometry error model
  double alpha1_, alpha2_, alpha3_, alpha4_;

};

GazeboRosAckermannDrive::GazeboRosAckermannDrive()
: impl_(std::make_unique<GazeboRosAckermannDrivePrivate>())
{
}

GazeboRosAckermannDrive::~GazeboRosAckermannDrive()
{
}

void GazeboRosAckermannDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->joints_.resize(7);

  auto steering_wheel_joint =
    _sdf->Get<std::string>("steering_wheel_joint", "steering_wheel_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_WHEEL] =
    _model->GetJoint(steering_wheel_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_WHEEL]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Steering wheel joint [%s] not found.", steering_wheel_joint.c_str());
    impl_->joints_.resize(6);
  }

  auto front_right_joint = _sdf->Get<std::string>("front_right_joint", "front_right_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT] = _model->GetJoint(front_right_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front right wheel joint [%s] not found, plugin will not work.", front_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto front_left_joint = _sdf->Get<std::string>("front_left_joint", "front_left_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT] = _model->GetJoint(front_left_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front left wheel joint [%s] not found, plugin will not work.", front_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_right_joint = _sdf->Get<std::string>("rear_right_joint", "rear_right_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT] = _model->GetJoint(rear_right_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear right wheel joint [%s] not found, plugin will not work.", rear_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_left_joint = _sdf->Get<std::string>("rear_left_joint", "rear_left_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT] = _model->GetJoint(rear_left_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear left wheel joint [%s] not found, plugin will not work.", rear_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto right_steering_joint =
    _sdf->Get<std::string>("right_steering_joint", "right_steering_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_RIGHT] =
    _model->GetJoint(right_steering_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Right wheel steering joint [%s] not found, plugin will not work.",
      right_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto left_steering_joint =
    _sdf->Get<std::string>("left_steering_joint", "left_steering_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_LEFT] =
    _model->GetJoint(left_steering_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Left wheel steering joint [%s] not found, plugin will not work.",
      left_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  impl_->max_speed_ = _sdf->Get<double>("max_speed", 20.0).first;
  impl_->max_steer_ = _sdf->Get<double>("max_steer", 0.6).first;

  // Max the steering wheel can rotate
  auto max_steering_angle = _sdf->Get<double>("max_steering_angle", 7.85).first;

  // Compute the angle ratio between the steering wheel and the tires
  impl_->steering_ratio_ = impl_->max_steer_ / max_steering_angle;

  auto pid = _sdf->Get<ignition::math::Vector3d>(
    "right_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  auto i_range = _sdf->Get<ignition::math::Vector2d>(
    "right_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_right_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "left_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "left_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_left_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>("linear_velocity_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>("linear_velocity_i_range", ignition::math::Vector2d::Zero).first;
  auto cmd_range = _sdf->Get<ignition::math::Vector2d>("linear_velocity_pid_cmd_range",
                                                       ignition::math::Vector2d(0, -1)).first;
  impl_->pid_linear_vel_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X(), cmd_range.Y(), cmd_range.X());

  // Update wheel radius for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  unsigned int id = 0;
  impl_->wheel_radius_ = impl_->CollisionRadius(
    impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]->GetChild()->GetCollision(id));

  // Compute wheel_base, front wheel separation, and rear wheel separation
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of joint and has only one collision
  auto front_right_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto front_left_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_right_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_left_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();

  impl_->front_wheel_separation_ = (front_left_center_pos - front_right_center_pos).Length();
  impl_->rear_wheel_separation_ = (rear_left_center_pos - rear_right_center_pos).Length();

  // to compute wheelbase, first position of axle centers are computed
  auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;
  auto rear_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;
  // then the wheelbase is the distance between the axle centers
  impl_->wheel_base_ = (front_axle_pos - rear_axle_pos).Length();

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosAckermannDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<GazeboRosAckermannDrivePrivate::OdomSource>(
          _sdf->Get<int>("odometry_source", GazeboRosAckermannDrivePrivate::OdomSource::WORLD).first);

  // Ground truth
  impl_->publish_ground_truth_tf_ = _sdf->Get<bool>("publish_ground_truth_tf", false).first;
  impl_->ground_truth_parent_frame_ = _sdf->Get<std::string>("ground_truth_parent_frame", "map").first;
  impl_->ground_truth_robot_base_frame_ = _sdf->Get<std::string>("ground_truth_robot_base_frame",
                                                                 "base_footprint_gt").first;

  // Odometry error model parameters
  if (impl_->odom_source_ == GazeboRosAckermannDrivePrivate::OdomSource::PARAMETRIC_ERROR_MODEL) {
    impl_->alpha1_ = _sdf->Get<double>("alpha1", 0.001).first;
    impl_->alpha2_ = _sdf->Get<double>("alpha2", 0.001).first;
    impl_->alpha3_ = _sdf->Get<double>("alpha3", 0.001).first;
    impl_->alpha4_ = _sdf->Get<double>("alpha4", 0.001).first;

    RCLCPP_INFO(impl_->ros_node_->get_logger(),
                "Computing odometry with parametric error model\n"
                "alpha1 [%f]\n"
                "alpha2 [%f]\n"
                "alpha3 [%f]\n"
                "alpha4 [%f]\n",
                impl_->alpha1_,
                impl_->alpha2_,
                impl_->alpha3_,
                impl_->alpha4_);
  }

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Advertise ground truth topic
  impl_->publish_ground_truth_ = _sdf->Get<bool>("publish_ground_truth", false).first;
  if (impl_->publish_ground_truth_) {
    impl_->ground_truth_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
            "ground_truth_odom", qos.get_publisher_qos("ground_truth_odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
            impl_->ros_node_->get_logger(), "Advertise ground truth on [%s]",
            impl_->ground_truth_pub_->get_topic_name());
  }

  // Advertise distance travelled
  impl_->publish_distance_ = _sdf->Get<bool>("publish_distance", false).first;
  if (impl_->publish_distance_) {
    impl_->distance_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
      "distance", qos.get_publisher_qos("distance", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise distance on [%s]",
      impl_->distance_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_ || impl_->publish_ground_truth_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    if (impl_->publish_wheel_tf_) {
      for (auto & joint : impl_->joints_) {
        RCLCPP_INFO(
          impl_->ros_node_->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame_.c_str(), joint->GetName().c_str(), joint->GetName().c_str());
      }
    }

    if (impl_->publish_ground_truth_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
                  "Publishing ground truth odom transforms between [%s] and [%s]",
                  impl_->ground_truth_parent_frame_.c_str(),
                  impl_->ground_truth_robot_base_frame_.c_str());
    }

  }

  auto pose = impl_->model_->WorldPose();
  impl_->odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  impl_->odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(
    pose.Rot());

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosAckermannDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosAckermannDrive::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  impl_->target_linear_ = 0;
  impl_->target_rot_ = 0;
  impl_->distance_.data = 0;
}

void GazeboRosAckermannDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosAckermannDrivePrivate::OnUpdate");
  #endif
  std::lock_guard<std::mutex> lock(lock_);

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom message
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }

  // Update encoder even if we're going to skip this update
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

  // Update odom message if using parametric error model
  if (odom_source_ == PARAMETRIC_ERROR_MODEL) {
    UpdateOdometryParametricErrorModel(_info.simTime);
  }

  // Update distance
  if (publish_distance_) {
    UpdateDistance();
  }

  // Update ground truth message if using ground truth
  if (publish_ground_truth_tf_) {
    UpdateGroundTruthWorld();
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (seconds_since_last_update < update_period_) {
    return;
  }

  if (publish_distance_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish distance");
#endif
    distance_pub_->publish(distance_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }
  if (publish_ground_truth_) {
    PublishGroundTruthMsg(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }
  if (publish_ground_truth_tf_) {
    PublishGroundTruthTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("update");
#endif
  // Current speed
  auto target_linear_vel = ignition::math::clamp(target_linear_, -max_speed_, max_speed_);
  auto rear_left_angular_vel = joints_[REAR_LEFT]->GetVelocity(0);
  auto rear_right_angular_vel = joints_[REAR_RIGHT]->GetVelocity(0);
  auto current_linear_vel = (rear_left_angular_vel + rear_right_angular_vel) / 2 * wheel_radius_;
  double linear_vel_diff = current_linear_vel - target_linear_vel;
  double linear_cmd = pid_linear_vel_.Update(linear_vel_diff, seconds_since_last_update);

  auto target_rot = target_rot_;
  target_rot = ignition::math::clamp(target_rot, -max_steer_, max_steer_);

  double tanSteer = tan(target_rot);

  auto target_left_steering =
    atan2(tanSteer, 1.0 - front_wheel_separation_ / 2.0 / wheel_base_ * tanSteer);
  auto target_right_steering =
    atan2(tanSteer, 1.0 + front_wheel_separation_ / 2.0 / wheel_base_ * tanSteer);

  auto left_steering_angle = joints_[STEER_LEFT]->Position(0);
  auto right_steering_angle = joints_[STEER_RIGHT]->Position(0);

  double left_steering_diff = left_steering_angle - target_left_steering;
  double left_steering_cmd =
    pid_left_steering_.Update(left_steering_diff, seconds_since_last_update);

  double right_steering_diff = right_steering_angle - target_right_steering;
  double right_steering_cmd =
    pid_right_steering_.Update(right_steering_diff, seconds_since_last_update);

  auto steer_wheel_angle = (left_steering_angle + right_steering_angle) * 0.5 / steering_ratio_;

  joints_[STEER_LEFT]->SetForce(0, left_steering_cmd);
  joints_[STEER_RIGHT]->SetForce(0, right_steering_cmd);
  joints_[REAR_RIGHT]->SetForce(0, linear_cmd);
  joints_[REAR_LEFT]->SetForce(0, linear_cmd);

  if (joints_.size() == 7) {
    joints_[STEER_WHEEL]->SetPosition(0, steer_wheel_angle);
  }

  last_update_time_ = _info.simTime;
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

void GazeboRosAckermannDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_linear_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

double GazeboRosAckermannDrivePrivate::CollisionRadius(const gazebo::physics::CollisionPtr & _coll)
{
  if (!_coll || !(_coll->GetShape())) {
    return 0;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    return cyl->GetRadius();
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

void GazeboRosAckermannDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  double vl = joints_[REAR_LEFT]->GetVelocity(0);
  double vr = joints_[REAR_RIGHT]->GetVelocity(0);

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = rear_wheel_separation_;

  // Book: Sigwart 2011 Autonomous Mobile Robots page:337
  double sl = vl * wheel_radius_ * seconds_since_last_update;
  double sr = vr * wheel_radius_ * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}

void GazeboRosAckermannDrivePrivate::UpdateDistance()
{
  auto prev_x = distance_odom_.pose.pose.position.x;
  auto prev_y = distance_odom_.pose.pose.position.y;

  auto pose = model_->WorldPose();
  distance_odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  distance_odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  distance_.data += static_cast<float>(hypot(
          prev_x - distance_odom_.pose.pose.position.x,
          prev_y - distance_odom_.pose.pose.position.y));

}
void GazeboRosAckermannDrivePrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosAckermannDrivePrivate::UpdateGroundTruthWorld()
{
  auto pose = model_->WorldPose();
  ground_truth_odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  ground_truth_odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  ground_truth_odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  ground_truth_odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  ground_truth_odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosAckermannDrivePrivate::UpdateOdometryParametricErrorModel(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  double seconds_since_last_update = (_current_time - last_odometry_error_model_update_).Double();
  last_odometry_error_model_update_ = _current_time;

  auto current_pose = geometry_msgs::msg::Pose2D();
  current_pose.x = pose.Pos().X();
  current_pose.y = pose.Pos().Y();
  current_pose.theta = pose.Rot().Yaw();

  auto delta = geometry_msgs::msg::Pose2D();
  delta.x = current_pose.x - last_pose_.x;
  delta.y = current_pose.y - last_pose_.y;
  delta.theta = current_pose.theta - last_pose_.theta;
  delta.theta = atan2(sin(delta.theta), cos(delta.theta));

  double delta_trans = sqrt(std::pow(delta.x, 2) + std::pow(delta.y, 2));
  double delta_rot1 = atan2(delta.y, delta.x) - last_pose_.theta;
  delta_rot1 = atan2(sin(delta_rot1), cos(delta_rot1));

  std::normal_distribution<double> delta_rot_rv(
          (1 + alpha1_) * delta.theta + alpha2_ * delta_trans,
          0.0
  );

  std::normal_distribution<double> delta_trans_rv(
          (1 + alpha3_) * delta_trans + alpha4_ * delta.theta,
          0.0
  );
//
//  std::normal_distribution<double> delta_rot_rv(
//          delta.theta,
//          sqrt(std::pow(alpha1_ * delta.theta, 2) + std::pow(alpha2_ * delta_trans, 2))
//  );
//
//  std::normal_distribution<double> delta_trans_rv(
//          delta_trans,
//          sqrt(std::pow(alpha3_ * delta_trans, 2) + std::pow(alpha4_ * delta.theta, 2))
//  );

  std::random_device rd{};
  std::mt19937 rng{rd()};
  double delta_rot_hat = delta_rot_rv(rng);
  double delta_trans_hat = delta_trans_rv(rng);

  // Add the delta with additional error to the odometry pose
  pose_error_model_.x += delta_trans_hat * cos(pose_error_model_.theta + delta_rot1);
  pose_error_model_.y += delta_trans_hat * sin(pose_error_model_.theta + delta_rot1);
  pose_error_model_.theta += delta_rot_hat;

  // Compute the ros odometry message and tf
  double w = delta_rot_hat / seconds_since_last_update;
  double v = delta_trans_hat / seconds_since_last_update;
  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_error_model_.theta);
  vt = tf2::Vector3(pose_error_model_.x, pose_error_model_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;

  last_pose_ = current_pose;
}

void GazeboRosAckermannDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosAckermannDrivePrivate::PublishGroundTruthTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = ground_truth_parent_frame_;
  msg.child_frame_id = ground_truth_robot_base_frame_;
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(ground_truth_odom_.pose.pose.position);
  msg.transform.rotation = ground_truth_odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosAckermannDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (const auto & joint : joints_) {
    auto pose = joint->GetChild()->WorldPose() - model_->WorldPose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = robot_base_frame_;
    msg.child_frame_id = joint->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosAckermannDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}

void GazeboRosAckermannDrivePrivate::PublishGroundTruthMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  ground_truth_odom_.pose.covariance[0] = covariance_[0];
  ground_truth_odom_.pose.covariance[7] = covariance_[1];
  ground_truth_odom_.pose.covariance[14] = 1000000000000.0;
  ground_truth_odom_.pose.covariance[21] = 1000000000000.0;
  ground_truth_odom_.pose.covariance[28] = 1000000000000.0;
  ground_truth_odom_.pose.covariance[35] = covariance_[2];

  ground_truth_odom_.twist.covariance[0] = covariance_[0];
  ground_truth_odom_.twist.covariance[7] = covariance_[1];
  ground_truth_odom_.twist.covariance[14] = 1000000000000.0;
  ground_truth_odom_.twist.covariance[21] = 1000000000000.0;
  ground_truth_odom_.twist.covariance[28] = 1000000000000.0;
  ground_truth_odom_.twist.covariance[35] = covariance_[2];

  // Set header
  ground_truth_odom_.header.frame_id = ground_truth_parent_frame_;
  ground_truth_odom_.child_frame_id = ground_truth_robot_base_frame_;
  ground_truth_odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  ground_truth_pub_->publish(ground_truth_odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosAckermannDrive)
}  // namespace gazebo_plugins
