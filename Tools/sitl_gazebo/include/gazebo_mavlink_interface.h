/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "CommandMotorSpeed.pb.h"
#include "MotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <stdio.h>

#include "common.h"

#include "SensorImu.pb.h"
#include "opticalFlow.pb.h"
#include "lidar.pb.h"
#include "sonarSens.pb.h"
#include "TargetPositionImage.pb.h"
#include "GimbalCommand.pb.h"
#include "platformLanding.pb.h"
#include <boost/bind.hpp>

#include <iostream>
#include <math.h>
#include <deque>
#include <random>
#include <sdf/sdf.hh>

#include "mavlink/v1.0/dronecourse/mavlink.h"

#include "gazebo/math/Vector3.hh"
#include <sys/socket.h>
#include <netinet/in.h>

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

static const uint32_t kDefaultMavlinkUdpPort = 14560;

namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const lidar_msgs::msgs::lidar> LidarPtr;
typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sonarSens_msgs::msgs::sonarSens> SonarSensPtr;
typedef const boost::shared_ptr<const target_camera::msgs::TargetPositionImage> TargetPosPtr;
typedef const boost::shared_ptr<const platform_msgs::msgs::PlatformLanding> PlatformLandingPtr;

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultGimbalCommandPubTopic = "/gazebo/command/gimbal_command";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultLidarTopic = "/lidar/link/lidar";
static const std::string kDefaultOpticalFlowTopic = "/camera/link/opticalFlow";
static const std::string kDefaultSonarTopic = "/sonar_model/link/sonar";
static const std::string kDefaultTargetPosTopic = "/TargetPos";
static const std::string kDefaultPlatformLandingTopic = "~/platform/landing";

class GazeboMavlinkInterface : public ModelPlugin {
 public:
  GazeboMavlinkInterface()
      : ModelPlugin(),

        received_first_referenc_(false),
        namespace_(kDefaultNamespace),
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
        gimbal_command_pub_topic_(kDefaultGimbalCommandPubTopic),
        imu_sub_topic_(kDefaultImuTopic),
        opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
        lidar_sub_topic_(kDefaultLidarTopic),
        sonar_sub_topic_(kDefaultSonarTopic),
        targetPos_sub_topic_(kDefaultTargetPosTopic),
        platform_landing_sub_topic_(kDefaultPlatformLandingTopic),
        model_{},
        world_(nullptr),
        left_elevon_joint_(nullptr),
        right_elevon_joint_(nullptr),
        elevator_joint_(nullptr),
        propeller_joint_(nullptr),
        gimbal_yaw_joint_(nullptr),
        gimbal_pitch_joint_(nullptr),
        gimbal_roll_joint_(nullptr),
        input_offset_{},
        input_scaling_{},
        zero_position_disarmed_{},
        zero_position_armed_{},
        input_index_{},
        lat_rad(0.0),
        lon_rad(0.0),
        mavlink_udp_port_(kDefaultMavlinkUdpPort)
        {}
  ~GazeboMavlinkInterface();

  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  bool received_first_referenc_;
  Eigen::VectorXd input_reference_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string gimbal_command_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;
  std::string platform_landing_sub_topic_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::PublisherPtr gimbal_command_pub_;
  transport::SubscriberPtr mav_control_sub_;
  transport::SubscriberPtr platform_landing_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_elevon_joint_;
  physics::JointPtr right_elevon_joint_;
  physics::JointPtr elevator_joint_;
  physics::JointPtr propeller_joint_;
  physics::JointPtr gimbal_yaw_joint_;
  physics::JointPtr gimbal_pitch_joint_;
  physics::JointPtr gimbal_roll_joint_;
  common::PID propeller_pid_;
  common::PID elevator_pid_;
  common::PID left_elevon_pid_;
  common::PID right_elevon_pid_;
  bool use_propeller_pid_;
  bool use_elevator_pid_;
  bool use_left_elevon_pid_;
  bool use_right_elevon_pid_;

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  void LidarCallback(LidarPtr& lidar_msg);
  void SonarCallback(SonarSensPtr& sonar_msg);
  void TargetPosCallback(TargetPosPtr& pose_message);
  void PlatformLandingCallback(PlatformLandingPtr& platform_landing_msg);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
  void handle_message(mavlink_message_t *msg);
  void pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs);

  static const unsigned n_out_max = 16;

  unsigned _rotor_count;

  double input_offset_[n_out_max];
  double input_scaling_[n_out_max];
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max];
  double zero_position_armed_[n_out_max];
  int input_index_[n_out_max];
  transport::PublisherPtr joint_control_pub_[n_out_max];

  transport::SubscriberPtr imu_sub_;
  transport::SubscriberPtr lidar_sub_;
  transport::SubscriberPtr sonar_sub_;
  transport::SubscriberPtr targetPos_sub_;
  transport::SubscriberPtr opticalFlow_sub_;
  transport::PublisherPtr gps_pub_;
  std::string imu_sub_topic_;
  std::string lidar_sub_topic_;
  std::string opticalFlow_sub_topic_;
  std::string sonar_sub_topic_;
  std::string targetPos_sub_topic_;

  common::Time last_time_;
  common::Time last_gps_time_;
  common::Time last_actuator_time_;
  double gps_update_interval_;
  double lat_rad;
  double lon_rad;
  void handle_control(double _dt);

  math::Vector3 gravity_W_;
  math::Vector3 velocity_prev_W_;
  math::Vector3 mag_d_;

  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  int _fd;
  struct sockaddr_in _myaddr;  ///< The locally bound address
  struct sockaddr_in _srcaddr;  ///< SITL instance
  socklen_t _addrlen;
  unsigned char _buf[65535];
  struct pollfd fds[1];

  struct sockaddr_in _srcaddr_2;  ///< MAVROS

  //so we dont have to do extra callbacks
  double optflow_xgyro;
  double optflow_ygyro;
  double optflow_zgyro;
  double optflow_distance;
  double sonar_distance;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_;

  // GPS coordinates for gazebo origin
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;

  };
}
