#include <uvdar_leader_follower/follower.h>
#include <uvdar_leader_follower/FollowerConfig.h>

bool is_initialized = false;
bool got_odometry   = false;
bool got_uvdar      = false;

Eigen::Vector3d follower_position;
Eigen::Vector3d follower_rpy;
Eigen::Vector3d follower_linear_velocity;
Eigen::Vector3d follower_angular_velocity;
double          follower_heading;

Eigen::Vector3d leader_position;
ros::Time       last_leader_contact;

// dynamically reconfigurable
Eigen::Vector3d position_offset    = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset     = 0.0;
double          uvdar_msg_interval = 0.1;
bool            use_estimator      = false;
bool            use_speed_tracker  = false;

VelocityEstimator estimator;
Eigen::Vector3d   leader_predicted_position;
Eigen::Vector3d   leader_predicted_velocity;

/* initialize //{ */
uvdar_leader_follower::FollowerConfig FollowerController::initialize(mrs_lib::ParamLoader& param_loader) {

  ROS_INFO("[Follower]: Waiting for odometry and uvdar");
  while (ros::ok()) {
    if (got_odometry && got_uvdar) {
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 3, 3> R;
  param_loader.loadMatrixStatic("Q", Q);
  param_loader.loadMatrixStatic("R", R);
  param_loader.loadParam("control_action_interval", control_action_interval);
  param_loader.loadParam("desired_offset/x", position_offset.x());
  param_loader.loadParam("desired_offset/y", position_offset.y());
  param_loader.loadParam("desired_offset/z", position_offset.z());
  param_loader.loadParam("heading_offset", heading_offset);

  //// initialize the dynamic reconfigurables with values from YAML file
  uvdar_leader_follower::FollowerConfig config;
  config.desired_offset_x  = position_offset.x();
  config.desired_offset_y  = position_offset.y();
  config.desired_offset_z  = position_offset.z();
  config.heading_offset    = heading_offset;
  config.filter_data       = use_estimator;
  config.use_speed_tracker = use_speed_tracker;
  ////

  VelocityEstimator::kalman3D::x_t initial_states;
  initial_states << follower_position.x() - position_offset.x(), follower_position.y() - position_offset.y(), follower_position.z() - position_offset.z(), 0, 0,
      0;
  estimator = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);


  is_initialized = true;
  return config;
}
//}

/* dynamicReconfigureCallback //{ */
void FollowerController::dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  position_offset   = Eigen::Vector3d(config.desired_offset_x, config.desired_offset_y, config.desired_offset_z);
  heading_offset    = config.heading_offset;
  use_speed_tracker = config.use_speed_tracker;

  if (!use_estimator && config.filter_data) {
    ROS_INFO("[%s]: Estimator started", ros::this_node::getName().c_str());
  }
  use_estimator = config.filter_data;
}
//}

/* receiveOdometry //{ */
void FollowerController::receiveOdometry(const nav_msgs::Odometry& odometry_msg) {

  follower_position.x() = odometry_msg.pose.pose.position.x;
  follower_position.y() = odometry_msg.pose.pose.position.y;
  follower_position.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  follower_rpy[0] = ac.getRoll();
  follower_rpy[1] = ac.getPitch();
  follower_rpy[2] = ac.getYaw();

  follower_heading = ac.getHeading();

  follower_linear_velocity.x() = odometry_msg.twist.twist.linear.x;
  follower_linear_velocity.y() = odometry_msg.twist.twist.linear.y;
  follower_linear_velocity.z() = odometry_msg.twist.twist.linear.z;

  follower_angular_velocity.x() = odometry_msg.twist.twist.angular.x;
  follower_angular_velocity.y() = odometry_msg.twist.twist.angular.y;
  follower_angular_velocity.z() = odometry_msg.twist.twist.angular.z;

  got_odometry = true;
}
//}

/* receiveUvdar //{ */
void FollowerController::receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg) {

  Eigen::Vector3d leader_new_position;

  leader_new_position.x() = uvdar_msg.pose.pose.position.x;
  leader_new_position.y() = uvdar_msg.pose.pose.position.y;
  leader_new_position.z() = uvdar_msg.pose.pose.position.z;


  last_leader_contact = uvdar_msg.header.stamp;
  got_uvdar           = true;

  leader_position = leader_new_position;

  if (use_estimator) {
    estimator.fuse(leader_new_position);
  }
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }

  if (use_estimator) {
    point.position.x() = leader_predicted_position.x() + position_offset.x();
    point.position.y() = leader_predicted_position.y() + position_offset.y();
    point.position.z() = leader_predicted_position.z() + position_offset.z();
  } else {
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = leader_position.z() + position_offset.z();
  }
  point.heading         = heading_offset;
  point.use_for_control = true;

  return point;
}
//}

/* createSpeedCommand //{ */
SpeedCommand FollowerController::createSpeedCommand() {
  SpeedCommand command;

  if (!got_odometry || !got_uvdar) {
    command.velocity        = Eigen::Vector3d(0, 0, 0);
    command.heading         = 0;
    command.height          = 0;
    command.use_for_control = false;
  }

  if (use_estimator) {
    command.velocity = leader_predicted_velocity;
    command.height   = leader_predicted_position.z() + position_offset.z();
    command.heading  = follower_heading;
  }

  if (use_speed_tracker) {
    command.use_for_control = true;
  } else {
    command.use_for_control = false;
  }
  return command;
}
//}

/* getCurrentEstimate //{ */
nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    auto leader_prediction          = estimator.predict(Eigen::Vector3d(0, 0, 0), control_action_interval);
    leader_predicted_position       = Eigen::Vector3d(leader_prediction[0], leader_prediction[1], leader_prediction[2]);
    leader_predicted_velocity       = Eigen::Vector3d(leader_prediction[3], leader_prediction[4], leader_prediction[5]);
    leader_est.pose.pose.position.x = leader_prediction[0];
    leader_est.pose.pose.position.y = leader_prediction[1];
    leader_est.pose.pose.position.z = leader_prediction[2];
    leader_est.twist.twist.linear.x = leader_prediction[3];
    leader_est.twist.twist.linear.y = leader_prediction[4];
    leader_est.twist.twist.linear.z = leader_prediction[5];
  } else {
    leader_est.pose.pose.position.x = leader_position.x();
    leader_est.pose.pose.position.y = leader_position.y();
    leader_est.pose.pose.position.z = leader_position.z();
  }

  return leader_est;
}
//}
