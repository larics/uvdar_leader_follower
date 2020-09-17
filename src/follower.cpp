#include <uvdar_leader_follower/follower.h>
#include <uvdar_leader_follower/FollowerConfig.h>
#include <uvdar_leader_follower/util.h>
#include <cmath>
using namespace std;

bool is_initialized     = false;
bool got_odometry       = false;
bool got_tracker_output = false;
bool got_uvdar          = false;

Eigen::Vector3d follower_position_odometry;
Eigen::Vector3d follower_linear_velocity_odometry;
double          follower_heading_odometry;
double          follower_heading_rate_odometry;

Eigen::Vector3d follower_position_tracker;
Eigen::Vector3d follower_linear_velocity_tracker;
double          follower_heading_tracker;
double          follower_heading_rate_tracker;

Eigen::Vector3d leader_position;
ros::Time       last_leader_contact;

// dynamically reconfigurable
Eigen::Vector3d position_offset          = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset           = 0.0;
double          uvdar_msg_interval       = 0.1;
bool            use_estimator            = true;
bool            use_speed_tracker        = true;
bool            use_trajectory_reference = false;

// constants
static constexpr auto MAX_HEIGHT = 3.8;
static constexpr auto MIN_HEIGHT = 2.2;
static constexpr auto MAX_VEL = 4.5;
static constexpr auto POS_ERROR_GAIN = 1;
int cnt = 0;

VelocityEstimator estimator;
Eigen::Vector3d   leader_predicted_position;
Eigen::Vector3d   leader_predicted_position_old;
Eigen::Vector3d   leader_predicted_velocity;
Eigen::Vector3d   position_error;
Eigen::Vector3d   position_error_old;

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

  //// initialize the dynamic reconfigurables with values from YAML file and values set above
  uvdar_leader_follower::FollowerConfig config;
  config.desired_offset_x         = position_offset.x();
  config.desired_offset_y         = position_offset.y();
  config.desired_offset_z         = position_offset.z();
  config.heading_offset           = heading_offset;
  config.filter_data              = use_estimator;
  config.use_trajectory_reference = use_trajectory_reference;
  config.use_speed_tracker        = use_speed_tracker;
  ////

  VelocityEstimator::kalman3D::x_t initial_states;

  // set initial state of estimator as follows: leader position: (current follower pos - desired offset), leader velocity: (0,0,0)
  initial_states << follower_position_odometry.x() - position_offset.x(), follower_position_odometry.y() - position_offset.y(),
      follower_position_odometry.z() - position_offset.z(), 0, 0, 0;
  estimator = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);

  is_initialized = true;
  return config;
}
//}

/* dynamicReconfigureCallback //{ */
void FollowerController::dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  position_offset          = Eigen::Vector3d(config.desired_offset_x, config.desired_offset_y, config.desired_offset_z);
  heading_offset           = config.heading_offset;
  use_speed_tracker        = config.use_speed_tracker;
  use_trajectory_reference = config.use_trajectory_reference;

  if (!use_estimator && config.filter_data) {
    ROS_INFO("[%s]: Estimator started", ros::this_node::getName().c_str());
  }
  use_estimator = config.filter_data;
}
//}

/* receiveOdometry //{ */
void FollowerController::receiveOdometry(const nav_msgs::Odometry& odometry_msg) {

  follower_position_odometry.x() = odometry_msg.pose.pose.position.x;
  follower_position_odometry.y() = odometry_msg.pose.pose.position.y;
  follower_position_odometry.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  follower_heading_odometry = ac.getHeading();

  follower_linear_velocity_odometry.x() = odometry_msg.twist.twist.linear.x;
  follower_linear_velocity_odometry.y() = odometry_msg.twist.twist.linear.y;
  follower_linear_velocity_odometry.z() = odometry_msg.twist.twist.linear.z;

  follower_heading_rate_odometry =
      ac.getHeadingRate(Eigen::Vector3d(odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z));

  got_odometry = true;
}
//}

/* receiveTrackerOutput //{ */
void FollowerController::receiveTrackerOutput(const mrs_msgs::PositionCommand& position_cmd) {

  follower_position_tracker.x() = position_cmd.position.x;
  follower_position_tracker.y() = position_cmd.position.y;
  follower_position_tracker.z() = position_cmd.position.z;

  follower_heading_tracker = position_cmd.heading;

  follower_linear_velocity_tracker.x() = position_cmd.velocity.x;
  follower_linear_velocity_tracker.y() = position_cmd.velocity.y;
  follower_linear_velocity_tracker.z() = position_cmd.velocity.z;

  follower_heading_rate_tracker = position_cmd.heading_rate;

  got_tracker_output = true;
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

  if (use_estimator && is_initialized) {
    estimator.fuse(leader_new_position);
  }
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }

  if (use_estimator) {
    static auto old_time = ros::Time::now().toSec();
    auto new_time = ros::Time::now().toSec();
    auto dt = new_time - old_time;
    point.position.x() = leader_predicted_position.x() + position_offset.x() + leader_predicted_velocity.x() * dt;
    point.position.y() = leader_predicted_position.y() + position_offset.y() + leader_predicted_velocity.y() * dt;
    point.position.z() = leader_predicted_position.z() + position_offset.z();
    old_time = new_time;
  } else {
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = leader_position.z() + position_offset.z();
  }
  point.heading         = heading_offset;
  point.use_for_control = true;
  
  point.position.z() = sss_util::saturation(point.position.z(), MIN_HEIGHT, MAX_HEIGHT);

  return point;
}
//}

/* createReferenceTrajectory //{ */
ReferenceTrajectory FollowerController::createReferenceTrajectory() {
  ReferenceTrajectory trajectory;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    trajectory.positions.push_back(Eigen::Vector3d::Zero());
    trajectory.headings.push_back(0.0);
    trajectory.sampling_time   = 0.0;
    trajectory.use_for_control = false;
    return trajectory;
  }

  // Example - start trajectory at current UAV position and move in the predicted direction of leader motion
  // No subsampling, only two points are created in this example
  Eigen::Vector3d point_1;
  double          heading_1;

  Eigen::Vector3d point_2;
  double          heading_2;

  trajectory.use_for_control = false;
  if (use_trajectory_reference) {
    if (use_estimator) {
      point_1   = follower_position_tracker;
      heading_1 = follower_heading_tracker;

      point_2   = leader_predicted_position + position_offset + (leader_predicted_velocity * control_action_interval);
      auto ref_head = atan2(leader_predicted_position.y() - point_1.y(), leader_predicted_position.x() - point_1.x());
      
      heading_2 = ref_head + heading_offset;
      point_2.z() = sss_util::saturation(point_2.z(), MIN_HEIGHT, MAX_HEIGHT);

      trajectory.positions.push_back(point_1);
      trajectory.positions.push_back(point_2);

      trajectory.headings.push_back(heading_1);
      trajectory.headings.push_back(heading_2);
      trajectory.sampling_time   = control_action_interval;
      trajectory.use_for_control = true;
    } else {
      ROS_WARN("[%s]: Tried to plan a trajectory without leader velocity estimation", ros::this_node::getName().c_str());
    }
  }

  return trajectory;
}
//}

/* createSpeedCommand //{ */
SpeedCommand FollowerController::createSpeedCommand() {
  SpeedCommand command;

  if (!got_odometry || !got_uvdar || !got_tracker_output) {
    command.velocity        = Eigen::Vector3d(0, 0, 0);
    command.heading         = 0;
    command.height          = 0;
    command.use_for_control = false;
  }

  if (use_estimator) {
    position_error_old = position_error;
    position_error = leader_predicted_position - follower_position_odometry;
    Eigen::Vector3d velocity_setpoint;
    Eigen::Vector3d position_controller_output;
    cnt++;

    // Calculate position offset dynamically
    double offset = 7.0;
    double yaw = atan2(leader_predicted_position.y() - follower_position_odometry.y(), leader_predicted_position.x() - follower_position_odometry.x());
    Eigen::Vector3d dynamic_position_offset;
    dynamic_position_offset[0] = -cos(yaw)*offset;
    dynamic_position_offset[1] = -sin(yaw)*offset;
    dynamic_position_offset[2] = 0.0;

    // Now we get position error from offseted position
    position_error += dynamic_position_offset;//position_offset;//dynamic_position_offset;

    // position controller output, also saturate it
    Eigen::Vector3d P = 0.5 * position_error;
    Eigen::Vector3d D = 0.7 * (position_error - position_error_old)/control_action_interval;
    position_controller_output = P+D;
    auto pos_mag = position_controller_output.norm();
    //pos_mag = sss_util::saturation(pos_mag, -1.5, 1.5);
    if (pos_mag < 0.0){
      cout << "pos_mag " << pos_mag << endl;
    }
    // OVO TREBA PROVJERITI!!
    position_controller_output.normalize();
    position_controller_output *= pos_mag;

    // Saturate leader predicted velocity, it should not exceed 3m/s
    auto vel_mag = leader_predicted_velocity.norm()*1.0;
    //vel_mag = sss_util::saturation(vel_mag, -3.5, 3.5);
    //cout << vel_mag << endl;
    if (vel_mag < 0.0){
      cout << "vel_mag " << vel_mag << endl;
    }
    Eigen::Vector3d saturated_leader_velocity = leader_predicted_velocity;
    saturated_leader_velocity.normalize();
    saturated_leader_velocity *= vel_mag;

    double start_speed_tracker = 0.0;
    if (cnt > 400){
      start_speed_tracker = 1.0;
      ROS_INFO_ONCE("Pozor! Spustim rizeni rychlosti!");
    }
    velocity_setpoint = (position_controller_output + saturated_leader_velocity)*start_speed_tracker;
    velocity_setpoint[2] = 0.0;
    double final_mag = velocity_setpoint.norm();
    final_mag = sss_util::saturation(final_mag, -4.95, 4.95);
    velocity_setpoint.normalize();
    velocity_setpoint *= final_mag;
    if (final_mag > 4.94) cout << "Saturated " << cnt << endl;
    
    //velocity_setpoint = leader_predicted_velocity;
    //cout << "vel setpoint " << endl << velocity_setpoint << endl << endl;

    // max speed back if the distance is less than x meters
    double yaw2 = atan2(leader_predicted_velocity.y(), leader_predicted_velocity.x());
    if ((leader_predicted_position - follower_position_odometry).norm() < 5.75/* ||
      ((fabs(yaw2+yaw) < 0.15) && leader_predicted_velocity.norm() > 1.0)*/){
      ROS_INFO("Nebezpeci! Bude to blizko.");
      velocity_setpoint[0] = -cos(yaw)*4.9;
      velocity_setpoint[1] = -sin(yaw)*4.9;
      velocity_setpoint[2] = 0.0;
    }
    
    //command.velocity = leader_predicted_velocity; //velocity_setpoint;
    command.velocity = velocity_setpoint;
    command.velocity[2] = 0.0;
    command.height   = leader_predicted_position.z() + position_offset.z();
    command.heading  = follower_heading_odometry;
    command.height = sss_util::saturation(command.height, MIN_HEIGHT, MAX_HEIGHT);

    auto ref_head = atan2(leader_predicted_position.y() - follower_position_odometry.y(), leader_predicted_position.x() - follower_position_odometry.x());
    command.heading  = ref_head + heading_offset;
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

// You can use this method for debugging purposes.
// It allows you to visualize the leader predictions in rviz
// It is called once per control action of the summer_schoo_supervisor

nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    leader_predicted_position_old = leader_predicted_position;
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
