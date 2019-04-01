/**
Implementation of smooth control law derived from:
A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment

Paper found here:

https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
*/

#ifndef SMOOTH_CONTROL_H
#define SMOOTH_CONTROL_H

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <cmath>
#include <optional>

#include <nav_msgs/Path.h>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>

struct Action
{
  double w;
  double dt;
};

class SmoothControl
{
public:
  SmoothControl(double k1, double k2, double axle_length, double simulation_frequency, double target_velocity,
                double m_lookahead_dist, double simulation_horizon, double target_reached_distance,
                double target_move_threshold, double acceleration_limit, double transition_distance);
  /**
   * Generate an immediate velocity command and visualize a smooth control trajectory
   * using the procedure described in 'A Smooth Control Law for Graceful Motion of
   * Differential Wheeled Mobile Robots in 2D Environment'. A radius of curvature is
   * generated and then combined with a target velocity to produce a control law for
   * the angular velocity of the robot (steering).
   *
   * @param[out] vel velocity_pair message to store command in
   * @param[in] path path to generate smooth control law for
   * @param[out] trajectory msg to store visualization trajectory in
   * @param[in] start_pos current position of the robot
   * @param[out] target the target pose the controller is planning for
   */
  void getPath(igvc_msgs::velocity_pair &vel, const nav_msgs::PathConstPtr &path, nav_msgs::Path &trajectory,
               const RobotState &start_pos, RobotState &target);

private:
  double k1_, k2_;
  double axle_length_;
  double simulation_frequency_;
  double target_velocity_;
  double lookahead_dist_;
  double simulation_horizon_;
  double target_reached_distance_;
  double target_move_threshold_;
  double acceleration_limit_;
  double transition_distance_;
  ros::Publisher target_pub_;
  ros::Publisher closest_point_pub_;
  std::optional<RobotState> target_;

  /**
   * Computes the control command to be executed for the next dt seconds
   * @param[in] state current state of the robot
   * @param[in] target target state of the robot
   * @param[in] second_target the target state after the next target
   * @param[in] dt the duration for which the generated command will be executed for
   * @return A control command for the next iteration
   */
  Action getAction(const RobotState& state, const RobotState& target, const RobotState& second_target,
                   const ros::Duration& dt) const;

  /**
   * Returns the curvature of the path from the current state to the target
   * @param state starting state
   * @param target target state
   * @return intantaneous curvature of the path from state to target
   */
  double getCurvature(const RobotState& state, const RobotState& target) const;

  /**
   * Propogates the current state given the current velocity and angular velocity
   * command for visualization purposes.
   * angles [delta, theta].
   *
   * Makes extensive use of differential drive odometry equations to calculate resultant
   * pose.
   *
   * Source: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
   *
   * @param[in] action Trajectory action to visualize
   * @param[in/out] state The state to use for state propogation
   */
  void propogateState(const Action& action, RobotState& state);

  /**
   * Returns if we have reached the target, given the current state and the target
   * @param state state to check for
   * @param target target to check for
   * @return whether or not we have reached the target
   */
  bool reachedTarget(const RobotState& state, const RobotState& target) const;

  /**
   * Find the furthest point along trajectory which is at least lookahead_dist_ away from the current position.
   *
   * @param[in] path path to get target position from
   * @param[in] path_index index of closest position along the path relative to current position
   * @param[in] state current state of the robot
   * @param[in] path_index the index of the closest point on the path to the current state
   * @return the positions of the first and second targets.
   */
  std::pair<RobotState, RobotState> getTargetPosition(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                                      const std::optional<RobotState>& target, size_t path_index) const;

  /**
   * Acquires a new target which is at least lookahead_dist_ away from the current position
   * @param[in] path path from which to find a target position
   * @param[in] state current state of the robot
   * @param[in] state_index the index of the closest point on the path to the current state
   * @return the newly acquired target position
   */
  RobotState acquireNewTarget(const nav_msgs::PathConstPtr& path, const RobotState& state, size_t state_index) const;

  /**
   * Finds the point closest to the old target on the current path. If the distance is greater
   * than new_target_threshold_, std::nullopt is returned.
   * @return the found target if within the threshold, or std::nullopt
   */
  size_t findTargetOnPath(const nav_msgs::PathConstPtr& path, const RobotState& target) const;

  /**
   * Finds the distance along the path provided from the current state to the target state
   * @param path the path to calculate the distance along
   * @param state the current state
   * @param target the target state
   * @return the distance from the current state to the target state along the given path
   */
  double distAlongPath(const nav_msgs::PathConstPtr& path, size_t path_index, size_t target_index) const;

  /**
   * Returns the closest point on the path to the given state
   * @param path path to find closest point on
   * @param state state to find closest point on path for
   * @return the index of the path which is closest to the state.
   */
  size_t getClosestIndex(const nav_msgs::PathConstPtr& path, const RobotState& state) const;

  /**
   * Calculates the velocity to go at for the next dt that stays within the acceleration limits
   * @param[in] state current state, which includes current velocity
   * @param[in] k1 curvature of path to follow to the current target
   * @param[in] k2 curvature of path to follow to the next target
   * @param[in] dt timestep to generate control for
   * @return an Action that follows the curvature and stays within the acceleration limits
   */
  Action motionProfile(const RobotState& state, double distance, double K1, double K2, const ros::Duration& dt) const;

  /**
   * Returns an action given the target left and right wheel velocities
   * @param left left wheel velocity
   * @param right right wheel velocity
   * @return the action that would result from the given left and right wheel velocities
   */
  Action toAction(VelocityProfile left, VelocityProfile right) const;

  /**
   * Converts velocity and angular velocity to left and right wheel velocities
   * @param[in] vel_msg message to store wheel velocities in
   * @param[in/out] action Action to convert from
   */
  void getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, double w, double v) const;
};

#endif
