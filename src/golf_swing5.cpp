#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_poseA;
  desired_poseA.header.frame_id = "base_link";
  desired_poseA.link_name = "r_wrist_roll_link";
  desired_poseA.pose.position.x = 0.5;
  desired_poseA.pose.position.y = -0.5;
  desired_poseA.pose.position.z = 0.46;

  desired_poseA.pose.orientation.x = 0.0;
  desired_poseA.pose.orientation.y = 0.0;
  desired_poseA.pose.orientation.z = 0.0;
  desired_poseA.pose.orientation.w = 1.0;

  desired_poseA.absolute_position_tolerance.x = 0.05;
  desired_poseA.absolute_position_tolerance.y = 0.05;
  desired_poseA.absolute_position_tolerance.z = 0.05;

  desired_poseA.absolute_roll_tolerance = 0.1;
  desired_poseA.absolute_pitch_tolerance = 0.1;
  desired_poseA.absolute_yaw_tolerance = 0.1;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_poseA,goalA);
  
  arm_navigation_msgs::MoveArmGoal goalB;

  goalB.motion_plan_request.group_name = "right_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.planner_id = std::string("");
  goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_poseB;
  desired_poseB.header.frame_id = "base_link";
  desired_poseB.link_name = "r_wrist_roll_link";
  desired_poseB.pose.position.x = 0.5;
  desired_poseB.pose.position.y = -0.2;
  desired_poseB.pose.position.z = 0.46;

  desired_poseB.pose.orientation.x = 0.0;
  desired_poseB.pose.orientation.y = 0.0;
  desired_poseB.pose.orientation.z = 0.0;
  desired_poseB.pose.orientation.w = 1.0;

  desired_poseB.absolute_position_tolerance.x = 0.05;
  desired_poseB.absolute_position_tolerance.y = 0.05;
  desired_poseB.absolute_position_tolerance.z = 0.05;

  desired_poseB.absolute_roll_tolerance = 0.1;
  desired_poseB.absolute_pitch_tolerance = 0.1;
  desired_poseB.absolute_yaw_tolerance = 0.1;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_poseB,goalB);

  arm_navigation_msgs::MoveArmGoal goalC;

  goalC.motion_plan_request.group_name = "right_arm";
  goalC.motion_plan_request.num_planning_attempts = 1;
  goalC.motion_plan_request.planner_id = std::string("");
  goalC.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalC.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_poseC;
  desired_poseC.header.frame_id = "base_link";
  desired_poseC.link_name = "r_wrist_roll_link";
  desired_poseC.pose.position.x = 0.5;
  desired_poseC.pose.position.y = 0;
  desired_poseC.pose.position.z = 0.46;

  desired_poseC.pose.orientation.x = 0.0;
  desired_poseC.pose.orientation.y = 0.0;
  desired_poseC.pose.orientation.z = 0.0;
  desired_poseC.pose.orientation.w = 1.0;

  desired_poseC.absolute_position_tolerance.x = 0.05;
  desired_poseC.absolute_position_tolerance.y = 0.05;
  desired_poseC.absolute_position_tolerance.z = 0.05;

  desired_poseC.absolute_roll_tolerance = 0.1;
  desired_poseC.absolute_pitch_tolerance = 0.1;
  desired_poseC.absolute_yaw_tolerance = 0.1;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_poseC,goalC);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
    finished_within_time = false;
    move_arm.sendGoal(goalB);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal B");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
    finished_within_time = false;
    move_arm.sendGoal(goalC);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal C");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
