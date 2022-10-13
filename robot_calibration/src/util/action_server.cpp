#include <robot_calibration/util/action_server.hpp>
#include <robot_calibration/util/poses_from_yaml.hpp>

#include <robot_calibration/optimization/ceres_optimizer.hpp>
#include <robot_calibration/optimization/export.hpp>
using namespace std::placeholders;

CalibratePoseServer::CalibratePoseServer(const rclcpp::Node::SharedPtr& node, std::string robot_poses_path)
: node_(node), robot_poses_path_(robot_poses_path)
{
  action_server_ = rclcpp_action::create_server<CalibratePose>(
      node_,
      "calibrate",
      std::bind(&CalibratePoseServer::handle_goal, this, _1, _2),
      std::bind(&CalibratePoseServer::handle_cancel, this, _1),
      std::bind(&CalibratePoseServer::handle_accepted, this, _1));

	capture_manager.init(node);

	// Save URDF for calibration/export step
	description_msg.data = capture_manager.getUrdf();

	load_robot_poses();
}

void CalibratePoseServer::load_robot_poses()
{
	RCLCPP_INFO(node_->get_logger(), "Loading YAML calibration poses from %s", robot_poses_path_.c_str());
    if (!robot_calibration::getPosesFromYaml(robot_poses_path_, robot_poses))
    {
      // Error will be printed in function
      RCLCPP_FATAL(node_->get_logger(), "Failed get get calibration poses from YAML file");
    }
}

rclcpp_action::GoalResponse CalibratePoseServer::handle_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const CalibratePose::Goal> goal)
{
	RCLCPP_INFO(node_->get_logger(), "Received goal request");
	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CalibratePoseServer::handle_cancel(
	const std::shared_ptr<GoalHandleCalibratePose> goal_handle)
{
	RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}

void CalibratePoseServer::handle_accepted(const std::shared_ptr<GoalHandleCalibratePose> goal_handle)
{
	auto logger = node_->get_logger();
	// Loop through robot poses if read from YAML or bag file
  if (!robot_poses.empty()) 
  {
    for (unsigned pose_idx = 0; pose_idx < robot_poses.size() && rclcpp::ok();
        ++pose_idx)
    {
      robot_calibration_msgs::msg::CalibrationData msg;

      // Move head/arm to pose
      if (!capture_manager.moveToState(robot_poses[pose_idx].joint_states))
      {
        RCLCPP_WARN(logger, "Unable to move to desired state for sample %u.", pose_idx);
        continue;
      }

      // Make sure sensor data is up to date after settling
      rclcpp::sleep_for(std::chrono::milliseconds(2000));

      // Get pose of the features
      if (!capture_manager.captureFeatures(robot_poses[pose_idx].features, msg))
      {
        RCLCPP_WARN(logger, "Failed to capture sample %u.", pose_idx);
        continue;
      }

      RCLCPP_INFO(logger, "Captured pose %u of %lu", pose_idx + 1, robot_poses.size());

      // Add to samples
      data.push_back(msg);
    }

    RCLCPP_INFO(logger, "Done capturing samples");
  }

	// Create instance of optimizer
  robot_calibration::OptimizationParams params;
  robot_calibration::Optimizer opt(description_msg.data);

  // Load calibration steps
  std::vector<std::string> calibration_steps =
      node_->declare_parameter<std::vector<std::string>>("calibration_steps", std::vector<std::string>());
  if (calibration_steps.empty())
  {
    RCLCPP_FATAL(logger, "Parameter calibration_steps is not defined");
  }

  // Run calibration steps
  for (auto step : calibration_steps)
  {
    params.LoadFromROS(node_, step);
    opt.optimize(params, data, logger, true);
    // opt.getOffsets()->writeToYAML("/home/abishalini/ws_studio/src/moveit_studio/moveit_studio_robot_config/kinova_gen3_lite_skills_center_config/calibration/camera_calibration.yaml");

		std::cout << "Parameter Offsets:" << std::endl;
		std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
  }

  // Write outputs
  robot_calibration::exportResults(opt, description_msg.data, data);

  RCLCPP_INFO(logger, "Done calibrating");
}
