/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include <memory>
#include <string>
#include "slam_toolbox/slam_toolbox_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  processor_type_ = PROCESS_LOCALIZATION;
  localization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&LocalizationSlamToolbox::localizePoseCallback,
    this, std::placeholders::_1));
  clear_localization_ = this->create_service<std_srvs::srv::Empty>(
    "slam_toolbox/clear_localization_buffer",
    std::bind(&LocalizationSlamToolbox::clearLocalizationBuffer, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  ssGetBestResponse_ = this->create_service<slam_toolbox::srv::DesiredPoseChecker>(
      "slam_toolbox/desired_pose_check",
      std::bind(&LocalizationSlamToolbox::desiredPoseCheck, this,
      std::placeholders::_1, std::placeholders::_2));

  ssGetElevatorMode_ = this->create_service<slam_toolbox::srv::ElevatorMode>(
      "slam_toolbox/elevator_mode",
      std::bind(&LocalizationSlamToolbox::elevatorMode, this,
      std::placeholders::_1, std::placeholders::_2));

  // in localization mode, we cannot allow for interactive mode
  enable_interactive_mode_ = false;

  // in localization mode, disable map saver
  map_saver_.reset();
}

/*****************************************************************************/
void LocalizationSlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock)) {
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
    req->initial_pose = pose;
    req->filename = filename;
    req->match_type =
      slam_toolbox::srv::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;
    if (dock) {
      RCLCPP_WARN(get_logger(),
        "LocalizationSlamToolbox: Starting localization "
        "at first node (dock) is correctly not supported.");
    }

    deserializePoseGraphCallback(nullptr, req, resp);
  }
}

/*****************************************************************************/
bool LocalizationSlamToolbox::clearLocalizationBuffer(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);
  RCLCPP_INFO(get_logger(),
    "LocalizationSlamToolbox: Clearing localization buffer.");
  smapper_->clearLocalizationBuffer();
  return true;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::serializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  RCLCPP_ERROR(get_logger(), "LocalizationSlamToolbox: Cannot call serialize map "
    "in localization mode!");
  return false;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type != procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a non-localization deserialization "
      "in localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void LocalizationSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  RCLCPP_INFO(get_logger(), "LocalizationSlamToolbox: Processing scans.");
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (shouldProcessScan(scan, pose)) {
    addScan(laser, scan, pose);
  }
}

/*****************************************************************************/
LocalizedRangeScan * LocalizationSlamToolbox::addScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  // dirty storage:
  last_laser_stored_ = laser;
  last_scan_stored_ = scan;
  last_odom_pose_stored_ = odom_pose;
  have_scan_values_ = true;

  boost::mutex::scoped_lock l(pose_mutex_);

  if (processor_type_ == PROCESS_LOCALIZATION && process_near_pose_) {
    processor_type_ = PROCESS_NEAR_REGION;
  }
  if (processor_type_ == PROCESS_LOCALIZATION && process_desired_pose_) {
    processor_type_ = PROCESS_DESIRED_POSE;
  }

  LocalizedRangeScan * range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  if (processor_type_ == PROCESS_NEAR_REGION) {
    if (!process_near_pose_) {
      RCLCPP_ERROR(get_logger(),
        "Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }

    // set our position to the requested pose and proces
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);

    // reset to localization mode
    update_reprocessing_transform = true;
    processor_type_ = PROCESS_LOCALIZATION;
  }else if (processor_type_ == PROCESS_DESIRED_POSE) {
    process_desired_pose_.reset(nullptr);
    processed = true;
  }else if (processor_type_ == PROCESS_LOCALIZATION) {
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance);
    update_reprocessing_transform = false;
  } 
  else {
    RCLCPP_FATAL(get_logger(), "LocalizationSlamToolbox: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }

  if (!processed) {
    delete range_scan;
    range_scan = nullptr;
  } 
  else{ 
    if (processor_type_ != PROCESS_DESIRED_POSE) {
      setTransformFromPoses(range_scan->GetCorrectedPose(), odom_pose,
        scan->header.stamp, update_reprocessing_transform);

      publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
    }   
  }
  if (processor_type_ == PROCESS_DESIRED_POSE){
    processor_type_ = PROCESS_LOCALIZATION;
  }

  return range_scan;
}

void LocalizationSlamToolbox::setInitialParametersElevatorMode(double position_search_maximum_distance, double position_search_space_dimension_distance){
  std::cout << "localization_slam_toolbox: Setting elevator mode parameters" << std::endl;
  smapper_->getMapper()->setParamLoopSearchMaximumDistance(position_search_maximum_distance);
  smapper_->getMapper()->setParamLoopSearchSpaceDimension(position_search_space_dimension_distance);
  smapper_->getMapper()->GetGraph()->UpdateLoopScanMatcher(30.0);
}


void LocalizationSlamToolbox::setInitialParametersForDesiredPose(double position_search_distance, double position_search_maximum_distance, double position_search_fine_angle_offset,
                          double position_search_coarse_angle_offset, double position_search_coarse_angle_resolution, double position_search_resolution, 
                          double position_search_smear_deviation,bool do_loop_closing_flag){

  smapper_->getMapper()->setParamLoopSearchSpaceDimension(position_search_distance);
  smapper_->getMapper()->setParamLoopSearchMaximumDistance(position_search_maximum_distance);
  smapper_->getMapper()->setParamFineSearchAngleOffset(position_search_fine_angle_offset);
  smapper_->getMapper()->setParamCoarseSearchAngleOffset(position_search_coarse_angle_offset);
  smapper_->getMapper()->setParamCoarseAngleResolution(position_search_coarse_angle_resolution);
  smapper_->getMapper()->setParamLoopSearchSpaceResolution(position_search_resolution);
  smapper_->getMapper()->setParamLoopSearchSpaceSmearDeviation(position_search_smear_deviation);
  smapper_->getMapper()->setParamDoLoopClosing(do_loop_closing_flag);
  smapper_->getMapper()->m_Initialized = false;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::desiredPoseCheck(
    const std::shared_ptr<slam_toolbox::srv::DesiredPoseChecker::Request> req,
    std::shared_ptr<slam_toolbox::srv::DesiredPoseChecker::Response> res) 
/*****************************************************************************/
{
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  Matrix3 covariance;
  LocalizedRangeScan * range_scan = nullptr;
  covariance.SetToIdentity();
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    smapper_->clearLocalizationBuffer();
  }

  if (req->pose_x == 0.0 || req->pose_y == 0.0) {
      RCLCPP_ERROR(get_logger(), "Error: pose_x or pose_y is not provided or cannot be equal to <0.0>");
      res->message = "Error: pose_x or pose_y is missing. Please use it";
      res->success = false;
      return false;
  }

  if (req->do_relocalization){
    RCLCPP_INFO(get_logger(), "LocalizationSlamToolbox: Searching for best response with relocalize");
    position_search_do_relocalization_ = true;
  } else {
    RCLCPP_INFO(get_logger(), "LocalizationSlamToolbox: Searching for best response without relocalize");
    position_search_do_relocalization_ = false;
  }

  {
    boost::mutex::scoped_lock l(pose_mutex_);
    process_desired_pose_ = std::make_unique<Pose2>(req->pose_x, req->pose_y, 0.0);
  }

  first_measurement_ = true;

  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    setInitialParametersForDesiredPose(position_search_distance_,((position_search_distance_*0.5)-1), position_search_fine_angle_offset_,
                          position_search_coarse_angle_offset_, position_search_coarse_angle_resolution_, 
                          position_search_resolution_, position_search_smear_deviation_,true); 

    if (req->search_distance != 0.0) {
      position_search_distance_ = req->search_distance;
      setInitialParametersForDesiredPose(position_search_distance_,((position_search_distance_*0.5)-1), position_search_fine_angle_offset_,
                            position_search_coarse_angle_offset_, position_search_coarse_angle_resolution_, 
                            position_search_resolution_, position_search_smear_deviation_,true); 
    } 
  }

  if (!have_scan_values_) {
    res->message = "No scan values stored try later";             
    res->success = false;
    return false;
  }
  else{
        bool processed = false;
        {    
          boost::mutex::scoped_lock l(pose_mutex_);
          process_desired_pose_ = std::make_unique<Pose2>(req->pose_x, req->pose_y, 0.0);
          range_scan = getLocalizedRangeScan(last_laser_stored_, last_scan_stored_, last_odom_pose_stored_);

          boost::mutex::scoped_lock lock(smapper_mutex_);
          range_scan->SetOdometricPose(*process_desired_pose_);
          range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
          processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);
        
          if (processed) {
            std::shared_ptr<Mapper::LocalizationInfos> response = smapper_->getMapper()->GetBestResponse();
            double best_response = response->bestResponse;
            double best_pose_x = response->bestPoseX;
            double best_pose_y = response->bestPoseY;

            if (best_response > position_search_minimum_best_response_) {

                if (position_search_do_relocalization_) {
                  setTransformFromPoses(range_scan->GetCorrectedPose(), last_odom_pose_stored_,
                    last_scan_stored_->header.stamp, true);

                  publishPose(range_scan->GetCorrectedPose(), covariance, last_scan_stored_->header.stamp);
                }
                else {
                  range_scan->SetOdometricPose(last_odom_pose_stored_);
                  range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
                }

                setInitialParametersForDesiredPose(this->get_parameter("loop_search_space_dimension").as_double(),this->get_parameter("loop_search_maximum_distance").as_double(), 
                                                  this->get_parameter("fine_search_angle_offset").as_double(),this->get_parameter("coarse_search_angle_offset").as_double(),
                                                  this->get_parameter("coarse_angle_resolution").as_double(),this->get_parameter("loop_search_space_resolution").as_double(),
                                                  this->get_parameter("loop_search_space_smear_deviation").as_double(),this->get_parameter("do_loop_closing").as_bool());

                res->message = "Found bestResponse";
                res->relocated_x= static_cast<float>(best_pose_x);
                res->relocated_y= static_cast<float>(best_pose_y);
                res->best_response = static_cast<float>(best_response);

                res->success = true;
                return true; 
            } else {
                smapper_->clearLocalizationBuffer();
                setInitialParametersForDesiredPose(this->get_parameter("loop_search_space_dimension").as_double(),this->get_parameter("loop_search_maximum_distance").as_double(), 
                                                  this->get_parameter("fine_search_angle_offset").as_double(),this->get_parameter("coarse_search_angle_offset").as_double(),
                                                  this->get_parameter("coarse_angle_resolution").as_double(),this->get_parameter("loop_search_space_resolution").as_double(),
                                                  this->get_parameter("loop_search_space_smear_deviation").as_double(),this->get_parameter("do_loop_closing").as_bool());

                res->message = "Couldn't find bestResponse";
                res->success = false;
                return false;
            }
          }
        res->message = "Couldn't find with this resolution at desired time, halving the angle_resolution and decreasing best_response then search again.";
        res->success = false;
        return false;
        }
    }
}

bool LocalizationSlamToolbox::elevatorMode(
    const std::shared_ptr<slam_toolbox::srv::ElevatorMode::Request> req,
    std::shared_ptr<slam_toolbox::srv::ElevatorMode::Response> res) 
{
  if (req->inside_elev_zone){
    boost::mutex::scoped_lock lock(smapper_mutex_);
    RCLCPP_INFO(get_logger(), "LocalizationSlamToolbox: Elevator zone settings are enabled");
    double position_search_maximum_distance = 1.0;
    double position_search_space_dimension_distance = 1.0;
    setInitialParametersElevatorMode(position_search_maximum_distance, position_search_space_dimension_distance);
    res->message = "Elevator mode is enabled";
    res->success = true;
    return true;
  } else {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    RCLCPP_INFO(get_logger(), "LocalizationSlamToolbox: Elevator zone settings are disabled");
    setInitialParametersElevatorMode(this->get_parameter("loop_search_maximum_distance").as_double(),this->get_parameter("loop_search_space_dimension").as_double());
    res->message = "Elevator mode is disabled";
    res->success = true;
    return true;  
  }
}

/*****************************************************************************/
void LocalizationSlamToolbox::localizePoseCallback(
  const
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION) {
    RCLCPP_ERROR(get_logger(),
      "LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  boost::mutex::scoped_lock l(pose_mutex_);
  if (process_near_pose_) {
    process_near_pose_.reset(new Pose2(msg->pose.pose.position.x,
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)));
  } else {
    process_near_pose_ = std::make_unique<Pose2>(msg->pose.pose.position.x,
        msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  }

  first_measurement_ = true;

  boost::mutex::scoped_lock lock(smapper_mutex_);
  smapper_->clearLocalizationBuffer();

  RCLCPP_INFO(get_logger(),
    "LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::LocalizationSlamToolbox)
