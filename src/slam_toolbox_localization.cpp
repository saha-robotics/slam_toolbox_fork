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

  set_parameters_srv_ = this->create_service<slam_toolbox::srv::SetParametersService>(
    "slam_toolbox/set_dynamic_parameters",
    std::bind(&LocalizationSlamToolbox::set_parameters_callback, this,
    std::placeholders::_1, std::placeholders::_2));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "slam_toolbox/table_vis", 10);

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

void LocalizationSlamToolbox::setInitialParameters(double position_search_distance, double position_search_maximum_distance, double position_search_fine_angle_offset,
                          double position_search_coarse_angle_offset, double position_search_coarse_angle_resolution, double position_search_resolution, 
                          double position_search_smear_deviation,bool do_loop_closing_flag,
                          int scan_buffer_size){

  smapper_->getMapper()->setParamLoopSearchSpaceDimension(position_search_distance);
  smapper_->getMapper()->setParamLoopSearchMaximumDistance(position_search_maximum_distance);
  smapper_->getMapper()->setParamFineSearchAngleOffset(position_search_fine_angle_offset);
  smapper_->getMapper()->setParamCoarseSearchAngleOffset(position_search_coarse_angle_offset);
  smapper_->getMapper()->setParamCoarseAngleResolution(position_search_coarse_angle_resolution);
  smapper_->getMapper()->setParamLoopSearchSpaceResolution(position_search_resolution);
  smapper_->getMapper()->setParamLoopSearchSpaceSmearDeviation(position_search_smear_deviation);
  smapper_->getMapper()->setParamDoLoopClosing(do_loop_closing_flag);
  smapper_->getMapper()->setParamScanBufferSize(scan_buffer_size);
  if(processor_type_ == PROCESS_LOCALIZATION){
    smapper_->getMapper()->m_Initialized = false;
  }
  else{
    smapper_->getMapper()->GetGraph()->UpdateLoopScanMatcher(this->get_parameter("max_laser_range").as_double());
  }
}

void LocalizationSlamToolbox::triggerTableSave(){
  if (processor_type_ == PROCESS){
    RCLCPP_INFO(get_logger(), "Triggering table save function");
    boost::mutex::scoped_lock lock(smapper_mutex_);
    smapper_->getMapper()->StartTableStorage(true);
    // but its get the last data.
    getSavedTableData();
  }
}

void LocalizationSlamToolbox::getSavedTableData() {
  if (processor_type_ == PROCESS) {
    RCLCPP_INFO(get_logger(), "Getting saved table data");
    std::cout << "Pose vector size: " << smapper_->getMapper()->poseVector.size() << std::endl;

    if (smapper_->getMapper()->poseVector.size() > 0) {
      const std::vector<karto::Mapper::TablePose>& localPoseVector = smapper_->getMapper()->poseVector;
      visualization_msgs::msg::MarkerArray marker_array;

      for (const auto& pose : localPoseVector) {
        std::cout << "Pose ID: " << pose.scanId
                  << ", X: " << pose.x
                  << ", Y: " << pose.y
                  << ", Yaw: " << pose.yaw << std::endl;

        // Create a sphere marker for the pose
        visualization_msgs::msg::Marker sphere_marker =
            vis_utils::toSphereMarker(
                "map",                   // Frame ID
                "pose_visualization",    // Namespace
                pose.x, pose.y, pose.yaw, // Position (yaw not visualized in a sphere)
                0.2,                     // Scale (sphere radius)
                {0.0, 1.0, 0.0, 1.0},    // Color: green (RGBA)
                shared_from_this());     // Node pointer

        sphere_marker.id = pose.scanId; // Assign unique ID to avoid marker conflicts
        marker_array.markers.push_back(sphere_marker);
      }

      marker_pub_->publish(marker_array);
    }
  }
}

void LocalizationSlamToolbox::set_parameters_callback(
    const std::shared_ptr<slam_toolbox::srv::SetParametersService::Request> request,
    std::shared_ptr<slam_toolbox::srv::SetParametersService::Response> response
) 
{
    if(request->table_save_mode){
      triggerTableSave();
      RCLCPP_INFO(get_logger(), "Triggering table save service");
    }

    if (request->default_mode) {
        RCLCPP_INFO(get_logger(), "Setting parameters for the default mode");
        boost::mutex::scoped_lock lock(smapper_mutex_);
        setInitialParameters(
            this->get_parameter("loop_search_space_dimension").as_double(),
            this->get_parameter("loop_search_maximum_distance").as_double(), 
            this->get_parameter("fine_search_angle_offset").as_double(),
            this->get_parameter("coarse_search_angle_offset").as_double(),
            this->get_parameter("coarse_angle_resolution").as_double(),
            this->get_parameter("loop_search_space_resolution").as_double(),
            this->get_parameter("loop_search_space_smear_deviation").as_double(),
            this->get_parameter("do_loop_closing").as_bool(),
            this->get_parameter("scan_buffer_size").as_int()
        );

        response->success = true;
        response->message = "Default parameters set successfully.";
        return;
    } 

    if (request->param_names.size() != request->param_values.size()) {
        response->success = false;
        response->message = "Mismatched param_names and param_values lengths.";
        return;
    }

    boost::mutex::scoped_lock lock(smapper_mutex_);
    // main purpose is resetting all the parameters first call. then initialize.
    setInitialParameters(
        this->get_parameter("loop_search_space_dimension").as_double(),
        this->get_parameter("loop_search_maximum_distance").as_double(), 
        this->get_parameter("fine_search_angle_offset").as_double(),
        this->get_parameter("coarse_search_angle_offset").as_double(),
        this->get_parameter("coarse_angle_resolution").as_double(),
        this->get_parameter("loop_search_space_resolution").as_double(),
        this->get_parameter("loop_search_space_smear_deviation").as_double(),
        this->get_parameter("do_loop_closing").as_bool(),
        this->get_parameter("scan_buffer_size").as_int()
    );

    for (size_t i = 0; i < request->param_names.size(); ++i) {
        const auto &param_name = request->param_names[i];
        const auto &param_value = request->param_values[i];

        if (param_name == "loop_search_maximum_distance") {
            smapper_->getMapper()->setParamLoopSearchMaximumDistance(param_value);
        } 
        else if (param_name == "loop_search_space_dimension") {
            smapper_->getMapper()->setParamLoopSearchSpaceDimension(param_value);
        } 
        else if (param_name == "loop_search_space_resolution"){
            smapper_->getMapper()->setParamLoopSearchSpaceResolution(param_value);
        }
        else if (param_name == "loop_search_space_smear_deviation"){
            smapper_->getMapper()->setParamLoopSearchSpaceSmearDeviation(param_value);
        }
        else if (param_name == "loop_search_maximum_distance"){
            smapper_->getMapper()->setParamLoopSearchMaximumDistance(param_value);
        }
        else if (param_name == "correlation_search_space_dimension"){
            smapper_->getMapper()->setParamCorrelationSearchSpaceDimension(param_value);
        }
        else if (param_name == "scan_buffer_size"){
            int value =static_cast<int>(param_value);
            smapper_->getMapper()->setParamScanBufferSize(value);        
        }
        else {
            RCLCPP_ERROR(get_logger(),
              "Parameter not recognized. Returning false");
        }
    }

    if(processor_type_ == PROCESS_LOCALIZATION){
      smapper_->getMapper()->m_Initialized = false;
    }
    else{
      smapper_->getMapper()->GetGraph()->UpdateLoopScanMatcher(this->get_parameter("max_laser_range").as_double());
    }
    response->success = true;
    response->message = "Parameters set successfully for the specified mode";
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
