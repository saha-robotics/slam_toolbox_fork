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
      "slam_toolbox/get_best_response",
      std::bind(&LocalizationSlamToolbox::getBestResponseCallback, this,
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
    // process_desired_pose_.reset(nullptr);
    update_reprocessing_transform = false;
    processor_type_ = PROCESS_LOCALIZATION;
  }else if (processor_type_ == PROCESS_LOCALIZATION) {
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance);
    update_reprocessing_transform = false;
  } 
  else {
    RCLCPP_FATAL(get_logger(), "LocalizationSlamToolbox: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }
  // if successfully processed, create odom to map transformation
  if (!processed) {
    delete range_scan;
    range_scan = nullptr;
  } 
  else {
    // compute our new transform
    setTransformFromPoses(range_scan->GetCorrectedPose(), odom_pose,
      scan->header.stamp, update_reprocessing_transform);

    publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }

  return range_scan;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::getBestResponseCallback(
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
  double loop_search_maximum_distance;
  double link_scan_maximum_distance;
  double minimum_best_response;
  double angle_resolution;
  double search_maximum_distance;
  double search_maximum_loop_closure_distance;
  double search_maximum_link_scan_distance;
  int process_timeout;
  this->get_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  this->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance);

  if (req->pose_x == 0.0 || req->pose_y == 0.0) {
      RCLCPP_ERROR(get_logger(), "Error: pose_x or pose_y is not provided.");
      res->message = "Error: pose_x or pose_y is missing. Please use it";
      res->success = false;
      return false;
  }
  if (req->minimum_best_response == 0.0) {
    RCLCPP_INFO(get_logger(), "minimum_best_response is not provided setting initial value to 0.5");
    minimum_best_response = (req->minimum_best_response != 0.0) ? req->minimum_best_response : 0.5;
  }
  if(req->angle_resolution == 0.0){
    RCLCPP_INFO(get_logger(), "angle_resolution is not provided setting initial value to 20.0");
    angle_resolution = (req->angle_resolution != 0.0) ? req->angle_resolution : 20.0;
  }  
  if(req->process_timeout == 0){
    RCLCPP_INFO(get_logger(), "process_timeout is not provided setting initial value to 10");
    process_timeout = (req->process_timeout != 0) ? req->process_timeout : 10;  
  }
  if(req->search_range == 0.0){
    RCLCPP_INFO(get_logger(), "search_maximum_distance is not provided setting initial value to yaml param");
    search_maximum_distance = req->search_range;
    this->set_parameter(rclcpp::Parameter("loop_search_maximum_distance", search_maximum_distance));
    this->set_parameter(rclcpp::Parameter("link_scan_maximum_distance", search_maximum_distance));
  }

  std::cout << "last_odom_pose_stored_ " << last_odom_pose_stored_ << std::endl;
  if (!have_scan_values_) {
    res->message = "No scan values stored try later";
    res->success = false;
    this->set_parameter(rclcpp::Parameter("loop_search_maximum_distance", loop_search_maximum_distance));
    this->set_parameter(rclcpp::Parameter("link_scan_maximum_distance", link_scan_maximum_distance));
    return false;
  }
  else{
      for (double angle = 0.0; angle <= 360.0; angle += angle_resolution) {
        bool processed = false;
        {    
          boost::mutex::scoped_lock l(pose_mutex_);
          process_desired_pose_ = std::make_unique<Pose2>(req->pose_x, req->pose_y, angle);
          range_scan = getLocalizedRangeScan(last_laser_stored_, last_scan_stored_, last_odom_pose_stored_);

          boost::mutex::scoped_lock lock(smapper_mutex_);
          range_scan->SetOdometricPose(*process_desired_pose_);
          range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
          processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);
        }
        double * best_response = smapper_->getMapper()->GetBestResponse();

        if (processed) {
          std::cout << "best_response " << *best_response << std::endl;
          std::cout << "req->minimum_best_response " << minimum_best_response << std::endl;
          if (best_response != nullptr && *best_response > minimum_best_response) {
              std::cout<< "finded best response" << std::endl;
              res->message = std::to_string(*best_response);  
              res->success = true;
              this->set_parameter(rclcpp::Parameter("loop_search_maximum_distance", loop_search_maximum_distance));
              this->set_parameter(rclcpp::Parameter("link_scan_maximum_distance", link_scan_maximum_distance));
              if (req->do_relocalize) {
                // compute our new transform
                setTransformFromPoses(range_scan->GetCorrectedPose(), last_odom_pose_stored_,
                  last_scan_stored_->header.stamp, true);

                publishPose(range_scan->GetCorrectedPose(), covariance, last_scan_stored_->header.stamp);
              }
              return true; 
          } else {
              res->message = "Couldn't find bestResponse";
              {
                boost::mutex::scoped_lock lock(smapper_mutex_);
                smapper_->clearLocalizationBuffer();
              }
          }
        }
        else {
          res->message = "Couldn't process scan will try again";
          end = std::chrono::high_resolution_clock::now();
          auto time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
          if (time_elapsed > process_timeout) {
              res->message = "Couldnt process scan in " + std::to_string(process_timeout) + " seconds";
              res->success = false;
              this->set_parameter(rclcpp::Parameter("loop_search_maximum_distance", loop_search_maximum_distance));
              this->set_parameter(rclcpp::Parameter("link_scan_maximum_distance", link_scan_maximum_distance));

              return false;
          }
        }

        end = std::chrono::high_resolution_clock::now();
        auto time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        if (time_elapsed > process_timeout) {
            res->message = "Couldnt find bestResponse in " + std::to_string(process_timeout) + " seconds";
            res->success = false;
              this->set_parameter(rclcpp::Parameter("loop_search_maximum_distance", loop_search_maximum_distance));
              this->set_parameter(rclcpp::Parameter("link_scan_maximum_distance", link_scan_maximum_distance));
            return false;
        }
      }
      res->message = "Couldn't find with this resolution at desired time, halving the angle_resolution and decreasing best_response then search again.";
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
