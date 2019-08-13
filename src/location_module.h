#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "csm/grid/probability_grid.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

// Allows AMCL to run from bag file
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

class location_module
{
public:
	location_module();
	~location_module();

private:
	tf::TransformBroadcaster* tfb_;

	void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);

	void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
	map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );

	Maplimits* map_limits;

	ProbabilityGrid* grid;

	// Minimum linear/angular search window in which the best possible scan alignment
	// will be found.
	double linear_search_window_;
	double angular_search_window_;

	// Weights applied to each part of the score.
	double translation_delta_cost_weight_;
	double rotation_delta_cost_weight_;

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher pose_pub_;

	ros::Subscriber initial_pose_sub_old_;
	ros::Subscriber map_sub_;

	message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
	ros::Subscriber initial_pose_sub_;

	RealTimeCorrelativeScanMatcher2D* scan_matcher_;();

};

location_module::location_module():
	map_limits(NULL),
	private_nh_("~"),
	first_map_received_(false),
	first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  private_nh_.param("linear_search_window", linear_search_window_, 1);
  private_nh_.param("angular_search_window", angular_search_window_, 2);
  private_nh_.param("translation_delta_cost_weight", translation_delta_cost_weight_, 3);
  private_nh_.param("rotation_delta_cost_weight", rotation_delta_cost_weight_, 3);
  
  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  tfb_.reset(new tf2_ros::TransformBroadcaster());
  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ = 
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             nh_);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived, this, _1));

  map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);

  scan_matcher_ = new RealTimeCorrelativeScanMatcher2D(linear_search_window,
		                                                   angular_search_window,
		                                                   translation_delta_cost_weight,
		                                                   rotation_delta_cost_weight);

}

location_module::~location_module()
{
  delete dsrv_;
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;

  if(grid != NULL)
  	delete grid;

  if(map_limits != NULL)
  	delete map_limits;
  // TODO: delete everything allocated in constructor
}

void
location_module::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
location_module::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);
  
  if(msg.header.frame_id != global_frame_id_)
    ROS_WARN("Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could cause issues with reading published topics",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());

  //freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  grid = convertMap(msg);

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();

}

ProbabilityGrid*
location_module::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  
  // ROS_ASSERT(map);

  CellLimits cell_limits(map_msg.info.width, map_msg.info.height);

  double resolution = map_msg.info.resolution;

  double origin_x = map_msg.info.origin.position.x + (map->size_x) * map->scale;
  double origin_y = map_msg.info.origin.position.y + (map->size_y) * map->scale;

  Eigen::Vector2d max(origin_x, origin_y);

  map_limits = new Maplimits(resolution, max, cell_limits);

  grid = new ProbabilityGrid(*map_limits);


  // Convert to player format

  for(int i = 0; i < map->size_y; i++){
  	for(int j = 0; j < map->size_x; j++)
  	{
  		int map_index = i * map->size_x + j;
  		int grid_x = map->size_x - j;
  		int grid_y = map->size_y - i;
  		int grid_index = grid_y * map->size_x + grid_x;

  	if(map_msg.data[map_index] == 100)
      grid->*lookupTable_[grid_index] = 1;
    else
      grid->*lookupTable_[grid_index] = 0;
  	}
  }
  	

  return grid;
}

//handle laser_scan message

// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = ros::Time::now();
  if( grid == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan_frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = ros::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    geometry_msgs::PoseStamped laser_pose;
    try
    {
      this->tf_->transform(ident, laser_pose, base_frame_id_);
    }
    catch(tf2::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan_frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    Eigen::Vector3f laser_pose_v;
    laser_pose_v.x = laser_pose.pose.position.x;
    laser_pose_v.y = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.z = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan_frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan_frame_id];
  }

  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
      }
    }
    angle += msg.angle_increment;
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  
}