#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/conditional_removal.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::OccupancyGrid map_topic_msg;
// minimum and maximum height
double thre_z_min = 0.3;
double thre_z_max = 2.0;
double map_resolution = 0.05;
int flag_pass_through = 0;
double thre_radius = 0.1;
// radius filter points threshold
int thres_point_count = 10;

pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_z(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_y(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_x(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// pass through filter
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in);
// radius filter
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
// convert to grid map data and publish
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_filters");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Rate loop_rate(1.0);

  private_nh.param("file_directory", file_directory, std::string("/home/"));

  private_nh.param("file_name", file_name, std::string("map"));

  pcd_file = file_directory + file_name + pcd_format;

  private_nh.param("thre_z_min", thre_z_min, 0.2);
  private_nh.param("thre_z_max", thre_z_max, 2.0);
  private_nh.param("map_resolution", map_resolution, 0.05);
  private_nh.param("map_topic_name", map_topic_name, std::string("map"));

  ros::Publisher map_topic_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

  // load .pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    PCL_ERROR("Couldn't read file: %s \n", pcd_file.c_str());
    return (-1);
  }

  std::cout << "Initial pointcloud number : " << pcd_cloud->points.size() << std::endl;

  // pass through filter
  PassThroughFilter(thre_z_min, thre_z_max, bool(pcd_cloud));
  // radius filter
  RadiusOutlierFilter(cloud_after_PassThrough_x, thre_radius, thres_point_count);
  // convert to grid map data and publish
  // SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg);
  SetMapTopicMsg(cloud_after_Radius, map_topic_msg);

  while (ros::ok()) {
    map_topic_pub.publish(map_topic_msg);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}

// filter pointcloud using pass through
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {

  // create filter_z
  pcl::PassThrough<pcl::PointXYZ> passthrough_z;
  // input pointcloud
  passthrough_z.setInputCloud(pcd_cloud);
  // set operation in z axis
  passthrough_z.setFilterFieldName("z");
  // set height range
  passthrough_z.setFilterLimits(thre_low, thre_high);
  // true : keep points out of range / false : keep points in the range
  passthrough_z.setFilterLimitsNegative(flag_in);
  // do filtering and save
  passthrough_z.filter(*cloud_after_PassThrough_z);

  // create filter_y
  pcl::PassThrough<pcl::PointXYZ> passthrough_y;
  // input pointcloud
  passthrough_y.setInputCloud(cloud_after_PassThrough_z);
  // set operation in y axis
  passthrough_y.setFilterFieldName("y");
  // set height range
  passthrough_y.setFilterLimits(-18, 10);
  // true : keep points out of range / false : keep points in the range
  passthrough_y.setFilterLimitsNegative(false);
  // do filtering and save
  passthrough_y.filter(*cloud_after_PassThrough_y);

  // create filter_x
  pcl::PassThrough<pcl::PointXYZ> passthrough_x;
  // input pointcloud
  passthrough_x.setInputCloud(cloud_after_PassThrough_y);
  // set operation in x axis
  passthrough_x.setFilterFieldName("x");
  // set height range
  passthrough_x.setFilterLimits(-10, 20);
  // true : keep points out of range / false : keep points in the range
  passthrough_x.setFilterLimitsNegative(false);
  // do filtering and save
  passthrough_x.filter(*cloud_after_PassThrough_x);

  // save to pcd file
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd",
                                      *cloud_after_PassThrough_x);
  std::cout << "pass through filter pointcloud : "
            << cloud_after_PassThrough_x->points.size() << std::endl;
}

// radius filter
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  // create filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  // define input pointcloud
  radiusoutlier.setInputCloud(pcd_cloud0);
  // set radius and find point in range
  radiusoutlier.setRadiusSearch(radius);
  // delete points if < threshold
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  // save to pcd file
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                      *cloud_after_Radius);
  std::cout << "radius filter pointcloud : " << cloud_after_Radius->points.size()
            << std::endl;
}

// convert to grid map data and publish
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg) {
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;

  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for (int i = 0; i < cloud->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // define origin position
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  // define grid map size
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  // point coord (x,y) in real map corresponding to grid map coord [x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for (int iter = 0; iter < cloud->points.size(); iter++) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // grid map's Occupancy posibility [0,100]
    msg.data[i + j * msg.info.width] = 100;
  }
}
