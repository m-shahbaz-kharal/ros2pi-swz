#include<rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace sensor_msgs::msg;
using namespace message_filters;
using namespace std;


std::string root_output_dir, lidar_output_dir, camera_output_dir;
bool debug;

// Synchronizer Callback, it will be called when camera and LiDAR data are in sync.
void callback(const Image::ConstPtr& image, const CameraInfo::ConstPtr& camara_info, const PointCloud2::ConstPtr& point_cloud)
{
  printf("%s\n", "All data in sync!" );

  // Convert ROS PointCloud2 to PCL point cloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*point_cloud, cloud);

  // Create a date string from the point cloud's timestamp to use in the file name of the saved data
  const int output_size = 100;
  char output[output_size];
  std::time_t raw_time = static_cast<time_t>(point_cloud->header.stamp.sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, "lidar_%Y_%m_%d_%H_%M_%S", timeinfo);

  // Creates a string containing the millisencods to be added to the previously created date string
  std::stringstream ss;
  ss << std::setw(9) << std::setfill('0') << point_cloud->header.stamp.nanosec;
  const size_t fractional_second_digits = 4;

  // Combine all of the pieces to get the output file name
  std::string output_file = lidar_output_dir + "/" + std::string(output) + "." + ss.str().substr(0, fractional_second_digits)+".pcd";

  // Save the point cloud as a PCD file
  printf("Output File: %s\n", output_file.c_str());
  pcl::io::savePCDFileASCII (output_file, cloud);
  printf("%s\n", output_file.c_str() );

  // Convert the ROS image to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  if (debug){
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }

  // Create the filename for the image
  output_file = camera_output_dir + "/" + std::string(output) + "." + ss.str().substr(0, fractional_second_digits)+".jpg";

  // Save the image
  cv::imwrite(output_file, cv_ptr->image);

}


int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lidar_and_cam_synchronizer");

  // Get parameters or use default values
  root_output_dir = node->declare_parameter<std::string>("root_output_dir", "/home/me/sync_out");
  bool debug = node->declare_parameter<bool>("debug", false);

  // Create subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node, "/flir_camera/image_raw");
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub(node, "/flir_camera/camera_info");
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub(node, "/lidar/points");

  // Create the synchronizer
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2>;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, camera_info_sub, point_cloud_sub);
  // bind to callback ros2
  sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create the folder for the current run. A new folder will be created each time the node runs
  char output[100];
  std::time_t raw_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  struct tm* timeinfo = std::localtime(&raw_time);
  std::strftime(output, 100, "run_%Y_%m_%d_%H_%M_%S", timeinfo);

  // Combine all of the pieces to get the output folders for this run
  lidar_output_dir = root_output_dir + "/" + std::string(output) + "/pcd";
  camera_output_dir = root_output_dir + "/" + std::string(output) + "/jpg";

  boost::filesystem::create_directories(lidar_output_dir);
  boost::filesystem::create_directories(camera_output_dir);

  RCLCPP_INFO(node->get_logger(), "lidar_output_dir: %s", lidar_output_dir.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
