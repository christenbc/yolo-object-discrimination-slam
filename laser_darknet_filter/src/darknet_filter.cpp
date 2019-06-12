/*
 * file_name
 * Copyright (c) 2019, Christen Blom-Dahl
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

/* Author: Christen Blom-Dahl */
/* Modified by: Christen Blom-Dahl*/


#include <ros/ros.h>
#include <image_transport/camera_common.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <laser_geometry/laser_geometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

// Activate this define to publish the image_out topic with debug imformation
//#define PUBLISH_IMAGE

// Activate this define to use the debug topic names
//#define USE_DEBUG_TOPICS

#if defined PUBLISH_IMAGE
  #include <image_transport/image_transport.h>
  #include <sensor_msgs/Image.h>
#endif

using namespace sensor_msgs;
using namespace message_filters;

class LaserScanDarknetFilter
{
private:
  bool up_and_running_;
  ros::NodeHandle nh_;
#if defined PUBLISH_IMAGE
  image_transport::ImageTransport it_;
  Subscriber<Image> image_sub_;
  image_transport::Publisher image_pub_;
#endif

  Subscriber<CameraInfo> info_sub_;

  Subscriber<LaserScan> scan_sub_;
  ros::Publisher scan_nav_pub_;
  ros::Publisher scan_struct_pub_;
  ros::Publisher scan_nonstruct_pub_;

  Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub_;

  typedef sync_policies::ApproximateTime<
#if defined PUBLISH_IMAGE
    Image,
#endif
    CameraInfo, LaserScan, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  laser_geometry::LaserProjection projector_;
  
  enum pointClass {ptOutOfImage = 0, ptStructural = 1, ptNonStructural = 2};
  
  // Get the classification of point coordinates. Possible return values are:
  //   ptOutOfImage: Point coordinates are outside of the image layout.
  //   ptStructural: Point coordinates are outside of any detected bouding box.
  //   ptNonStructural: Poin coordinates are inside of any of the detected bounding boxes.
  pointClass getPointClass(const cv::Point2d& point, 
                           unsigned imgWidth, unsigned imgHeight,
                           const darknet_ros_msgs::BoundingBoxes& bounding_boxes)
  {
    if (point.x < 0 || point.x >= imgWidth || point.y < 0 || point.y >= imgHeight) {
        // The point lays out of the image
        return ptOutOfImage;
    }
    else {
        // Iterate throug all the detected bounding boxes until the coordinates lay inside any of them
        // or we get to the end of the list
        for (unsigned idx = 0; idx < bounding_boxes.bounding_boxes.size(); idx++) {
            const darknet_ros_msgs::BoundingBox &boundingBox = bounding_boxes.bounding_boxes.at(idx);
            if (point.x >= boundingBox.xmin && point.x <= boundingBox.xmax && point.y >= boundingBox.ymin && point.y <= boundingBox.ymax) {
                // The poin lays inside this bounding box
                return ptNonStructural;
            }
        }
      // The point is outside of all the detected bounding boxes
      return ptStructural;
    }
  }

public:
  LaserScanDarknetFilter()
#if defined PUBLISH_IMAGE
    : it_(nh_)
#endif
  {
    up_and_running_ = true;
#ifndef USE_DEBUG_TOPICS
    // Assign topic names
    std::string image_topic = nh_.resolveName("image");
    std::string scan_topic = nh_.resolveName("laser_scan");
    std::string bounding_boxes_topic = nh_.resolveName("bounding_boxes");
#else
    // Assign debug topic names
    std::string image_topic = nh_.resolveName("/camera_floor/driver/color/image_raw");
    std::string scan_topic = nh_.resolveName("/f_raw_scan");
    std::string bounding_boxes_topic = nh_.resolveName("/darknet_ros/bounding_boxes");
#endif
    // Calculate camera_info topic name from image topic name.
    std::string info_topic = image_transport::getCameraInfoTopic(image_topic);
#if defined PUBLISH_IMAGE
    image_sub_.subscribe(nh_, image_topic, 1);
#endif
    // Subscribe to topics
    info_sub_.subscribe(nh_, info_topic, 1);
    scan_sub_.subscribe(nh_, scan_topic, 1);
    bounding_boxes_sub_.subscribe(nh_, bounding_boxes_topic, 1);
    // We use an sync_policies::ApproximateTime message sincronicity because different topics do not have
    // exact time in their data.
    sync_.reset(new Sync(MySyncPolicy(80), // You can change the queue size to another value if needed
#if defined PUBLISH_IMAGE
                image_sub_,
#endif
                info_sub_, scan_sub_, bounding_boxes_sub_));
    // Each time we get messages synchronized for all the topics, the LaserScanDarknetFilter::callback
    // method will be called with all the messages
    sync_->registerCallback(boost::bind(&LaserScanDarknetFilter::callback,
                                        this, _1, _2, _3
#if defined PUBLISH_IMAGE
                                        , _4
#endif
                                        ));
#if defined PUBLISH_IMAGE
    image_pub_ = it_.advertise("image_out", 1);
#endif
    // LaserScan with points inside the camera view
    scan_nav_pub_ = nh_.advertise<LaserScan>("navscan_scan", 1000);
    // LaserScan with points that do not lay in any detected bounding box
    scan_struct_pub_ = nh_.advertise<LaserScan>("struct_scan", 1000);
    // LaserScan with points thay lay in any detected bounding box
    scan_nonstruct_pub_ = nh_.advertise<LaserScan>("nonstruct_scan", 1000);
  }

  void callback(
#if defined PUBLISH_IMAGE
                const ImageConstPtr& image_msg,
#endif
                const CameraInfoConstPtr& info_msg,
                const LaserScanConstPtr& scan_msg,
                const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg)
  {
#if defined PUBLISH_IMAGE
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, image_encodings::BGR8);
      // Image contains the data from the image message
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[laser_scan_filter] Failed to convert image\n%s", ex.what());
      return;
    }
#endif

    // Get the model for the camera
    cam_model_.fromCameraInfo(info_msg);
	
    // Copy the scan_msg data into the three published LaserScan topics
    LaserScan scan_nav(*scan_msg);
    LaserScan scan_struct(*scan_msg);
    LaserScan scan_nonstruct(*scan_msg);
	
    std::string error_msg;
    // Retrieve the transformation from the LaserScan frame to the camera frame
    bool success = tf_listener_.waitForTransform(
                cam_model_.tfFrame(),
                scan_msg->header.frame_id,
                scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() * double(scan_msg->time_increment)),
                ros::Duration(1.0 / 30),
                ros::Duration(0.01),
                &error_msg);
    if (!success) {
      ROS_WARN("[laser_scan_filter] TF exception:\n%s", error_msg.c_str());
      return;
    }
    sensor_msgs::PointCloud2 laser_cloud;
    try {
      // Create a PointCloud2 object with points in the camera frame
      projector_.transformLaserScanToPointCloud(cam_model_.tfFrame(), *scan_msg, laser_cloud, tf_listener_);
    }
    catch(tf::TransformException& ex) {
      if (up_and_running_) {
        ROS_WARN_THROTTLE(1, "[laser_scan_filter] Tansform unavailable %s", ex.what());
        return;
      }
      else {
        ROS_INFO_THROTTLE(.3, "[laser_scan_filter] Ignoring Scan: Waiting for TF");
      }
      return;
    }
    const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
    const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
    const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
    const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");
    if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1) {
      ROS_INFO_THROTTLE(.3, "[laser_scan_filter] x, y, z and index fields are required, skipping scan");
      return;
    }
    const unsigned i_idx_offset = laser_cloud.fields[unsigned(i_idx_c)].offset;
    const unsigned x_idx_offset = laser_cloud.fields[unsigned(x_idx_c)].offset;
    const unsigned y_idx_offset = laser_cloud.fields[unsigned(y_idx_c)].offset;
    const unsigned z_idx_offset = laser_cloud.fields[unsigned(z_idx_c)].offset;
	
    const unsigned pstep = laser_cloud.point_step;
    const long int pcount = laser_cloud.width * laser_cloud.height;
    const long int limit = pstep * pcount;
	
    unsigned imgWidth = info_msg->width;
    unsigned imgHeight = info_msg->height;
    unsigned i_idx, x_idx, y_idx, z_idx;
    // Iterate through all the points in the cloud
    for(
      i_idx = i_idx_offset,
      x_idx = x_idx_offset,
      y_idx = y_idx_offset,
      z_idx = z_idx_offset;

      x_idx < limit;

      i_idx += pstep,
      x_idx += pstep,
      y_idx += pstep,
      z_idx += pstep)
    {
      // Get the x, y, z coordinates of the current point and the index in the LaserScan message
      float x = *((float*)(&laser_cloud.data[x_idx]));
      float y = *((float*)(&laser_cloud.data[y_idx]));
      float z = *((float*)(&laser_cloud.data[z_idx]));
      int index = *((int*)(&laser_cloud.data[i_idx]));

      if (z > 0) { // If the point lays in front of the camera
        cv::Point3d pt_cv((double(x)), (double(y)), (double(z)));
        cv::Point3d pt_cv2((double(x)) + 0.01, (double(y)), (double(z)));
        cv::Point2d uv;
        // Project the 3D point into the 2D space of the camera image
        uv = cam_model_.project3dToPixel(pt_cv);
        // Get the layout class of the point coordinates
        pointClass pt_class = getPointClass(uv, imgWidth, imgHeight, *bounding_boxes_msg);
#if defined PUBLISH_IMAGE
        cv::Scalar color;
#endif
        switch (pt_class) {
          case ptOutOfImage:
            // Clear the points of all the LaserScans
            scan_nav.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            scan_struct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            scan_nonstruct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            break;
          case ptStructural:
            // Clear the corresponding point of the non structural LaserScan
            scan_nonstruct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
#if defined PUBLISH_IMAGE
            color = CV_RGB(0,255,0);
#endif
            break;
          case ptNonStructural:
            // Clear the corresponding point of the structural LaserScan
            scan_struct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
#if defined PUBLISH_IMAGE
            color = CV_RGB(255,0,0);
#endif
            break;
        }
#if defined PUBLISH_IMAGE
        if (pt_class != ptOutOfImage) {
          cv::Point2d uv2;
          uv2 = cam_model_.project3dToPixel(pt_cv2);
          int radius = abs(int(uv.x - uv2.x));
          if (radius < 1) {
            radius = 1;
          }
          cv::circle(image, uv, radius, color, -1);
        }
#endif
      }
      else {
        // If the z coordinate is negative the point lays in the back of the camera
        // then we clear all the LaserScan points since the camera can oly see points
        // in positive z coordinates.
        scan_nav.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
        scan_struct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
        scan_nonstruct.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
      }
    }

#if defined PUBLISH_IMAGE
    image_pub_.publish(input_bridge->toImageMsg());
#endif
    // Publish all LaserScan messages
    scan_nav_pub_.publish(scan_nav);
    scan_struct_pub_.publish(scan_struct);
    scan_nonstruct_pub_.publish(scan_nonstruct);
    up_and_running_ = true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  LaserScanDarknetFilter filter;
  ros::spin();
}
