#ifndef ELAS_RECONFIGURE_H
#define ELAS_RECONFIGURE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <elas_ros/reconfigureConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <elas_ros/ElasFrameData.h>
#include <elas.h>

typedef image_transport::SubscriberFilter Subscriber;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
typedef image_transport::Publisher Publisher;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

#endif
