#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include "livox_ros_driver/CustomMsg.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <queue>
#include <assert.h>
#include <memory>

struct RecordDatabase {
    using Ptr = std::shared_ptr<RecordDatabase>;
    RecordDatabase(const std::string &save_path) {
        record_oi.open(save_path.c_str(), std::ios::trunc);
    }

    void record(const double &timestamp, const std::string &sensor_id, const std::string &desc = "") {
        record_mtx.lock();
        record_oi << std::fixed << std::setprecision(9) << timestamp << " " << sensor_id << " " << desc << std::endl;
        record_mtx.unlock();
    }

    std::ofstream record_oi;
    std::mutex record_mtx;
};

RecordDatabase::Ptr record_database;
ros::Publisher pub_lidar;
ros::Publisher pub_camera;
ros::Publisher pub_imu;

void livoxLidarHandler(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
    pcl::PointCloud<pcl::PointXYZINormal> pcl_in;
    auto time_end = livox_msg_in->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = livox_msg_in->points[i].x;
        pt.y = livox_msg_in->points[i].y;
        pt.z = livox_msg_in->points[i].z;
        float s = float(livox_msg_in->points[i].offset_time / (float)time_end);
        pt.intensity = 0.1 * livox_msg_in->points[i].reflectivity;
        pt.curvature = livox_msg_in->points[i].line + s*0.1;// integer part: line number; decimal part: timestamp
        pcl_in.push_back(pt);
    }

    /// timebase 5ms ~ 50000000, so 10 ~ 1ns

    ros::Time timestamp(livox_msg_in->header.stamp.toSec());

    record_database->record(livox_msg_in->header.stamp.toSec(), "livox_lidar");

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp = timestamp;
    std::string frame_id = "livox_lidar_frame";
    pcl_ros_msg.header.frame_id = frame_id;

    pub_lidar.publish(pcl_ros_msg);
}

void livoxImuHandler(const sensor_msgs::Imu::ConstPtr& imu_msg_in) {
    sensor_msgs::Imu::Ptr imu_msg_out(new sensor_msgs::Imu);
    imu_msg_out->header = imu_msg_in->header;
    imu_msg_out->angular_velocity = imu_msg_in->angular_velocity;
    imu_msg_out->linear_acceleration = imu_msg_in->linear_acceleration;

    imu_msg_out->orientation = imu_msg_in->orientation;
    imu_msg_out->orientation_covariance = imu_msg_in->orientation_covariance;
    imu_msg_out->angular_velocity_covariance = imu_msg_in->angular_velocity_covariance;
    imu_msg_out->linear_acceleration_covariance = imu_msg_in->linear_acceleration_covariance;
    
    record_database->record(imu_msg_out->header.stamp.toSec(), "livox_imu");

    pub_imu.publish(imu_msg_out);
}

void BaslerCameraHandler(const sensor_msgs::Image::ConstPtr& image_msg_in) {
    sensor_msgs::Image::Ptr image_msg_out(new sensor_msgs::Image);
    image_msg_out->header = image_msg_in->header;
    image_msg_out->height = image_msg_in->height;
    image_msg_out->width = image_msg_in->width;
    image_msg_out->is_bigendian = image_msg_in->is_bigendian;
    image_msg_out->encoding = image_msg_in->encoding;
    image_msg_out->step = image_msg_in->step;
    image_msg_out->data = image_msg_in->data;

    // cv_bridge::CvImageConstPtr bayer_ptr = cv_bridge::toCvCopy(image_msg_in, sensor_msgs::image_encodings::BAYER_RGGB8);
    // cv::Mat bayer_img = bayer_ptr->image.clone();
    // cv::Mat bgr_img; cvtColor(bayer_img, bgr_img, CV_BayerRG2BGR);
    // sensor_msgs::Image::Ptr image_msg_out = cv_bridge::CvImage(image_msg_in->header, "bgr8", bgr_img).toImageMsg();

    record_database->record(image_msg_out->header.stamp.toSec(), "basler_image");

    pub_camera.publish(image_msg_out);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "recordor_and_convertor");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033[0m Format Convert Started.");

    record_database.reset(new RecordDatabase("/shared/ws_sensors/sensor_record.txt"));

    ros::Subscriber sub_livox_lidar = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, livoxLidarHandler);
    pub_lidar = nh.advertise<sensor_msgs::PointCloud2>("/livox_lidar", 100);

    ros::Subscriber sub_livox_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 100, livoxImuHandler);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/livox_imu", 100);

    ros::Subscriber sub_basler_camera = nh.subscribe<sensor_msgs::Image>("/pylon_camera_publisher/image_raw", 100, BaslerCameraHandler);
    pub_camera = nh.advertise<sensor_msgs::Image>("/basler_camera", 100);

    ros::spin();
}