//
// Created by jinxu on 2024/07/30.
//
#ifndef ESKF_IWO_H
#define ESKF_IWO_H

// ros
#include <ros/ros.h>

#include <eskf_iwo/eskf.hpp>
#include <eskf_iwo/static_imu_init.h>
#include <eskf_iwo/common/imu.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>

#include <thread>

namespace eskf_iwo
{
    #ifndef FILENAME
    #define FILENAME "/home/nvidia/depthaiTest_ws/src/imu_whell_ros_test/config/eskf_iwo.yaml"
    #endif

    class EskfIwo
    {
        public:
            EskfIwo(ros::NodeHandle& pnh);
            EskfIwo(const EskfIwo&) = delete;
            EskfIwo operator=(const EskfIwo&) = delete;

            // Destructor
            ~EskfIwo();

            bool initialize();

            typedef boost::shared_ptr<EskfIwo> Ptr;
            typedef boost::shared_ptr<const EskfIwo> ConstPtr;
            
        private:
            sad::IMU ConvertToSadIMU(const sensor_msgs::Imu& imu_msg);
            sad::Odom ConvertToSadOdom(const nav_msgs::Odometry& odom_msg);

            void batchImuProcessing(const double& time_bound);

            bool clearState();
            bool createRosIO();
            bool loadParameters();

            void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
            void wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg);

            void pubOdom(const sad::NavState<double>& odom);

            // ros
            ros::NodeHandle nh;
            ros::Subscriber sub_imu, sub_wheel;
            ros::Publisher pub_odometry;

            sad::IMU imu;
            sad::Odom odom;
            std::deque<sad::IMU> imu_deque_;

            // 初始化器
            sad::ESKFD eskf;
            // 全局标志
            bool imu_inited = false;
            sad::StaticIMUInit imu_init;  // 使用默认配置
            std::string IMU_TOPIC, WHEEL_TOPIC, ODOM_TOPIC, FRAME_ID;
    };

    typedef EskfIwo::Ptr EskfIwoPtr;
    typedef EskfIwo::ConstPtr EskfIwoConstPtr;
}

#endif // ESKF_IWO_H
