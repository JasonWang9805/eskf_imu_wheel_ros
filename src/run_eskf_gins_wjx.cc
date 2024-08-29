//
// Created by jinxu on 2024/07/30.
//
#include <eskf_iwo/run_eskf_gins_wjx.h>

namespace eskf_iwo
{
    EskfIwo::EskfIwo(ros::NodeHandle& pnh):
    // is_gravity_set(false),
    // is_first_img(true),
    nh(pnh) 
    {
        google::InitGoogleLogging("iwo");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;

        return;
    }

    EskfIwo::~EskfIwo()
    {
        google::FlushLogFiles(google::INFO);
        google::ShutdownGoogleLogging();

        return;
    }

    bool EskfIwo::loadParameters()
    {
        auto iwoConfig = YAML::LoadFile(FILENAME);
        LOG(INFO)<< " read parameters path is : "<< FILENAME<< std::endl;
        try
        {
            IMU_TOPIC = iwoConfig["iwo_sub_topics"]["imu_topic"].as<std::string>("/imu");
            WHEEL_TOPIC = iwoConfig["iwo_sub_topics"]["wheel_topic"].as<std::string>("/wheel");

            ODOM_TOPIC = iwoConfig["iwo_pub_topics"]["odom_topic"].as<std::string>("/imu/odom");

            FRAME_ID = iwoConfig["iwo_frame_topics"]["frame_id"].as<std::string>("/world");

        } catch (...) 
        {
            LOG(ERROR)<< "bad conversion. "<< std::endl;
            return false;
        }
        
        return true;
    }

    bool EskfIwo::createRosIO()
    {
        sub_imu = nh.subscribe(IMU_TOPIC, 2000, &EskfIwo::imu_callback, this, ros::TransportHints().tcpNoDelay());
        sub_wheel = nh.subscribe(WHEEL_TOPIC, 2000, &EskfIwo::wheel_callback, this, ros::TransportHints().tcpNoDelay());

        pub_odometry = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1000);

        // TODO:service

        return true;
    }

    bool EskfIwo::initialize() 
    {
        if (!loadParameters()) return false;
        ROS_INFO("Finish loading ROS parameters...");

        if (!createRosIO()) return false;
        ROS_INFO("Finish creating ROS IO...");

        return true;
    }

    bool EskfIwo::clearState()
    {
        imu_inited = false;
        return true;
    }

    sad::IMU EskfIwo::ConvertToSadIMU(const sensor_msgs::Imu& imu_msg)
    {
        sad::IMU imu;

        // 赋值陀螺仪数据
        imu.gyro_ = Eigen::Vector3d(
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        );

        // 赋值加速度计数据
        imu.acce_ = Eigen::Vector3d(
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        );

        imu.timestamp_ = imu_msg.header.stamp.toSec();

        return imu;
    }

    sad::Odom EskfIwo::ConvertToSadOdom(const nav_msgs::Odometry& odom_msg) 
    {
        sad::Odom odom;

        // 赋值时间戳信息
        odom.timestamp_ = odom_msg.header.stamp.toSec();
        // 赋值车速信息
        odom.averageSpeed_ = odom_msg.twist.twist.linear.x;
        // 赋值车角速度信息
        odom.averageAngularVelocity_ = odom_msg.twist.twist.angular.x;

        return odom;
    }

    // 获取两帧之间的 IMU 数据
    void EskfIwo::batchImuProcessing(const double& time_bound)
    {
        int used_imu_cntr = 0;

        for (const auto& imu_ : imu_deque_) 
        {
            double imu_time_ = imu_.timestamp_;
            // TODO:检测之前的时间戳
            if (imu_time_ < eskf.current_time_)
            {
                ++used_imu_cntr;
                continue;
            }
            if (imu_time_ > time_bound) break;

            eskf.Predict(imu_);
            ++used_imu_cntr;
        }

        // Set the state ID for the new IMU state.
        // 设置 IMU 新状态的状态 ID。
        // state_server.imu_state.id = IMUState::next_id++;

        // Remove all used IMU msgs.
        // 删除容器中使用过的 IMU 消息。
        imu_deque_.erase(imu_deque_.begin(), imu_deque_.begin()+used_imu_cntr);
    }

    void EskfIwo::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        // 转成 sad::imu
        sad::IMU imu = ConvertToSadIMU(*imu_msg);

        // test
        // LOG(INFO)<< "imu recive data ..."<< std::endl;
        
        if (!imu_init.InitSuccess())
        {
            // 添加 imu data
            imu_init.AddIMU(imu);
            return;
        }
        
        if(!imu_inited)
        {
            sad::ESKFD::Options options;
            options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
            options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
            eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
            imu_inited = true;
        }

        if(imu_inited) imu_deque_.push_back(imu);

        return;
    }

    void EskfIwo::wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        sad::Odom odom = ConvertToSadOdom(*odom_msg);
        
        if (!imu_init.InitSuccess() || !imu_inited) 
        {
            imu_init.AddOdom(odom);
            return;
        }

        // 利用时间戳信息获取想要的信息
        batchImuProcessing(odom.timestamp_);

        auto statePredict = eskf.GetNominalState();
        
        LOG(INFO)<< "imu odom: "<< statePredict<< std::endl;

        if (imu_inited) 
        {
            // 增添 odom 观测信息
            eskf.ObserveWheelSpeed(odom);
        }
        auto stateObserve = eskf.GetNominalState();

        pubOdom(stateObserve);

        LOG(INFO)<< "wheel odom: "<< stateObserve<< std::endl;

        return;
    }

    void EskfIwo::pubOdom(const sad::NavState<double>& stateObserve)
    {
        nav_msgs::Odometry odomMsg;

        // 时间戳
        odomMsg.header.stamp = ros::Time(stateObserve.timestamp_);
        odomMsg.header.frame_id = FRAME_ID;  // 设置适当的 frame_id

        // 位置（平移）
        odomMsg.pose.pose.position.x = stateObserve.p_.x();
        odomMsg.pose.pose.position.y = stateObserve.p_.y();
        odomMsg.pose.pose.position.z = stateObserve.p_.z();

        // 旋转（四元数）
        tf2::Quaternion quat;
        Eigen::Quaterniond eigenQuat(stateObserve.R_.unit_quaternion().w(), 
                                    stateObserve.R_.unit_quaternion().x(), 
                                    stateObserve.R_.unit_quaternion().y(), 
                                    stateObserve.R_.unit_quaternion().z());
        quat.setX(eigenQuat.x());
        quat.setY(eigenQuat.y());
        quat.setZ(eigenQuat.z());
        quat.setW(eigenQuat.w());
        odomMsg.pose.pose.orientation.x = quat.x();
        odomMsg.pose.pose.orientation.y = quat.y();
        odomMsg.pose.pose.orientation.z = quat.z();
        odomMsg.pose.pose.orientation.w = quat.w();

        // 线速度
        // odomMsg.twist.twist.linear.x = stateObserve.v_.x();
        // odomMsg.twist.twist.linear.y = stateObserve.v_.y();
        // odomMsg.twist.twist.linear.z = stateObserve.v_.z();

        // 角速度（如果有的话，假设可以从其他地方获得角速度）
        // 这里用零填充作为示例
        odomMsg.twist.twist.angular.x = 0.0;
        odomMsg.twist.twist.angular.y = 0.0;
        odomMsg.twist.twist.angular.z = 0.0;

        pub_odometry.publish(odomMsg);
    }

}