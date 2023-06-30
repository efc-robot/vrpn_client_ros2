#include "vrpn_listener/vrpn_listener.hpp"

VRPNListener::VRPNListener(std::string name) : Node(name)
{
    // load params
    loadParams();

    // init vrpn connection
    std::string host = _server + ":" + std::to_string(_port);
    _vrpn_connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host.c_str()));

    // create timers
    _mainloop_timer = this->create_wall_timer(std::chrono::seconds(1) / _mainloop_frequency, std::bind(&VRPNListener::mainloop, this));
    _trackers_refresh_timer = this->create_wall_timer(std::chrono::seconds(1) / _refresh_trackers_frequency, std::bind(&VRPNListener::refresh_trackers, this));

    RCLCPP_INFO(this->get_logger(), "VRPN listener initialized");
}

void VRPN_CALLBACK VRPNListener::handlePose(void *userData, const vrpn_TRACKERCB poseData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new pose data from sender [%s]", synchronizer->sender_name.c_str());

    // write pose data
    geometry_msgs::msg::PoseStamped pose_msg;
    // write pose data: position
    pose_msg.pose.position.x = poseData.pos[0];
    pose_msg.pose.position.y = poseData.pos[1];
    pose_msg.pose.position.z = poseData.pos[2];
    // write pose data: quanternion
    pose_msg.pose.orientation.x = poseData.quat[0];
    pose_msg.pose.orientation.y = poseData.quat[1];
    pose_msg.pose.orientation.z = poseData.quat[2];
    pose_msg.pose.orientation.w = poseData.quat[3];

    pose_msg.header.frame_id = listener->_frame_id;
    pose_msg.header.stamp.sec = poseData.msg_time.tv_sec;
    pose_msg.header.stamp.nanosec = poseData.msg_time.tv_usec * 1000;

    // publish pose data
    synchronizer->pose_publisher->publish(pose_msg);
}

void VRPN_CALLBACK VRPNListener::handleTwist(void *userData, const vrpn_TRACKERVELCB twistData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new twist data from sender [%s]", synchronizer->sender_name.c_str());

    // write twist data
    geometry_msgs::msg::TwistStamped twist_msg;
    // write twist data: linear
    twist_msg.twist.linear.x = twistData.vel[0];
    twist_msg.twist.linear.y = twistData.vel[1];
    twist_msg.twist.linear.z = twistData.vel[2];
    // write twist data: angular
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(
            twistData.vel_quat[0],
            twistData.vel_quat[1],
            twistData.vel_quat[2],
            twistData.vel_quat[3]));
    double roll, pitch, yaw;
    rot_mat.getRPY(roll, pitch, yaw);
    twist_msg.twist.angular.x = roll;
    twist_msg.twist.angular.y = pitch;
    twist_msg.twist.angular.z = yaw;

    // publish twist data
    synchronizer->twist_publisher->publish(twist_msg);
}

void VRPN_CALLBACK VRPNListener::handleAccel(void *userData, const vrpn_TRACKERACCCB accelData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new accel data from sender [%s]", synchronizer->sender_name.c_str());

    // write accel data
    geometry_msgs::msg::AccelStamped accel_msg;
    // write accel data: linear
    accel_msg.accel.linear.x = accelData.acc[0];
    accel_msg.accel.linear.y = accelData.acc[1];
    accel_msg.accel.linear.z = accelData.acc[2];
    // write accel data: angular
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(
            accelData.acc_quat[0],
            accelData.acc_quat[1],
            accelData.acc_quat[2],
            accelData.acc_quat[3]));
    double roll, pitch, yaw;
    rot_mat.getRPY(roll, pitch, yaw);
    accel_msg.accel.angular.x = roll;
    accel_msg.accel.angular.y = pitch;
    accel_msg.accel.angular.z = yaw;

    // publish accel data
    synchronizer->accel_publisher->publish(accel_msg);
}

void VRPNListener::vrpnConnectionMainloop()
{
    _vrpn_connection->mainloop();

    if (!_vrpn_connection->doing_okay())
    {
        RCLCPP_WARN(this->get_logger(), "VRPN connection is not 'doing okay'");
    }
}

void VRPNListener::refresh_trackers()
{
    for (int i = 0; _vrpn_connection->sender_name(i) != NULL; i++)
    {
        std::string sender_name = _vrpn_connection->sender_name(i);

        if (_synchronizers.count(sender_name) != 0)
        {
            continue;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Found new sender: " << sender_name);
            createSynchronizer(sender_name);
        }
    }
}

void VRPNListener::createSynchronizer(std::string sender_name)
{
    std::string &tracker_name = sender_name;
    std::string &synchronizer_name = sender_name;
    std::string rigid_name = sender_name;
    replaceSpace(rigid_name);

    // new synchronizer
    std::shared_ptr<Synchronizer> new_synchronizer = std::make_shared<Synchronizer>();
    new_synchronizer->sender_name = sender_name;
    new_synchronizer->listener_ptr = this;
    new_synchronizer->vrpn_tracker = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), _vrpn_connection.get());
    std::string pose_topic_name = "/vrpn/" + rigid_name + "/pose";
    std::string twist_topic_name = "/vrpn/" + rigid_name + "/twist";
    std::string accel_topic_name = "/vrpn/" + rigid_name + "/accel";
    new_synchronizer->pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 1);
    new_synchronizer->twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name, 1);
    new_synchronizer->accel_publisher = this->create_publisher<geometry_msgs::msg::AccelStamped>(accel_topic_name, 1);

    // register tracker change handler
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handlePose);
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handleTwist);
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handleAccel);

    // create new tracker mainloop timer
    this->create_wall_timer(
        std::chrono::seconds(1) / _tracker_mainloop_frequency,
        std::bind(&vrpn_Tracker_Remote::mainloop, new_synchronizer->vrpn_tracker));

    // register new synchronizer
    _synchronizers.insert(std::make_pair(synchronizer_name, new_synchronizer));

    RCLCPP_INFO_STREAM(this->get_logger(), "New synchronizer created: " << synchronizer_name);
}

void VRPNListener::mainloop()
{
    vrpnConnectionMainloop();
}

template <class T>
void VRPNListener::loadParam(std::string param_name, T default_value, T &param)
{
    // declare parameter
    this->declare_parameter(param_name, default_value);

    // load parameter
    bool success = this->get_parameter(param_name, param);

    if (!success)
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] failed", param_name.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] success", param_name.c_str());
    }
}

void VRPNListener::loadParams()
{
    loadParam(std::string("server"), std::string(""), _server);
    loadParam(std::string("port"), 3883, _port);
    loadParam(std::string("frame_id"), std::string("world"), _frame_id);
    loadParam(std::string("mainloop_frequency"), 100.0, _mainloop_frequency);
    loadParam(std::string("refresh_trackers_frequency"), 1.0, _refresh_trackers_frequency);
    loadParam(std::string("tracker_mainloop_frequency"), 100.0, _tracker_mainloop_frequency);
}

void VRPNListener::replaceSpace(std::string &ori_str)
{
    for (int i = 0; i < int(ori_str.size()); i++)
    {
        if (ori_str[i] == ' ')
        {
            ori_str[i] = '_';
        }
    }
}

int main(int argc, char **argv)
{
    // init ROS2 node
    rclcpp::init(argc, argv);

    // create VRPNListener entity
    std::string name = "vrpn_listener";
    auto vrpn_listener = std::make_shared<VRPNListener>(name);

    // run ROS2 node
    rclcpp::spin(vrpn_listener);

    // shutdown ROS2 node
    rclcpp::shutdown();

    return 0;
}
