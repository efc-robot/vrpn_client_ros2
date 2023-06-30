#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <unistd.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
// #include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

struct Synchronizer
{
    std::string sender_name;
    void *listener_ptr;
    std::shared_ptr<vrpn_Tracker_Remote> vrpn_tracker;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_publisher;
};

class VRPNListener : public rclcpp::Node
{
private:
    // params
    int _port;
    std::string _server;
    std::string _frame_id;
    double _mainloop_frequency;
    double _refresh_trackers_frequency;
    double _tracker_mainloop_frequency;

    // vrpn entitis
    std::shared_ptr<vrpn_Connection> _vrpn_connection;

    // synchronizer
    std::unordered_map<std::string, std::shared_ptr<Synchronizer>> _synchronizers;

    // timers
    rclcpp::TimerBase::SharedPtr _mainloop_timer;
    rclcpp::TimerBase::SharedPtr _trackers_refresh_timer;

    static void VRPN_CALLBACK handlePose(void *userData, const vrpn_TRACKERCB trackerData);
    static void VRPN_CALLBACK handleTwist(void *userData, const vrpn_TRACKERVELCB trackerData);
    static void VRPN_CALLBACK handleAccel(void *userData, const vrpn_TRACKERACCCB trackerData);
    void vrpnConnectionMainloop();
    void refresh_trackers();
    void mainloop();
    template <class T>
    void loadParam(std::string param_name, T default_value, T &param);
    void loadParams();
    void replaceSpace(std::string &ori_str);
    void createSynchronizer(std::string sender_name);

public:
    VRPNListener(std::string name);
};