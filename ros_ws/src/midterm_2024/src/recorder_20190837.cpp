#include <memory>
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "midterm_2024_msg/msg/midterm.hpp"
#include "mymat.hpp"
#include "myQuaternion.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using std::placeholders::_1;

class Recorder : public rclcpp::Node
{
public:
    Recorder(const char* nodeName)
    : Node(nodeName)
    {
        cnt = 0;
        p_results.open("p_results_gazebo.csv");
        w_results.open("w_results_gazebo.csv");
        q_results.open("q_results_gazebo.csv");
        //////////////// TODO ////////////////
        // TODO: you can set your subscriber for you published topic in another node
        // TODO: msg topic should be "Midterm/recordings"

        // Create a subscription to the topic "Midterm/recordings"
        sub = create_subscription<midterm_2024_msg::msg::Midterm>(
            "Midterm/recordings",
            10, // Queue size
            std::bind(&Recorder::topic_callback, this, std::placeholders::_1) // Callback function
        );

        //////////////// TODO End ////////////////

    }
private:
    void topic_callback(const midterm_2024_msg::msg::Midterm::SharedPtr msg)
    {
        //////////////// TODO ////////////////
        // you should make below 3 csv files from published information.
        // p_results_gazebo.csv contains earth position in global coordinate system: : x,y,z
        // w_results_gazebo.csv contains earth angular velocity in global coordinate system: x,y,z
        // q_results_gazebo.csv contains earth quaternion in global coordinate system:: w,x,y,z

        if(p_results.is_open()){
            p_results << msg->position.x << ", " << msg->position.y << ", " << msg->position.z << ", "<< '\n';
        }
        if(w_results.is_open()){
            w_results << msg->angular_velocity.x << ", " << msg->angular_velocity.y  << ", " << msg->angular_velocity.z << ", " << '\n';
        }
        if(q_results.is_open()){
            q_results << msg->quaternion.w << ", " << msg->quaternion.x << ", " << msg->quaternion.y << ", " << msg->quaternion.z << ", " << '\n';
        }
        //////////////// TODO End ////////////////
        cnt = cnt + 1;
        // subscription rate would be 10Hz -> total 1000 sec
        // If your environment is very slow, try to control below count to record more than 1000 sec
        if( cnt == 10000) {
            p_results.close();
            q_results.close();
            w_results.close();
        }

    }
    //////////////// TODO ////////////////
    // TODO: you can set your subscriber for you published topic in another node
    rclcpp::Subscription<midterm_2024_msg::msg::Midterm>::SharedPtr sub;
    //////////////// TODO End ////////////////
    int cnt;
    std::ofstream p_results;
    std::ofstream q_results;
    std::ofstream w_results;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Recorder>("recorder_20190837"));
    rclcpp::shutdown();

    return 0;
}
