#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "midterm_2024_msg/msg/midterm.hpp"
#include "mymat.hpp"
#include "myQuaternion.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include <stdio.h>

using std::placeholders::_1;

class Sun_generator : public rclcpp::Node
{
public:
    Sun_generator(const char* nodeName)
    : Node(nodeName)
    {
        //////////////// TODO ////////////////
        // TODO: you can set your subscriber and publishers
        // TODO: subscribed topic should be "demo/model_states_demo"
        // TODO: published topic should be "/Midterm/ForceInput/TorqueBySun" with type : geometry_msgs::msg::Wrench
        // TODO: published topic should be "/Midterm/recordings" with type : midterm_2024_msg::msg::Midterm

        // Create publishers
        pub_torque = create_publisher<geometry_msgs::msg::Wrench>("/Midterm/ForceInput/TorqueBySun", 10);
        pub_recordings = create_publisher<midterm_2024_msg::msg::Midterm>("/Midterm/recordings", 10);

        // Create a subscription to the topic "demo/model_states_demo"
        sub = create_subscription<gazebo_msgs::msg::ModelStates>(
            "demo/model_states_demo",
            10, // Queue size
            std::bind(&Sun_generator::topic_callback, this, std::placeholders::_1) // Callback function
        );

        //////////////// TODO End ////////////////
    }
private:
    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        //////////////// TODO ////////////////
        // TODO: Calculate the torque by sun in world frame.
        // TODO: get information (position, quaternion, angular velocity)publish them by topic (in GCS)

        auto pose = msg->pose[1];    //xyz position and xyzw orientation
        auto twist = msg->twist[1];     //xyz linear and xyz angular velocities

        //compute torque:
        Vec3 w_gcs(twist.angular.x, twist.angular.y, twist.angular.z);
        Vec3 T_dir, torque;
        Vec3 z_w(0.0, 0.0, 1.0);

        T_dir = w_gcs.cross(z_w);
        torque = T_dir * (20.0/sqrt(T_dir.squarednorm()));

//Wrench message
        // Create an instance of the Wrench message
        auto wrench_msg = geometry_msgs::msg::Wrench();
        // Populate the fields of the Wrench message
        wrench_msg.force.x = 0.0;
        wrench_msg.force.y = 0.0;
        wrench_msg.force.z = 0.0;
        wrench_msg.torque.x = torque.get_elem(0,0);
        wrench_msg.torque.y = torque.get_elem(1,0);
        wrench_msg.torque.z = torque.get_elem(2,0);
        // Publish the Wrench message
        pub_torque->publish(wrench_msg);

//Midterm Message
        // Create an instance of the Midterm message
        auto midterm_msg = midterm_2024_msg::msg::Midterm();
        // Populate the fields of the Midterm message
        midterm_msg.angular_velocity = twist.angular;
        midterm_msg.position = pose.position;
        midterm_msg.quaternion = pose.orientation;

        // Publish the Midterm message
        pub_recordings->publish(midterm_msg);

        Quaternion Q(0.97904547, 0, -0.20364175, 0);
        Vec3 w_b(0,0,10);
        Vec3 w_w = vecRotatedByQuat(w_b, Q);

        printf("Angular Velocity = {x : %f, y : %f, z: %f}\n",  w_w.get_elem(0,0), w_w.get_elem(1,0), w_w.get_elem(2,0));

    }
    // TODO: you can set your subscriber and publishers
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_torque;
    rclcpp::Publisher<midterm_2024_msg::msg::Midterm>::SharedPtr pub_recordings;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub;


    //////////////// TODO End ////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sun_generator>("sun_torque_generator_20190837"));
    rclcpp::shutdown();

    return 0;
}
