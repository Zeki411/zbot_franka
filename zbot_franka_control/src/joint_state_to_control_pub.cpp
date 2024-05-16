#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#define NUMBER_OF_JOINTS 7

class JointStateToControl : public rclcpp::Node
{
    public:
        JointStateToControl()
        : Node("joint_state_to_control")
        {
            joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states_gui_commands", 10, std::bind(&JointStateToControl::joint_state_callback, this, std::placeholders::_1)
            );

            joint_control_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/panda_arm_controller/joint_trajectory", 10);

        }
        
    private:
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received joint state message");

            trajectory_msgs::msg::JointTrajectory control_msg; // Create a new JointTrajectory message
            trajectory_msgs::msg::JointTrajectoryPoint point; // Create a new JointTrajectoryPoint
            
            for (int i = 0; i < NUMBER_OF_JOINTS; i++)
            {
                control_msg.joint_names.push_back(msg->name[i]);
                point.positions.push_back(msg->position[i]);
                // point.velocities.push_back(msg->velocity[i]);
                // point.effort.push_back(msg->effort[i]);
            }

            // point.time_from_start = rclcpp::Duration::from_seconds(1); // Set a fixed duration for simplicity

            // Add the populated point to the JointTrajectory message
            control_msg.points.push_back(point);
            control_msg.header.stamp = this->now();

            // Publish the populated JointTrajectory message
            joint_control_pub_->publish(control_msg);

            RCLCPP_INFO(this->get_logger(), "Published joint trajectory message");
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_control_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateToControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
