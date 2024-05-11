
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateToControl : public rclcpp::Node
{
    public:
        JointStateToControl()
        : Node("joint_state_to_control")
        {
            joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states_gui", 10, std::bind(&JointStateToControl::joint_state_callback, this, std::placeholders::_1)
            );

            joint_control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/panda_joint_group_controller/commands", 10);

        }
        
    private:
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::vector<double> joint_positions = msg->position;
            std::vector<double> joint_velocities = msg->velocity;
            std::vector<double> joint_efforts = msg->effort;

            RCLCPP_INFO(this->get_logger(), "Joint 1: %f, Joint 2: %f, Joint 3: %f, Joint 4: %f, Joint 5: %f, Joint 6: %f, Joint 7: %f",
                joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], joint_positions[6]);

            RCLCPP_INFO(this->get_logger(), "Joint Finger 1: %f, Joint Finger 2: %f", joint_positions[7], joint_positions[8]);

            std_msgs::msg::Float64MultiArray control_msg;
            control_msg.data = joint_positions;

            joint_control_pub_->publish(control_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_control_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateToControl>());
    rclcpp::shutdown();
    return 0;
}