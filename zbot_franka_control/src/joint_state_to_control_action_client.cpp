#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#define NUMBER_OF_JOINTS 7


using namespace std::chrono_literals;

class JointStateToControl : public rclcpp::Node
{
public:
    JointStateToControl()
    : Node("joint_state_to_control"),
      ready_to_send(true)
    {
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&JointStateToControl::joint_state_callback, this, std::placeholders::_1)
        );

        // joint_states_gui_commands_sub_ = nullptr;
        this->joint_states_gui_commands_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_gui_commands", 1, std::bind(&JointStateToControl::joint_state_gui_commands_callback, this, std::placeholders::_1)
        );

        this->follow_joint_trajectory_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "panda_arm_controller/follow_joint_trajectory"
        );

        current_joint_positions.resize(NUMBER_OF_JOINTS, 0.0);
        pending_joint_positions.resize(NUMBER_OF_JOINTS, 0.0);
        latest_joint_state_positions.resize(NUMBER_OF_JOINTS, 0.0);

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_gui_commands_sub_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> latest_joint_state_positions;
    std::vector<double> current_joint_positions;
    std::vector<double> pending_joint_positions;
    
    bool ready_to_send;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            latest_joint_state_positions[i] = msg->position[i];
        }
    }

    void joint_state_gui_commands_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        bool is_same = true;

        if (!this->follow_joint_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        if(!ready_to_send)
        {
            // RCLCPP_INFO(this->get_logger(), "Not ready to send");
            return;
        }

        // check if joint positions are same to the current joint positions
        for (int i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            if (msg->position[i] != current_joint_positions[i])
            {
                is_same = false;
                break;
            }
        }

        if (is_same)
        {
            // RCLCPP_INFO(this->get_logger(), "Same joint positions");
            return;
        }
        
        ready_to_send = false;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

        RCLCPP_INFO(this->get_logger(), "Joint Position command: %f %f %f %f %f %f %f",
            msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6]);

        for (int i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            goal_msg.trajectory.joint_names.push_back(msg->name[i]);\
            point.positions.push_back(msg->position[i]);
            pending_joint_positions[i] = msg->position[i];
        }

        // set the duration of the trajectory
        point.time_from_start.sec = 1;
        point.time_from_start.nanosec = 0;
        
        goal_msg.trajectory.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&JointStateToControl::goal_response_callback, this, std::placeholders::_1);
        // send_goal_options.feedback_callback =
        //     std::bind(&JointStateToControl::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&JointStateToControl::goal_result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending goal to action server.");
        this->follow_joint_trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback:");

        // for (size_t i = 0; i < feedback->joint_names.size(); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "Joint: %s", feedback->joint_names[i].c_str());
        //     RCLCPP_INFO(this->get_logger(), " - Desired position: %f", feedback->desired.positions[i]);
        //     RCLCPP_INFO(this->get_logger(), " - Actual position: %f", feedback->actual.positions[i]);
        //     RCLCPP_INFO(this->get_logger(), " - Error: %f", feedback->error.positions[i]);
        // }
    }

    void goal_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
        ready_to_send = true;
        current_joint_positions = latest_joint_state_positions;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was successful");
                current_joint_positions = pending_joint_positions; // Update the current joint positions
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    builtin_interfaces::msg::Duration toBuiltinDuration(const rclcpp::Duration& duration)
    {
        builtin_interfaces::msg::Duration builtin_duration;
        builtin_duration.sec = duration.seconds();
        builtin_duration.nanosec = duration.nanoseconds() % 1000000000;
        return builtin_duration;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateToControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
