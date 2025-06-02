#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace BT;

class WarningMode : public StatefulActionNode
{
  public:
    WarningMode(const std::string& name, const NodeConfig& cfg)
      : StatefulActionNode(name, cfg)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64>("warningmode", 1);

      sub_risk_ = node_->create_subscription<std_msgs::msg::Float64>(
          "risk_level", 10,
          [this](std_msgs::msg::Float64::SharedPtr m){ risk_ = m->data; });

      sub_res_ = node_->create_subscription<std_msgs::msg::Float64>(
          "warningmode_result", 10,
          [this](std_msgs::msg::Float64::SharedPtr m){ result_flag_ = m->data; });
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override
    {
      /* command = true 1회 발행 */
      std_msgs::msg::Float64 cmd; cmd.data = 1.0;
      pub_cmd_->publish(cmd);
      result_flag_ = 0;
      RCLCPP_INFO(node_->get_logger(), "[BT] WarningMode start");
      return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
      rclcpp::spin_some(node_);

      if (risk_< 1)
      {
        std_msgs::msg::Float64 cmd; 
        cmd.data = 2.0;
        pub_cmd_->publish(cmd);
        return NodeStatus::SUCCESS;
      }

      if (result_flag_==1.0) 
      { 
        std_msgs::msg::Float64 cmd; cmd.data = 0.0;
        pub_cmd_->publish(cmd);
        return NodeStatus::SUCCESS; 
       }
      
      if (risk_ > 30.0)
       {
        std_msgs::msg::Float64 cmd; cmd.data = 0.0;
        pub_cmd_->publish(cmd);
        return NodeStatus::FAILURE; 
        }

      return NodeStatus::RUNNING;
    }

    void onHalted() override { result_flag_ = 0.0; }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr      pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    sub_res_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_risk_;

    double risk_{0.0};
    double result_flag_{0.0};
};
