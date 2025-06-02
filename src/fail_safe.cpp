#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace BT;

class fail_safe : public StatefulActionNode
{
  public:
  fail_safe(const std::string& name, const NodeConfig& cfg)
      : StatefulActionNode(name, cfg)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64>("shouldershift", 1.0);

      sub_risk_ = node_->create_subscription<std_msgs::msg::Float64>(
          "risk_level", 10,
          [this](std_msgs::msg::Float64::SharedPtr m){ risk_ = m->data; });

      sub_res_ = node_->create_subscription<std_msgs::msg::Float64>(
          "shouldershift_result", 10,
          [this](std_msgs::msg::Float64::SharedPtr m){ result_flag_ = m->data; });
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override
    {
      std_msgs::msg::Float64 cmd; 
      cmd.data = 1.0;
      pub_cmd_->publish(cmd);
      result_flag_ = 0.0;     // 반드시 초기화
      RCLCPP_INFO(node_->get_logger(), "[BT] ShoulderShift start");
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
      }

      if (result_flag_==1)     
      { 
        std_msgs::msg::Float64 cmd; 
        cmd.data = 0;
        pub_cmd_->publish(cmd); //다시 디폴트로 복구 
        return NodeStatus::SUCCESS; 
       } // 서버에서 리스크 감지하면 성공 보내고 그거오면 여기도 성공함
      

    //   if (risk_ > 150.0)    { return NodeStatus::FAILURE; } // 실패란 없음

      return NodeStatus::RUNNING;
    }

    void onHalted() override { result_flag_ = 0; }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr      pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    sub_res_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_risk_;

    double risk_{0.0};
    double   result_flag_{0.0};
};
