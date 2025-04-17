#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

class PointPublisher : public rclcpp::Node
{
public:
  PointPublisher()
  : Node("point_publisher"), flag_old(false)
  {
    // Create a publisher on the "point" topic, queue depth 10
    pub_pos = this->create_publisher<geometry_msgs::msg::Point>("/desired_pos", 10);
    sub_flags = this->create_subscription<std_msgs::msg::Bool>(
      "/joint_flags",
      10,
      std::bind(&PointPublisher::flags_callback, this, std::placeholders::_1));
  }

private:
    void flags_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool flag = msg->data;
        RCLCPP_INFO(this->get_logger(), "msg_flag: %s", flag ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "msg_flag_old: %s", flag_old ? "true" : "false");
        if (flag && !flag_old) {
            RCLCPP_INFO(this->get_logger(), "change flag");
            publish_point();
        }
        flag_old = flag;
    }
    void publish_point()
    {
        auto msg = geometry_msgs::msg::Point();
        msg.x = 0.1;
        msg.y = 0.0;
        msg.z = 0.12;
        RCLCPP_INFO(this->get_logger(),
                    "Publishing Point(x=%.2f, y=%.2f, z=%.2f)",
                    msg.x, msg.y, msg.z);
        pub_pos->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_flags;
    bool flag_old;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisher>());
    rclcpp::shutdown();
    return 0;
}
