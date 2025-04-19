#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

class PointPublisher : public rclcpp::Node
{
public:
  PointPublisher()
  : Node("point_publisher"), flag_old(true), task_index(1)
  {
    // Create a publisher on the "point" topic, queue depth 10
    pub_pos = this->create_publisher<geometry_msgs::msg::Point>("/desired_pos", 10);
    sub_flags = this->create_subscription<std_msgs::msg::Bool>(
        "/joint_flags",
        10,
        std::bind(&PointPublisher::flags_callback, this, std::placeholders::_1));

      task_points_.resize(3);

      task_points_[0].x = 0.1; task_points_[0].y = 0.08; task_points_[0].z = 0.1;
      task_points_[1].x = 0.1; task_points_[1].y = 0.08; task_points_[1].z = 0.12;
      task_points_[2].x = 0.1; task_points_[2].y = 0.08; task_points_[2].z = 0.1;
  }

private:
    void flags_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool flag = msg->data;
        RCLCPP_INFO(this->get_logger(), "msg_flag: %s", flag ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "msg_flag_old: %s", flag_old ? "true" : "false");
        if (flag && flag_old) {
            publish_point(task_index);
        }
        else if (flag && !flag_old) {
            RCLCPP_INFO(this->get_logger(), "change flag");
            task_index++;
            publish_point(task_index);
            RCLCPP_INFO(this->get_logger(), "task_index: %d", task_index);
            flag_old = true;
        }
        else{
            flag_old = flag;
        }
        
    }
    void publish_point(int idx)
    {
        int vi = std::clamp(idx - 1, 0, (int)task_points_.size() - 1);

        auto msg = task_points_[vi];
        RCLCPP_INFO(this->get_logger(),
                    "Publishing Point(x=%.2f, y=%.2f, z=%.2f)",
                    msg.x, msg.y, msg.z);
        pub_pos->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_flags;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esp_flags;
    bool flag_old;
    bool flag_esp;
    bool flag_sub;
    bool flag;
    int task_index;
    std::vector<geometry_msgs::msg::Point> task_points_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisher>());
    rclcpp::shutdown();
    return 0;
}
