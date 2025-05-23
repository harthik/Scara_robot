#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::placeholders::_1;

class JointPublisher : public rclcpp::Node {
public:
    JointPublisher()
        : Node("joint_publisher"), q(VectorXd::Zero(3)), t(0.0), dt(0.01), a_q_move(VectorXd::Zero(3)), elapsed_time(0.0),msg_flag(false), pos_l(0.12f) {
        publisher_angles = this->create_publisher<std_msgs::msg::Float32MultiArray>("/joint_angles", 10);
        publisher_flags = this->create_publisher<std_msgs::msg::Bool>("/joint_flags", 10);

        // Subscriber for desired position
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/desired_pos",
            10,
            std::bind(&JointPublisher::position_callback, this, _1));

        actual_pos_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/actual_pos", 10,
            std::bind(&JointPublisher::actual_pos_call, this, _1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JointPublisher::timer_callback, this));
        q(0) = 0.0;
        q(1) = M_PI/2;
        q(2) = 0.08;

        pd = VectorXd::Zero(3);
        pd(0) = 0.1;
        pd(1) = 0.08;
        pd(2) = 0.12;
    }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Check if the new position is different from the current desired position
        pd(0) = msg->x;
        pd(1) = msg->y;
        pd(2) = msg->z;
        RCLCPP_INFO(this->get_logger(), "recived pos: [%f, %f, %f]", pd(0), pd(1), pd(2));
    }
    void actual_pos_call(const std_msgs::msg::Float32::SharedPtr msg) {
        // Check if the new position is different from the current desired position
        pos_l = msg->data;
        RCLCPP_INFO(this->get_logger(), "recived pos: %f", pos_l);
    }

    void timer_callback() {
        VectorXd qdot = compute_joint_angle_update(q);
        VectorXd q_move = qdot * dt; // angle to be moved for the time step
        a_q_move += q_move*180/M_PI; // accumulated angle to be moved being published to motors
        VectorXd pos = direct_kinematics_SCARA(q); // calculating pose based on model alone not real robot

        q += qdot * dt;
        //t += dt;
        elapsed_time += dt;

        // Publish the accumulated updates every 1 seconds this needs to be updated when publishing to stepper motors
        if (elapsed_time >= 1.0) {
            std_msgs::msg::Float32MultiArray msg;
            std::vector<float> float_data;
            for (int i = 0; i < a_q_move.size(); ++i) {
                float raw = static_cast<float>(a_q_move(i));
                // round to two decimals
                float two_dec = std::round(raw * 100.0f) / 100.0f;
                float_data.push_back(two_dec);
            }
            msg.data = float_data;
            double error  = (pd - pos).norm();
            
            if (error >= 0.001){ // need to change flags from bool to int
                msg_flag = false;
                // build a Bool message and publish it
                std_msgs::msg::Bool flag_msg;
                flag_msg.data = msg_flag;
                publisher_flags->publish(flag_msg);
                publisher_angles->publish(msg);
                RCLCPP_INFO(this->get_logger(),"moving");
                RCLCPP_INFO(this->get_logger(), "error is %f", error);
                RCLCPP_INFO(this->get_logger(), "publishing q_move: [%f, %f, %f]", a_q_move(0), a_q_move(1), a_q_move(2));
                RCLCPP_INFO(this->get_logger(), "at position [%f, %f, %f]", pos(0), pos(1), pos(2));
            }
            else if (msg_flag == false && error < 0.001) {
                msg_flag = true;
                std_msgs::msg::Bool flag_msg;
                flag_msg.data = msg_flag;
                publisher_flags->publish(flag_msg);
                RCLCPP_INFO(this->get_logger(),"reached");
            }
            a_q_move.setZero();
            elapsed_time = 0.0;

        }
    }

    VectorXd direct_kinematics_SCARA(const VectorXd &q) {
        VectorXd xe(3);
        xe(0) = 0.1 * cos(q(0)) + 0.08 * cos(q(0) + q(1));
        xe(1) = 0.1 * sin(q(0)) + 0.08 * sin(q(0) + q(1));
        xe(2) = static_cast<double>(pos_l); 
        return xe;
    }

    MatrixXd analytical_jacobian(const VectorXd &q) {
        MatrixXd J_A(3, 3);
        J_A(0, 0) = -0.1 * sin(q(0)) - 0.08 * sin(q(0) + q(1));
        J_A(0, 1) = -0.08 * sin(q(0) + q(1));
        J_A(0, 2) = 0;
        
        // Second row: partial derivatives of y
        J_A(1, 0) = 0.1 * cos(q(0)) + 0.08 * cos(q(0) + q(1));
        J_A(1, 1) = 0.08 * cos(q(0) + q(1));
        J_A(1, 2) = 0;
        
        // Third row: partial derivatives of z
        J_A(2, 0) = 0;
        J_A(2, 1) = 0;
        J_A(2, 2) = 1.0;
        return J_A;
    }

    VectorXd compute_joint_angle_update(const VectorXd &q) {

        VectorXd pd_dot(3);
        pd_dot(0) = 0.0;
        pd_dot(1) = 0.0;
        pd_dot(2) = 0;

        MatrixXd K = MatrixXd::Identity(3, 3);
        VectorXd x_e = direct_kinematics_SCARA(q);
        MatrixXd J_A = analytical_jacobian(q);
        VectorXd e = pd - x_e;
        MatrixXd J_A_pseudo_inverse = J_A.completeOrthogonalDecomposition().pseudoInverse();
        VectorXd qdot = J_A_pseudo_inverse * (pd_dot + K * e);
        return qdot;
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_angles;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_flags; // change msg type to int
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actual_pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    VectorXd q;
    double t;
    double dt;
    VectorXd accumulated_q_move;
    VectorXd a_q_move;
    double elapsed_time;
    VectorXd pdi;
    VectorXd pd;
    bool msg_flag;
    float pos_l;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}