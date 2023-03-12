#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class CartpoleController : public rclcpp::Node
{
  public:
    CartpoleController(double kp=0.1, double ki=0.01, double kd=0.1, double frequency=50.0)
    : Node("cartpole_controller"), count_(0)
    {
      this->declare_parameter("cartpole_kp");
      // I-controller param
      // this->declare_parameter("cartpole_ki");
      try{
        this->kp_ = get_parameter("cartpole_kp").as_double();
        // I-controller param
        // this->kp_ = get_parameter("cartpole_ki").as_double();
      }
      catch(const rclcpp::exceptions::ParameterNotDeclaredException & e) {
        this->kp_ = kp;
        // I-controller param
        // this->ki_ = ki;
        RCLCPP_INFO(this->get_logger(), "'cartpole_kp' not set, defaulting to %f", this->kp_);
        // I-controller param
        // RCLCPP_INFO(this->get_logger(), "'cartpole_ki' not set, defaulting to %f", this->ki_);
      }
      force_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/cart_cmd_force", 10);
      joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CartpoleController::joint_states_callback, this, _1));
      this->period_ = 1.0 / frequency;
      timer_ = this->create_wall_timer(std::chrono::duration<double, std::ratio<1>>(this->period_), bind(&CartpoleController::timer_callback, this));

      this->cumulative_error_ = 0.0;
    }
  private:
    void timer_callback()
    {
      try{
        if (this->kp_ != get_parameter("cartpole_kp").as_double()) {
          this->kp_ = get_parameter("cartpole_kp").as_double();
          RCLCPP_INFO(this->get_logger(), "kp changed to %f", this->kp_);
        }
        // I-controller param
        // if (this->ki_ != get_parameter("cartpole_ki").as_double()) {
        //   this->ki_ = get_parameter("cartpole_ki").as_double();
        //   RCLCPP_INFO(this->get_logger(), "ki changed to %f", this->ki_);
        // }
      }
      catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {}
      std_msgs::msg::Float64 message = std_msgs::msg::Float64();
      
      // P-controller
      message.data = (-this->kp_ * this->beam_to_cart_position_);

      // PI-controller, needs checking
      // message.data = (-this->kp_ * this->beam_to_cart_position_) +
      //                (-this->period_ * this->ki_ * this->cumulative_error_);
      // this->cumulative_error_ += this->beam_to_cart_position_;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);

      force_publisher_->publish(message);
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      size_t count = msg->name.size();
      for (size_t i = 0; i < count; ++i) {
        // RCLCPP_INFO(this->get_logger(), "Joint state %s: {pos: %f, vel: %f, eff: %f}",
        // msg->name.at(i).c_str(), msg->position.at(i), msg->velocity.at(i), msg->effort.at(i));
        if (msg->name.at(i) == "beam_to_cart") {
          this->beam_to_cart_position_ = double(msg->position.at(i));
        }
      }

    }
    double kp_;
    // double ki_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    double period_;
    double beam_to_cart_position_;
    double cumulative_error_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartpoleController>());
  rclcpp::shutdown();
  return 0;
}
