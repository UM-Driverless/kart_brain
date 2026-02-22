#ifndef KB_COMS_MICRO_HPP_
#define KB_COMS_MICRO_HPP_

#include "kb_coms_serial_driver.hpp"
#include "kb_interfaces/msg/frame.hpp"
#include <rclcpp/rclcpp.hpp>

class KB_coms_micro : public rclcpp::Node {

  public:
    KB_coms_micro();

    ~KB_coms_micro();

  private:
    void rx_callback(const SerialDriver::Frame &frame);

    void tx_callback(const kb_interfaces::msg::Frame::SharedPtr msg);

    std::unique_ptr<SerialDriver> serial_;

    // Declaration of all publishers
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_heart_pub_;

    // Declaration of all subscribers
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr tx_sub_;
};

#endif // KB_COMS_MICRO_HPP_
