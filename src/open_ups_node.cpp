#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "usbhid.h"
#include "HArray.h"
#include "HIDOpenUPS.h"
#include "HIDInterface.h"

static const int MINIBOX_VENDOR_ID = 0x04d8;
static const int OPENUPS_PRODUCT_ID = 0xd004;
static const int MAX_TRANSFER_SIZE = 32;

namespace open_ups
{

class OpenUPSNode : public rclcpp::Node
{
public:
  OpenUPSNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("open_ups_node", options)
  {
    usb_ = std::make_shared<USBHID>(MINIBOX_VENDOR_ID, OPENUPS_PRODUCT_ID, MAX_TRANSFER_SIZE, 0);
    usb_->open();
    if (!usb_->isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open USB device\n");
      exit(EXIT_FAILURE);
    }
    ups_ = std::make_shared<HIDOpenUPS>(usb_.get());
    battery_state_.cell_voltage.resize(3);
    battery_state_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&OpenUPSNode::timer_callback, this));
  }

  void timer_callback()
  {
    ups_->GetStatus();
    battery_state_.header.stamp = this->now();
    battery_state_.voltage = ups_->m_fVBat;
    battery_state_.current = ups_-> m_fCCharge - ups_->m_fCDischarge;
    battery_state_.capacity = ups_->m_nCapacity;
    battery_state_.cell_voltage[0] = ups_->m_fVCell[0];
    battery_state_.cell_voltage[1] = ups_->m_fVCell[2];
    battery_state_.cell_voltage[2] = ups_->m_fVCell[3];
    battery_state_.temperature = ups_->m_fTemperature;
    battery_state_pub_->publish(battery_state_);
    ups_->printValues();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<USBHID> usb_;
  std::shared_ptr<HIDOpenUPS> ups_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  sensor_msgs::msg::BatteryState battery_state_;
};

}  // namespace open_ups

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(open_ups::OpenUPSNode)
