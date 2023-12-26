#include "rclcpp/rclcpp.hpp"
#include "usbhid.h"
#include "HArray.h"
#include "HIDOpenUPS.h"
#include "HIDInterface.h"

static const int MINIBOX_VENDOR_ID = 0x04d8;
static const int OPENUPS_PRODUCT_ID = 0xd004;
static const int MAX_TRANSFER_SIZE = 32;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("open_ups_node");
  USBHID *dev = new USBHID(MINIBOX_VENDOR_ID, OPENUPS_PRODUCT_ID, MAX_TRANSFER_SIZE, 0);
  dev->open();
	if (!dev->isOpened())
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to open USB device\n");
    exit(EXIT_FAILURE);
	}
  HIDInterface *ups = new HIDOpenUPS(dev);
  ups->GetStatus();
	ups->printValues();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
