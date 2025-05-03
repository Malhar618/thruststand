#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <unistd.h>
#include <cmath>

#define ADDR_OPERATING_MODE    11
#define ADDR_TORQUE_ENABLE     64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

#define PROTOCOL_VERSION     2.0
#define DXL_ID_ROLL             2
#define BAUDRATE           4000000
#define DEVICENAME     "/dev/ttyUSB0"

#define RESOLUTION      4096.0
#define MID_POINT       (RESOLUTION/2.0)
#define DEG2UNIT        (RESOLUTION/360.0)

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("roll_sine_position_node");

  auto port = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  auto ph   = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(!port->openPort()) {
    RCLCPP_FATAL(node->get_logger(), "Cannot open %s", DEVICENAME);
    return 1;
  }
  if(!port->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(node->get_logger(), "Cannot set baud %d", BAUDRATE);
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Port opened @ %d", BAUDRATE);

  // 1) Switch into Position Mode **once**
  {
    uint8_t err;
    int res = ph->write1ByteTxRx(port, DXL_ID_ROLL, ADDR_OPERATING_MODE, 3, &err);
    usleep(2000);
    if(res!=COMM_SUCCESS || err) {
      RCLCPP_FATAL(node->get_logger(),
        "OpMode→Position failed: %s / %s",
        ph->getTxRxResult(res),
        ph->getRxPacketError(err)
      );
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Operating mode → Position");
  }

  // 2) Enable torque
  {
    uint8_t err;
    int res = ph->write1ByteTxRx(port, DXL_ID_ROLL, ADDR_TORQUE_ENABLE, 1, &err);
    usleep(2000);
    if(res!=COMM_SUCCESS || err) {
      RCLCPP_FATAL(node->get_logger(),
        "Torque enable failed: %s / %s",
        ph->getTxRxResult(res),
        ph->getRxPacketError(err)
      );
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Torque ENABLED");
  }

  // 3) Sine‑drive loop
  rclcpp::Rate rate(100);  // 100 Hz
  double t=0, dt=0.01;     // dt matches 100Hz
  const double amp=20.0;   // degrees
  const double freq=0.5;   // Hz

  while(rclcpp::ok()) {
    // compute next set‑point ([-amp, +amp])
    double deg = amp * std::sin(2*M_PI*freq*t);
    uint32_t goal = uint32_t( MID_POINT + deg*DEG2UNIT );

    // write it
    {
      uint8_t err;
      int res = ph->write4ByteTxRx(port, DXL_ID_ROLL, ADDR_GOAL_POSITION, goal, &err);
      usleep(2000);
      if(res!=COMM_SUCCESS)
        RCLCPP_ERROR(node->get_logger(), "write4Byte failed: %s", ph->getTxRxResult(res));
      else if(err)
        RCLCPP_ERROR(node->get_logger(), "packet err: %s", ph->getRxPacketError(err));
    }

    // read back
    {
      uint8_t err;  uint32_t present=0;
      int res = ph->read4ByteTxRx(port, DXL_ID_ROLL, ADDR_PRESENT_POSITION, &present, &err);
      usleep(2000);
      if(res==COMM_SUCCESS && !err) {
        double act = (double(present)-MID_POINT)/DEG2UNIT;
        RCLCPP_INFO(node->get_logger(),
          "t=%.2f  goal=%.1f°  read=%.1f°", t, deg, act
        );
      }
    }

    rate.sleep();
    rclcpp::spin_some(node);
    t += dt;
  }

  // on exit → disable torque
  { uint8_t e; ph->write1ByteTxRx(port, DXL_ID_ROLL, ADDR_TORQUE_ENABLE, 0, &e); }
  port->closePort();
  rclcpp::shutdown();
  return 0;
}
