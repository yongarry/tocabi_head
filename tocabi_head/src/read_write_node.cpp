// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "tocabi_head/GetPosition.h"
#include "tocabi_head/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "tocabi_msgs/matrix_3_4.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

#include <Eigen/Dense>
#include "tf/tf.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

ros::Publisher get_HMD_orientation_yaw;
ros::Publisher get_dynamixel_orientation_yaw;

bool getPresentPositionCallback(
  tocabi_head::GetPosition::Request & req,
  tocabi_head::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

void getHmdOrientationCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  uint8_t dxl_error = 0;
  int32_t position = 0;
  std_msgs::Float64 msg1, msg2;

  int dxl_comm_result = COMM_TX_FAIL;

  tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("[Pitch:%f] [Yaw:%f]", pitch, yaw);
  int pitch_ = (pitch + M_PI) * 4096 / (2 * M_PI);
  int yaw_   = (yaw + M_PI) * 4096 / (2 * M_PI);
  
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)1, ADDR_GOAL_POSITION, yaw_, &dxl_error);

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)2, ADDR_GOAL_POSITION, pitch_, &dxl_error);  
    
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 1, yaw_);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 2, pitch_);

    // dxl_comm_result = packetHandler->read4ByteTxRx(
    // portHandler, (uint8_t)1, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    // msg1.data = yaw;
    
    // get_HMD_orientation_yaw.publish(msg1);
    // double dynamixel_yaw = position * (2 * M_PI) / 4096 - M_PI;
    // msg2.data = dynamixel_yaw;
    // get_dynamixel_orientation_yaw.publish(msg2);

  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber get_HMD_orientation = nh.subscribe<geometry_msgs::Pose>("/HMD", 10, getHmdOrientationCallback);

  get_HMD_orientation_yaw = nh.advertise<std_msgs::Float64>("/HMD_yaw", 100);
  get_dynamixel_orientation_yaw = nh.advertise<std_msgs::Float64>("/dynamixel_yaw", 100);
  ros::spin();

  portHandler->closePort();
  return 0;
}
