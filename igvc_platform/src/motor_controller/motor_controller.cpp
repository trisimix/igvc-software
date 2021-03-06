#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <igvc_utils/NodeUtils.hpp>

// nanopb header files for protobuffer encoding/decoding
#include <igvc_platform/nanopb/pb_common.h>
#include <igvc_platform/nanopb/pb_decode.h>
#include <igvc_platform/nanopb/pb_encode.h>
#include "igvc_platform/nanopb/protos/igvc.pb.h"  // compiled protobuf definition

#include "motor_controller.h"

MotorController::MotorController(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  // initialize private node handle
  ros::NodeHandle pNh("~");

  // initialize subscriber to /motors topic
  cmd_sub_ = nh_.subscribe("/motors", 1, &MotorController::cmdCallback, this);

  // initialize publishers to publish mbed stats
  enc_pub_ = nh_.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);
  enabled_pub_ = nh_.advertise<std_msgs::Bool>("/robot_enabled", 1);
  battery_pub_ = nh_.advertise<std_msgs::Float64>("/battery", 1);

  // get server ip address and port number from the launch file
  igvc::getParam(pNh, std::string("ip_addr"), ip_addr_);
  igvc::getParam(pNh, std::string("port"), tcpport_);

  ROS_INFO_STREAM("Connecting to server:"
                  << "\n\tIP: " << ip_addr_ << "\n\tPort: " << std::to_string(tcpport_));
  sock_ = igvc::make_unique<EthernetSocket>(ip_addr_, tcpport_);
  ROS_INFO_STREAM("Using Boost " << (*sock_).getBoostVersion());
  ROS_INFO_STREAM("Successfully Connected to TCP Host:"
                  << "\n\tIP: " << (*sock_).getIP() << "\n\tPort: " << (*sock_).getPort());

  igvc::getParam(pNh, std::string("battery_alpha"), battery_alpha_);
  igvc::getParam(pNh, std::string("min_battery_voltage"), min_battery_voltage_);

  battery_avg_ = min_battery_voltage_;

  // PID variables
  igvc::getParam(pNh, std::string("p_l"), p_l_);
  igvc::getParam(pNh, std::string("p_r"), p_r_);
  igvc::getParam(pNh, std::string("d_l"), d_l_);
  igvc::getParam(pNh, std::string("d_r"), d_r_);
  igvc::getParam(pNh, std::string("i_r"), i_r_);
  igvc::getParam(pNh, std::string("i_l"), i_l_);

  igvc::param(pNh, "log_period", log_period_, 5.0);

  // communication frequency
  igvc::getParam(pNh, std::string("frequency"), frequency_);
  ros::Rate rate(frequency_);

  setPID();  // Set PID Values on mbed

  // send motor commands
  while (ros::ok())
  {
    sendRequest();
    recieveResponse();
    ros::spinOnce();
    rate.sleep();
  }
}

void MotorController::cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command_ = *msg;
}

void MotorController::setPID()
{
  ros::Rate rate(frequency_);
  ROS_INFO_STREAM("Setting PID Values:"
                  << "\n\t P => L: " << p_l_ << " R: " << p_r_ << "\n\t D => L: " << d_l_ << " R: " << d_r_
                  << "\n\t I => L: " << i_l_ << " R: " << i_r_);

  bool valid_values = false;  // pid values have been set correctly

  /* This is the buffer where we will store the request message. */
  uint8_t requestbuffer[256];
  size_t message_length;
  bool status;

  /* allocate space for the request message to the server */
  RequestMessage request = RequestMessage_init_zero;

  /* Create a stream that will write to our buffer. */
  pb_ostream_t ostream = pb_ostream_from_buffer(requestbuffer, sizeof(requestbuffer));

  /* indicate that pid fields will contain values */
  request.has_p_l = true;
  request.has_p_r = true;
  request.has_i_l = true;
  request.has_i_r = true;
  request.has_d_l = true;
  request.has_d_r = true;

  /* fill in the message fields */
  request.p_l = static_cast<float>(p_l_);
  request.p_r = static_cast<float>(p_r_);
  request.i_l = static_cast<float>(i_l_);
  request.i_r = static_cast<float>(i_r_);
  request.d_l = static_cast<float>(d_l_);
  request.d_r = static_cast<float>(d_r_);

  /* encode the protobuffer */
  status = pb_encode(&ostream, RequestMessage_fields, &request);
  message_length = ostream.bytes_written;

  /* check for any errors.. */
  if (!status)
  {
    ROS_ERROR_STREAM("Encoding failed: " << PB_GET_ERROR(&ostream));
    ros::shutdown();
  }

  size_t n;             // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
  uint8_t buffer[256];  // buffer to read response into
  // unsigned char responsebuffer[256];

  /* Send PID values via ethernet and recieve response to ensure proper setting */
  while (ros::ok() && !valid_values)
  {
    (*sock_).sendMessage(reinterpret_cast<char*>(requestbuffer), message_length);

    memset(buffer, 0, sizeof(buffer));
    n = (*sock_).readMessage(buffer);  // blocks until data is read
    // memcpy(responsebuffer, buffer, sizeof(responsebuffer));

    if (n == 0)
    {
      ROS_ERROR_STREAM("Connection closed by server");
      ros::shutdown();
    }

    /* Allocate space for the decoded message. */
    ResponseMessage response = ResponseMessage_init_zero;

    /* Create a stream that reads from the buffer. */
    pb_istream_t istream = pb_istream_from_buffer(buffer, n);

    /* decode the message. */
    status = pb_decode(&istream, ResponseMessage_fields, &response);

    /* check for any errors.. */
    if (!status)
    {
      ROS_ERROR_STREAM("Decoding Failed: " << PB_GET_ERROR(&istream));
      ros::shutdown();
    }

    valid_values = (response.p_l == static_cast<float>(p_l_)) && (response.p_r == static_cast<float>(p_r_)) &&
                   (response.i_l == static_cast<float>(i_l_)) && (response.i_r == static_cast<float>(i_r_)) &&
                   (response.d_l == static_cast<float>(d_l_)) && (response.d_r == static_cast<float>(d_r_));

    rate.sleep();
  }
  ROS_INFO_ONCE("Sucessfully set all PID values");
}

void MotorController::sendRequest()
{
  /* This is the buffer where we will store the request message. */
  uint8_t requestbuffer[256];

  /* allocate space for the request message to the server */
  RequestMessage request = RequestMessage_init_zero;

  /* Create a stream that will write to our buffer. */
  pb_ostream_t ostream = pb_ostream_from_buffer(requestbuffer, sizeof(requestbuffer));

  /* indicate that speed fields will contain values */
  request.has_speed_l = true;
  request.has_speed_r = true;

  /* fill in the message fields */
  request.speed_l = static_cast<float>(current_motor_command_.left_velocity);
  request.speed_r = static_cast<float>(current_motor_command_.right_velocity);

  /* encode the protobuffer */
  bool status = pb_encode(&ostream, RequestMessage_fields, &request);
  size_t message_length = ostream.bytes_written;

  /* check for any errors.. */
  if (!status)
  {
    ROS_ERROR_STREAM("Encoding failed: " << PB_GET_ERROR(&ostream));
    ros::shutdown();
  }

  /* Send the message strapped to a pigeon's leg! */
  (*sock_).sendMessage(reinterpret_cast<char*>(requestbuffer), message_length);
}

void MotorController::recieveResponse()
{
  /* Read response from the server */
  size_t n;             // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
  uint8_t buffer[256];  // buffer to read response into

  memset(buffer, 0, sizeof(buffer));
  /* read from the buffer */
  n = (*sock_).readMessage(buffer);  // blocks until data is read

  if (n == 0)
  {
    ROS_ERROR_STREAM("Connection closed by server");
    ros::shutdown();
  }

  /* Allocate space for the decoded message. */
  ResponseMessage response = ResponseMessage_init_zero;

  /* Create a stream that reads from the buffer. */
  pb_istream_t istream = pb_istream_from_buffer(buffer, n);

  /* decode the message. */
  bool status = pb_decode(&istream, ResponseMessage_fields, &response);

  /* check for any errors.. */
  if (!status)
  {
    ROS_ERROR_STREAM("Decoding Failed: " << PB_GET_ERROR(&istream));
    ros::shutdown();
  }

  publishResponse(response);

  ROS_INFO_STREAM_THROTTLE(log_period_, "Rate: " << 1 / response.dt_sec << "hz.");
}

void MotorController::publishResponse(const ResponseMessage& response)
{
  /* update the exponentially weighted moving voltage average and publish */
  std_msgs::Float64 battery_msg;
  battery_avg_ = (battery_alpha_ * battery_avg_) + ((1 - battery_alpha_) * response.voltage);
  battery_msg.data = battery_avg_;
  battery_pub_.publish(battery_msg);

  if (battery_avg_ < min_battery_voltage_)
  {
    ROS_ERROR_STREAM_THROTTLE(log_period_, "Battery voltage dangerously low:"
                                               << "\n\tCurr. Voltage: " << battery_avg_
                                               << "\n\tMin. Voltage: " << min_battery_voltage_);
  }

  std_msgs::Bool enabled_msg;
  enabled_msg.data = response.estop;
  enabled_pub_.publish(enabled_msg);

  /* publish encoder feedback */
  igvc_msgs::velocity_pair enc_msg;
  enc_msg.left_velocity = response.speed_l;
  enc_msg.right_velocity = response.speed_r;
  enc_msg.duration = response.dt_sec;
  enc_msg.header.stamp = ros::Time::now() - ros::Duration(response.dt_sec);
  enc_pub_.publish(enc_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  MotorController motor_controller(&nh);
  return 0;
}
