#include "ros/ros.h"
#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <deque>
#include <vector>

using namespace std;

#include "SerialStream.hpp"

using namespace ass;

bool arduinoReady = false;

void processMsg(SerialStream::Message &iMsg)
{
  arduinoReady = true;
  ROS_INFO("Host Received Message with %d packets", iMsg.packets.size());
  for(auto pkt : iMsg.packets) {
    switch(pkt->type) {
      case STRING: {
        char *ptr = pkt->getData<char>();
        ROS_INFO(" STRING: %s", ptr);
        } break;
      case INT32: {
        uint32_t *ptr = pkt->getData<uint32_t>();
        ROS_INFO(" INT32 array: [%d, %d, %d, %d ...]", ptr[0], ptr[1], ptr[2], ptr[3]);
        } break;
      case FLOAT32: {
        float *ptr = pkt->getData<float>();
        ROS_INFO(" FLOAT32 array: [%f, %f, %f, %f ...]", ptr[0], ptr[1], ptr[2], ptr[3]);
        } break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle n;

  ROS_INFO("Opening serial connection to Arduino");
  int serial_fd = open("/dev/ttyACM0", O_RDWR);
  if(serial_fd >= 0) {
    ROS_INFO("Success!");
  } else {
    ROS_INFO("Failure!");
    exit(-1);
  }

  struct termios portSettings;
  if(tcgetattr(serial_fd, &portSettings) < 0) {
    ROS_INFO("Failed to get port settings");
    close(serial_fd);
    serial_fd = -1;
    exit(-1);
  } else {
    portSettings.c_cflag &= ~PARENB; // Disable parity
    portSettings.c_cflag &= ~CSTOPB; // Use only 1 stop bit
    portSettings.c_cflag &= ~CSIZE;
    portSettings.c_cflag |= CS8; // 8-bit bytes
    portSettings.c_cflag &= ~CRTSCTS; // Disable flow control
    portSettings.c_cflag |= CREAD | CLOCAL;
    portSettings.c_lflag &= ~ICANON; // Non-cononical mode
    portSettings.c_lflag &= ~ECHO;  // Disable echo
    portSettings.c_lflag &= ~ECHOE; // Disable erasure
    portSettings.c_lflag &= ~ECHONL; // Disable new-line echo
    portSettings.c_lflag &= ~ISIG; // Disable ISIG, INTR, QUIT, SUSP
    portSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    portSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    portSettings.c_oflag &= ~(OPOST | ONLCR);
    portSettings.c_cc[VMIN] = 0;
    portSettings.c_cc[VTIME] = 10; // Block of 1 second
    cfsetispeed(&portSettings, B115200);
    cfsetospeed(&portSettings, B115200);
    if(tcsetattr(serial_fd, TCSANOW, &portSettings) < 0) {
      ROS_INFO("Failed to set baud rate");
      close(serial_fd);
      serial_fd = -1;
      exit(-1);
    }
  }

  sleep(2);
  tcflush(serial_fd, TCIOFLUSH);

  SerialStream ss(serial_fd);

  ss.recycleRecvMessage(new SerialStream::Message());

  ROS_INFO("Spinning forever");
  char buff;

  ros::Time rosThrottle = ros::Time::now();
  ros::Time msgThrottle = rosThrottle;

  ros::Rate loopRate(60);

  uint32_t testIntData[10];
  float testFloatData[10];

  for(int i = 0; i < 10; i++) {
    testIntData[i] = uint32_t(i);
  }
  for(int i = 0; i < 10; i++) {
    testFloatData[i] = float(i);
  }

  while(ros::ok()) {
    ss.tick();

    SerialStream::Message *recvMsg;
    if(recvMsg = ss.popRecvMessage()) {
      processMsg(*recvMsg);
      ss.recycleRecvMessage(recvMsg);
    }

    ros::Time time = ros::Time::now();
    if((time - rosThrottle) > ros::Duration(0.1)) {
      rosThrottle = time;
      ros::spinOnce();
    }

    if(arduinoReady && ((time - msgThrottle) > ros::Duration(0.0666))) {
      msgThrottle = time;

      SerialStream::Message *msg = ss.getSendMessage();
      msg->setPacketCount(3);

      SerialStream::Packet *pkt0 = msg->packets[0];
      pkt0->type = STRING;
      pkt0->byteSize = 5;
      pkt0->resizeIfNeeded();
      memmove(pkt0->data, "PING", pkt0->byteSize);

      SerialStream::Packet *pkt1 = msg->packets[1];
      pkt1->type = INT32;
      pkt1->byteSize = 10 * sizeof(uint32_t);
      pkt1->resizeIfNeeded();
      memmove(pkt1->data, (char *) testIntData, pkt1->byteSize);

      SerialStream::Packet *pkt2 = msg->packets[2];
      pkt2->type = FLOAT32;
      pkt2->byteSize = 10 * sizeof(float);
      pkt2->resizeIfNeeded();
      memmove(pkt2->data, (char *) testFloatData, pkt2->byteSize);

      ss.queueSendMessage(msg);

    }
    // loopRate.sleep();
  }
}
