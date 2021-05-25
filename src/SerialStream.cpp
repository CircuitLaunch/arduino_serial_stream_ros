#include "SerialStream.hpp"

using namespace ass;

SerialStream::Packet::~Packet()
{
  if(manage) if(data) delete [] data;
}

uint16_t SerialStream::Packet::size()
{
  switch(type) {
    case STRING:
      return byteSize;
    case INT32:
    case FLOAT32:
      return byteSize >> 2;
  }
  return 0;
}

void SerialStream::Packet::resizeIfNeeded()
{
  if(byteSize > capacity) {
    if(manage && data) delete [] data;
    manage = true;
    data = new uint8_t[byteSize];
    capacity = byteSize;
  }
}

void SerialStream::Packet::convert()
{
  if(type == STRING) return;
  int i = byteSize >> 2;
  uint32_t *ptr = (uint32_t *) data;
  while(i--) ptr[i] = htonl(ptr[i]);
}

vector<SerialStream::Packet *> SerialStream::Message::packetPool;

SerialStream::Message::Message()
: packets() { }

SerialStream::Message::~Message()
{
  while(packets.size()) {
    SerialStream::Packet *pkt = packets.back();
    packets.pop_back();
    SerialStream::Message::packetPool.push_back(pkt);
  }
}

void SerialStream::Message::setPacketCount(uint8_t iCount)
{
  while(packets.size() < iCount) {
    SerialStream::Packet *pkt;
    if(packetPool.size()) {
      pkt = packetPool.back();
      packetPool.pop_back();
    } else {
      pkt = new SerialStream::Packet();
    }
    packets.push_back(pkt);
  }
  while(packets.size() > iCount) {
    SerialStream::Packet *pkt = packets.back();
    packets.pop_back();
    packetPool.push_back(pkt);
  }
}

SerialStream::SerialStream(int iFD)
: fd(iFD),
  recvMsgQueue(), recvMsgPool(), sendMsgQueue(), sendMsgPool(), recvPktCount(0), sendPktCount(0),
  recvPkt(nullptr), readPtr(nullptr), readCount(0), readState(IDLE),
  sendPkt(nullptr), writePtr(nullptr), writeCount(0), writeState(IDLE)
{ }

SerialStream::~SerialStream()
{
  while(recvMsgQueue.size()) {
    Message *m = recvMsgQueue.front();
    recvMsgQueue.pop_front();
    delete m;
  }
  while(sendMsgQueue.size()) {
    Message *m = sendMsgQueue.front();
    sendMsgQueue.pop_front();
    delete m;
  }
  while(recvMsgPool.size()) {
    Message *m = recvMsgPool.front();
    recvMsgPool.pop_back();
    delete m;
  }
  while(sendMsgPool.size()) {
    Message *m = sendMsgPool.front();
    sendMsgPool.pop_back();
    delete m;
  }
}

void SerialStream::queueSendMessage(SerialStream::Message *iMsg)
{
  sendMsgQueue.push_back(iMsg);
}

SerialStream::Message *SerialStream::getSendMessage()
{
  if(sendMsgPool.size()) {
    SerialStream::Message *msg = sendMsgPool.back();
    sendMsgPool.pop_back();
    return msg;
  }
  return new SerialStream::Message();
}

void SerialStream::recycleRecvMessage(SerialStream::Message *iMsg)
{
  recvMsgPool.push_back(iMsg);
}

SerialStream::Message *SerialStream::popRecvMessage()
{
  if(recvMsgQueue.size()) {
    SerialStream::Message *front = recvMsgQueue.front();
    recvMsgQueue.pop_front();
    return front;
  }
  return nullptr;
}

void SerialStream::tick()
{
  switch(writeState) {
    case IDLE:
      if(sendMsgQueue.size()) {
        sendMsg = sendMsgQueue.front();
        sendMsgQueue.pop_front();
        writeState = CMD;
        writePtr = &sendMsg->cmd;
        writeCount = 1;
      }
      break;
    case CMD:
      if(!doWrite()) {
        sendPktCount = uint8_t(sendMsg->packets.size());
        writeState = PKTS;
        writePtr = &sendPktCount;
        writeCount = 1;
      } break;
    case PKTS:
      if(!doWrite()) {
        sendPktIndex = 0;
        sendPkt = sendMsg->packets[sendPktIndex];
        writeState = TYPE;
        writePtr = &sendPkt->type;
        writeCount = 1;
      } break;
    case TYPE:
      if(!doWrite()) {
        writeState = SIZE;
        netWrite16 = htons(sendPkt->byteSize);
        writePtr = (uint8_t *) &netWrite16;
        writeCount = sizeof(sendPkt->byteSize);
      }
      break;
    case SIZE:
      if(!doWrite()) {
        writeState = DATA;
        writePtr = (uint8_t *) sendPkt->data;
        writeCount = sendPkt->byteSize;
        sendPkt->convert();
      }
      break;
    case DATA:
      if(!doWrite()) {
        sendPktIndex++;
        if(sendPktIndex == sendPktCount) {
          recycleSendMessage(sendMsg);
          sendPktIndex = 0;
          sendPktCount = 0;
          writeState = IDLE;
          writePtr = nullptr;
          writeCount = 0;
          sendPkt = nullptr;
          sendMsg = nullptr;
        } else {
          sendPkt = sendMsg->packets[sendPktIndex];
          writeState = TYPE;
          writePtr = &sendPkt->type;
          writeCount = 1;
        }
      }
      break;
  }

  switch(readState) {
    case IDLE:
      if(recvMsgPool.size()) {
        // ROS_INFO("read IDLE");
        recvMsg = getRecvMessage();
        readState = CMD;
        readPtr = &recvMsg->cmd;
        readCount = 1;
      }
      break;
    case CMD:
      if(!doRead()) {
        // ROS_INFO("read CMD");
        readState = PKTS;
        readPtr = &recvPktCount;
        readCount = 1;
      }
      break;
    case PKTS:
      if(!doRead()) {
        // ROS_INFO("read PKTS");
        recvMsg->setPacketCount(recvPktCount);
        recvPktIndex = 0;
        recvPkt = recvMsg->packets[recvPktIndex];
        readState = TYPE;
        readPtr = &recvPkt->type;
        readCount = 1;
      } break;
    case TYPE:
      if(!doRead()) {
        // ROS_INFO("read TYPE = %d", recvPkt->type);
        readState = SIZE;
        readPtr = (uint8_t *) &netRead16;
        readCount = sizeof(recvPkt->byteSize);
      }
      break;
    case SIZE:
      if(!doRead()) {
        recvPkt->byteSize = ntohs(netRead16);
        // ROS_INFO("read SIZE = %d", recvPkt->byteSize);
        readState = DATA;
        recvPkt->resizeIfNeeded();
        readPtr = (uint8_t *) recvPkt->data;
        readCount = recvPkt->byteSize;
      }
      break;
    case DATA:
      if(!doRead()) {
        // ROS_INFO("read DATA");
        recvPkt->convert();
        recvPktIndex++;
        if(recvPktIndex == recvPktCount) {
          recvMsgQueue.push_back(recvMsg);
          recvPktIndex = 0;
          recvPktCount = 0;
          readState = IDLE;
          readPtr = nullptr;
          readCount = 0;
          recvPkt = nullptr;
          recvMsg = nullptr;
        } else {
          recvPkt = recvMsg->packets[recvPktIndex];
          readState = TYPE;
          readPtr = &recvPkt->type;
          readCount = 1;
        }
      }
  }
}

int SerialStream::doWrite()
{
  if(!writeCount) return 0;

  int bytesWritten = write(fd, writePtr, writeCount);
  if(bytesWritten < 0) return writeCount;

  writeCount -= bytesWritten;
  writePtr += bytesWritten;

  return writeCount;
}

int SerialStream::doRead()
{
  if(!readCount) return 0;

  int bytesAvail = 0;
  ioctl(fd, FIONREAD, &bytesAvail);
  if(!bytesAvail) return readCount;
  // ROS_INFO("Bytes available");
  int bytesRead = read(fd, readPtr, readCount);
  if(bytesRead < 0) return readCount;

  readCount -= bytesRead;
  readPtr += bytesRead;

  return readCount;
}

SerialStream::Message *SerialStream::getRecvMessage()
{
  if(recvMsgPool.size()) {
    SerialStream::Message *msg = recvMsgPool.back();
    recvMsgPool.pop_back();
    return msg;
  }
  return new SerialStream::Message();
}

void SerialStream::recycleSendMessage(SerialStream::Message *iMsg)
{
  sendMsgPool.push_back(iMsg);
}
