#ifndef FLATHEADERS
#include "robot_enable_message.h"
#include "robot_enable.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "robot_status_message.h"
#include "robot_status.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::robot_enable;

namespace industrial
{
namespace robot_enable_message
{

RobotEnableMessage::RobotEnableMessage(void)
{
  this->init();
}

RobotEnableMessage::~RobotEnableMessage(void)
{

}

bool RobotEnableMessage::init(industrial::simple_message::SimpleMessage & msg)
{
    return true;
}

bool RobotEnableMessage::init(industrial::simple_message::SimpleMessage & msg,int mssg_type)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init(mssg_type);
  this->setCommType(industrial::simple_message::CommTypes::TOPIC);//

  if (data.load(this->enable_))//unload(this->enable_
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload robot enable data");
  }
  return rtn;
}

void RobotEnableMessage::init(industrial::robot_enable::RobotEnable & enable)
{
    return;
}

void RobotEnableMessage::init(industrial::robot_enable::RobotEnable & enable,int mssg_type)
{
  this->init(mssg_type);
  this->enable_.copyFrom(enable);
}

void RobotEnableMessage::init()
{
    return;
}

void RobotEnableMessage::init(int mssg_type)
{
  this->setMessageType(mssg_type);
  this->enable_.init();
}

bool RobotEnableMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot enable message load");
  if (buffer->load(this->enable_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load robot enable data");
  }
  return rtn;
}

bool RobotEnableMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot enable message unload");

  if (buffer->unload(this->enable_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload robot enable data");
  }
  return rtn;
}

}
}

