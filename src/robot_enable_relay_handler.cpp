/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_enable_relay_handler.h"
#include "hsr_rosi_test/RobotEnable.h"
#include "simple_message/log_wrapper.h"
#include "robot_enable.h"
#include "ros/ros.h"

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace industrial::robot_enable;
using namespace industrial::robot_enable_message;

namespace industrial_robot_client
{
namespace robot_enable_relay_handler
{

bool RobotEnableRelayHandler::init(SmplMsgConnection* connection,int mssg_type)
{
  this->pub_robot_enable_ = this->node_.advertise<hsr_rosi_test::RobotEnable>("robot_enable", 1);
  ROS_ERROR("3__________________-_____111-1-1--1-1-1-1-1-1-1-1-1-");
   init((int)2600, connection);
   return(sendsimple_message(mssg_type));
}

bool RobotEnableRelayHandler::sendsimple_message(int mssg_type)
{
    SimpleMessage msg;
     return internal(msg,mssg_type);
}

bool RobotEnableRelayHandler::internal(SimpleMessage& in,int mssg_type)
{
    RobotEnableMessage enable_msg;
   // industrial::robot_enable enable_s = true;
    //std::cout<<"3__111111"<<std::endl;
    ROS_ERROR("3__111111");
    if (!enable_msg.init(in,mssg_type))
    {
      LOG_ERROR("Failed to initialize enable message");
      return false;
    }

    return internal(enable_msg);
}

bool RobotEnableRelayHandler::internal(RobotEnableMessage & in)
{
    hsr_rosi_test::RobotEnable enable;
    bool ret = false;

     std::cout<<"3__222222"<<std::endl;
     ROS_ERROR("3__222222");
   // enable.header.stamp = ros::Time::now();
    enable = toROSMsgEnum(in.enable_.getRobotEnable());
    this->pub_robot_enable_.publish(enable);

    // Reply back to the controller if the sender requested it.
    if (CommTypes::TOPIC == in.getCommType())
    {
      SimpleMessage topic;
      industrial::byte_array::ByteArray bytee;
      std::cout<<"3__333333"<<std::endl;
      ROS_ERROR("3__333333");
      //in.toReply(topic, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
      in.toTopic(topic);
      ret =  this->getConnection()->sendMsg(topic);
      bytee = topic.getData();
      std::cout<<"topic= "<<topic.getMessageType()<<std::endl;
      std::cout<<"bytee= "<<bytee.getBufferSize()<<std::endl;
      std::cout<<"topic= "<<topic.getCommType()<<std::endl;
      std::cout<<"sendMsg.ret = "<<ret<<std::endl;
    }
    return ret;
}

bool RobotEnableRelayHandler::internalCB(SimpleMessage& in)
{
    return true;
}

bool RobotEnableRelayHandler::internalCB(RobotEnableMessage & in)
{
   return true;
}

}
}

