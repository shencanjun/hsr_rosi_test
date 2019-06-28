/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
#ifndef FLATHEADERS
#include "robot_enable.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "robot_status.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

#ifdef ROS
// Files below used to translate between ROS messages enums and
// enums defined in this file
#include "industrial_msgs/RobotMode.h"
#include "industrial_msgs/TriState.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace robot_enable
{

//namespace RobotModes
//{

//#ifdef ROS

hsr_rosi_test::RobotEnable toROSMsgEnum(bool enable)
{
  hsr_rosi_test::RobotEnable Enable;
  switch (enable)
  {
    case true:
      Enable.enable = 1;
      break;
    case false:
      Enable.enable = 0;
      break;
    //case RobotModes::UNKNOWN:
    return Enable;
  }
  //return ::RobotMode::UNKNOWN;

}
//;

//#endif


RobotEnable::RobotEnable(void)
{
  this->init();
}
RobotEnable::~RobotEnable(void)
{

}

void RobotEnable::init()
{
  this->init(false);
}

void RobotEnable::init(bool enable)
{
  this->setDrivesEnable(enable);
}

void RobotEnable::copyFrom(RobotEnable &src)
{
  this->setDrivesEnable(src.getRobotEnable());
}

bool RobotEnable::operator==(RobotEnable &rhs)
{
  return this->drives_powered_ == rhs.drives_powered_;
}

bool RobotEnable::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing robot status load");

  if (buffer->load(this->drives_powered_))
  {

    LOG_COMM("Robot status successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("Robot status not loaded");
    rtn = false;
  }

  return rtn;
}

bool RobotEnable::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing robot status unload");
  if (buffer->unload(this->drives_powered_))
  {

    rtn = true;
    LOG_COMM("Robot status successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload robot status");
    rtn = false;
  }

  return rtn;
}

}
}

