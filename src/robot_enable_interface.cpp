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

#include "robot_enable_interface.h"
#include "industrial_utils/param_utils.h"

using industrial::smpl_msg_connection::SmplMsgConnection;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace robot_enable_interface
{

RobotEnableInterface::RobotEnableInterface()
{
  this->connection_ = NULL;
  mssg_type_ = 0;
  this->add_handler(&default_robot_enable_handler_);
}

bool RobotEnableInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  bool ret =  default_tcp_connection_.init(ip_addr, port);
  std::cout<<"net ret = "<<ret<<std::endl;
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool RobotEnableInterface::init(SmplMsgConnection* connection)
{
  this->connection_ = connection;
  if(!this->connection_->isConnected()){
      connection_->makeConnect();
    //  if (!manager_.init(connection_))
     //   return false;
  }
  if (!default_robot_enable_handler_.init(connection_,mssg_type_))
      return false;
  this->add_handler(&default_robot_enable_handler_);
    return true;
}

bool RobotEnableInterface::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->connection_ = connection;
  connection_->makeConnect();
  // initialize message-manager
  if (!manager_.init(connection_))
    return false;
  // initialize default handlers
//  if (!default_joint_handler_.init(connection_, joint_names_))
//    return false;
//  this->add_handler(&default_joint_handler_);
  //if (!default_robot_enable_handler_.init(connection_))
  //    return false;
  //this->add_handler(&default_robot_enable_handler_);
  return true;
}

bool RobotEnableInterface::set_mssage_type(int mssg_type)
{
    if(mssg_type ==  0)
       return false;
    else
       mssg_type_ = mssg_type;
    return true;
}

//bool RobotEnableInterface::set_enable_srv_callback(hsr_rosi_test::SetEnableSrv::Request &req,hsr_rosi_test::SetEnableSrv::Response &res)
//{
//    if(!this->connection_->isConnected()){
//        connection_->makeConnect();
//        if (!manager_.init(connection_))
//          return false;
//    }
//    if(req.enable)
//    {
//        if (!default_robot_enable_handler_.init(connection_,2600))
//            return false;
//    }
//    else
//    {
//        if (!default_robot_enable_handler_.init(connection_,2601))
//            return false;
//    }
//    ROS_ERROR("here..................");
//    this->add_handler(&default_robot_enable_handler_);
//    res.finsh = true;
//    return true;
//}
//bool RobotEnableInterface::clear_fault_srv_callback(hsr_rosi_test::ClearFaultSrv::Request &req, hsr_rosi_test::ClearFaultSrv::Response &res)
//{
//    if(!this->connection_->isConnected()){
//        connection_->makeConnect();
//        if (!manager_.init(connection_))
//          return false;
//    }
//    if (!default_robot_enable_handler_.init(connection_,2603))
//        return false;
//    this->add_handler(&default_robot_enable_handler_);
//    ROS_ERROR("here..................");
//    res.finsh = true;
//    return true;
//}

//bool RobotEnableInterface::stop_move_srv_callback(hsr_rosi_test::StopMoveSrv::Request &req, hsr_rosi_test::StopMoveSrv::Response &res)
//{
//    if(this->connection_->isConnected()){
//        connection_->makeConnect();
//        if (!manager_.init(connection_))
//          return false;
//    }
//    if (!default_robot_enable_handler_.init(connection_,2602))
//        return false;
//    this->add_handler(&default_robot_enable_handler_);
//    res.finsh = true;
//    return true;
//}

//void RobotEnableInterface::start()
//{
//    set_enable_srv = n_rosi.advertiseService("set_robot_enable",&RobotEnableInterface::set_enable_srv_callback, this);
//    stop_move_srv = n_rosi.advertiseService("stop_robot_moving",&RobotEnableInterface::stop_move_srv_callback, this);
//    clear_fault_srv = n_rosi.advertiseService("clear_robot_fault",&RobotEnableInterface::clear_fault_srv_callback,this);
//    ros::spin();
//}

void RobotEnableInterface::run()
{
  manager_.spin();
}

} // robot_state_interface
} // industrial_robot_client
