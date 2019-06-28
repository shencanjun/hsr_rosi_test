#include "robot_enable_interface.h"
#include "ros/ros.h"
#include "hsr_rosi_test/ClearFaultSrv.h"
#include "hsr_rosi_test/SetEnableSrv.h"
#include "hsr_rosi_test/StopMoveSrv.h"

using industrial_robot_client::robot_enable_interface::RobotEnableInterface;

RobotEnableInterface *rsi;

bool set_enable_srv_callback(hsr_rosi_test::SetEnableSrv::Request &req,hsr_rosi_test::SetEnableSrv::Response &res)
{
    if(req.enable)
    {
        rsi = new RobotEnableInterface();
       rsi->set_mssage_type(2600);
    }
    else
    {
        rsi = new RobotEnableInterface();
        rsi->set_mssage_type(2601);
    }
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}
bool clear_fault_srv_callback(hsr_rosi_test::ClearFaultSrv::Request &req, hsr_rosi_test::ClearFaultSrv::Response &res)
{
    rsi = new RobotEnableInterface();
    rsi->set_mssage_type(2603);
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}

bool stop_move_srv_callback(hsr_rosi_test::StopMoveSrv::Request &req, hsr_rosi_test::StopMoveSrv::Response &res)
{
    rsi = new RobotEnableInterface();
    rsi->set_mssage_type(2602);
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "hsr_robot_interface");

  ros::NodeHandle n_rosi;
  //rsi = new RobotEnableInterface();
  //rsi->init();
  ros::ServiceServer set_enable_srv = n_rosi.advertiseService("set_robot_enable",&set_enable_srv_callback);
  ros::ServiceServer stop_move_srv = n_rosi.advertiseService("stop_robot_moving",&stop_move_srv_callback);
  ros::ServiceServer clear_fault_srv = n_rosi.advertiseService("clear_robot_fault",&clear_fault_srv_callback);
  ros::spin();
  return 0;
}
