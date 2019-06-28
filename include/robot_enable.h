#ifndef ROBOT_ENABLE_H
#define ROBOT_ENABLE_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "hsr_rosi_test/RobotEnable.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "hsr_rosi_test/RobotEnable.h"
#endif

namespace industrial
{
namespace robot_enable
{

/**
 * \brief Enumeration mirrors industrial_msgs/RobotMode definition
 *
 */
//namespace RobotModes
//{
//enum RobotMode
//{
//  UNKNOWN = -1,

//  MANUAL = 1, AUTO = 2,
//};

//#ifdef ROS
hsr_rosi_test::RobotEnable toROSMsgEnum(bool enable);
//#endif

//}
//typedef RobotModes::RobotMode RobotMode;

/**
 * \brief Enumeration mirrors industrial_msgs/TriState definition.
 * NOTE: The TS prefix is needed because the ON and TRUE value collide
 * with other defined types on some systems.
 *
 */
//namespace TriStates
//{

//enum TriState
//{
//  TS_UNKNOWN = -1,
//  // These values must all be the same
//  TS_TRUE = 1,   TS_ON = 1,  TS_ENABLED = 1,  TS_HIGH = 1,
//  // These values must all be the same
//  TS_FALSE = 0,   TS_OFF = 0,  TS_DISABLED = 0,  TS_LOW = 0
//};

//#ifdef ROS
//int toROSMsgEnum(TriStates::TriState state);
//#endif

//}
//typedef TriStates::TriState TriState;

/**
 * \brief Class encapsulated robot status data.  The robot status data is
 * meant to mirror the industrial_msgs/RobotStatus message.
 *
 *
 * The byte representation of a robot status is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   drives_powered      (industrial::shared_types::shared_int)    4  bytes
 *   e_stopped           (industrial::shared_types::shared_int)    4  bytes
 *   error_code          (industrial::shared_types::shared_int)    4  bytes
 *   in_error            (industrial::shared_types::shared_int)    4  bytes
 *   in_motion           (industrial::shared_types::shared_int)    4  bytes
 *   mode                (industrial::shared_types::shared_int)    4  bytes
 *   motion_possible     (industrial::shared_types::shared_int)    4  bytes
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class RobotEnable : public industrial::simple_serialize::SimpleSerialize
{
public:
/**
 * \brief Default constructor
 *
 * This method creates empty data.
 *
 */
RobotEnable(void);
/**
 * \brief Destructor
 *
 */
~RobotEnable(void);

/**
 * \brief Initializes an empty robot status
 *
 */
void init();

/**
 * \brief Initializes a full robot status message
 *
 */
void init(bool enable);

bool getRobotEnable()
{
  return bool(drives_powered_);
}

/**
 * \brief Copies the passed in value
 *
 * \param src (value to copy)
 */
void copyFrom(RobotEnable &src);

/**
 * \brief == operator implementation
 *
 * \return true if equal
 */
bool operator==(RobotEnable &rhs);

void setDrivesEnable(bool drivesPowered)
{
  this->drives_powered_ = drivesPowered;
}

// Overrides - SimpleSerialize
bool load(industrial::byte_array::ByteArray *buffer);
bool unload(industrial::byte_array::ByteArray *buffer);
unsigned int byteLength()
{
  return 7 * sizeof(industrial::shared_types::shared_int);
}

private:
/**
 * \brief Drive power state (see TriStates::TriState)
 */
bool drives_powered_;

};

}
}

#endif // ROBOT_ENABLE_H
