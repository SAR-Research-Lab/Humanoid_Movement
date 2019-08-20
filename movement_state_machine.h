/* Include interfaces */

#ifndef MOVEMENT_STATE_MACHINE_
#define MOVEMENT_STATE_MACHINE_

/*****************************************************
 *              Header Includes                      *
 *****************************************************/
#include <ros/ros.h>
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/Point.h"

/*****************************************************
 *            Definitions                            *
 *****************************************************/

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
extern void Movement_StateMachineInit(void);
extern void Movement_ManageStateMachine(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint); 

#endif /*MOVEMENT_STATE_MACHINE_*/
