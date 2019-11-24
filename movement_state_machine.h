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
typedef enum
{
    E_MOVEMENT_IDLE,
    E_MOVEMENT_LOOKING_FOR_FACE,
    E_MOVEMENT_FACE_DEBOUNCE,
    E_MOVEMENT_FACE_IN_MOTION,
    E_MOVEMENT_ARM_IN_MOTION,
    E_MOVEMENT_WAIT_FOR_SERVO_RESET,
    E_MOVEMENT_COUNT
}T_MOVEMENT_STATE;

typedef enum
{
   E_CENTER_IN_ROI,
   E_CENTER_LEFT_OF_ROI,
   E_CENTER_RIGHT_OF_ROI,
   E_NO_ROI,
   E_RESET_SERVO
}T_ROI_FROM_CENTER; 

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
extern void Movement_StateMachineInit(void);
extern T_MOVEMENT_STATE Movement_ManageStateMachine(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint); 
extern void send_ServoPosition(T_ROI_FROM_CENTER centerRelPos);
extern void trigger_ArmMotion(void);

#endif /*MOVEMENT_STATE_MACHINE_*/
