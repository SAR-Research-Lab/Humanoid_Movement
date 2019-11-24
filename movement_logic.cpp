
/*****************************************************
 *              Header Includes                      *
 *****************************************************/
#include <std_msgs/Bool.h>
#include <dynamixel_control/ServoPosition.h>
#include "movement_state_machine.h"

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/

static dynamixel_control::ServoPosition servoPositionState; 
static ros:: Subscriber roi_sub; 
static ros:: Subscriber point_sub;
static ros:: Publisher servo_pub;
static ros:: Publisher arm_pub;
static std_msgs::Bool arm_trigger;
static geometry_msgs::Point imageCenterPoint;
static sensor_msgs::RegionOfInterest currentRoi; 
static T_MOVEMENT_STATE current_state; 

/*****************************************************
 *            Definitions                            *
 *****************************************************/

#define DEBUG 

typedef enum
{
    E_PRINT_STOP,
    E_PRINT_TURN_LEFT,
    E_PRINT_TURN_RIGHT,
    E_PRINT_NO_ROI,
    E_PRINT_ROI_DATA,
    E_PRINT_POINT_DATA,
    E_PRINT_CURRENT_STATE,
    E_PRINT_CENTER_SERVO, 

}T_DEBUG_PRINT_MSG; 

static char* state_array[E_MOVEMENT_COUNT] =
{
    "E_MOVEMENT_IDLE",
    "E_MOVEMENT_LOOKING_FOR_FACE",
    "E_MOVEMENT_FACE_DEBOUNCE",
    "E_MOVEMENT_FACE_IN_MOTION",
    "E_MOVEMENT_ARM_IN_MOTION",
    "E_MOVEMENT_WAIT_FOR_SERVO_RESET" 
    
};


/*****************************************************
 *              Prototypes                           *
 *****************************************************/ 
static void debug_Print(T_DEBUG_PRINT_MSG myMsg);

/*****************************************************
 *              Topic callback                       *
 *              for getting the region               *
 *              of interest where the face           *
 *              has been detected                    *
 *****************************************************/
static void roiCallback(sensor_msgs::RegionOfInterest myRoi)
{
    ROS_INFO("AREA: %d", myRoi.width*myRoi.height); 
    currentRoi = myRoi;
    debug_Print(E_PRINT_ROI_DATA); 
 
}

/*****************************************************
 *              pointCallback:                       *
 *              Topic callback                       * 
 *              for the center point                 *
 *              of the image                         *
 *****************************************************/
static void pointCallback(geometry_msgs::Point myPoint)
{
    imageCenterPoint = myPoint;
    debug_Print(E_PRINT_POINT_DATA);  
}

/*****************************************************
 *              trigger_ArmMotion                    *
 *              Sends the trigger command            *
 *              to the arm node to send the          *
 *              wave commands                        *
 *****************************************************/
void trigger_ArmMotion(void)
{
    arm_trigger.data = 1; 
    arm_pub.publish(arm_trigger);
}

/*****************************************************
 *              send_ServoPosition                   *
 *              Sends servo information              *
 *              based on various factors             *
 *                                                   *
 *****************************************************/
void send_ServoPosition(T_ROI_FROM_CENTER centerRelPos)
{
    /* Set default values */
    servoPositionState.servoClockWiseRotation = 0;  /* Counterclockwise */
    servoPositionState.servoDegreeRotation = 0;     /* Don't move */
    servoPositionState.servoSetCenter = 0;          /* Dont't center */

    switch(centerRelPos)
    {
        case E_CENTER_IN_ROI:
            debug_Print(E_PRINT_STOP); 
            break;
        case E_CENTER_LEFT_OF_ROI:
            servoPositionState.servoClockWiseRotation = 0; 
            servoPositionState.servoDegreeRotation = 1; 
            servo_pub.publish(servoPositionState); 
            debug_Print(E_PRINT_TURN_LEFT);
            break;
        case E_CENTER_RIGHT_OF_ROI:
            servoPositionState.servoClockWiseRotation = 1; 
            servoPositionState.servoDegreeRotation = 1;
            servo_pub.publish(servoPositionState);
            debug_Print(E_PRINT_TURN_RIGHT);
            break;
        case E_RESET_SERVO:
            servoPositionState.servoSetCenter = 1; 
            servo_pub.publish(servoPositionState);
            debug_Print(E_PRINT_CENTER_SERVO);
            break;
        case E_NO_ROI:
        default:
            debug_Print(E_PRINT_NO_ROI);
            break;

    }

}


/*****************************************************
 *              Main Entry Point                     *
 *****************************************************/
int main(int argc, char **argv) 
{ 
   /* Initialize the ROS system. */
   ros::init(argc, argv, "movement_logic_rebecca");

   /* Establish this program as a ROS node. */
   ros::NodeHandle nh;

   /* Set up the subscriber for the face detected region of interest and center point */
   roi_sub   = nh.subscribe("/roi", 100, roiCallback);
   point_sub = nh.subscribe("/geometry_msgs", 100, pointCallback);
   
   /* Set up the publishers for the servo and the humanoid arm mover */
   servo_pub = nh.advertise<dynamixel_control::ServoPosition>("dynamixel_control_rebecca", 100);
   arm_pub   = nh.advertise<std_msgs::Bool>("arm_commander", 100); 

   /* Set up the subscriber for the face detection results */ 

   /* Set up the rate */
   ros:: Rate rate(10); 

   /* Set up the state machine */
   Movement_StateMachineInit(); 
    
   while(ros::ok())
   { 
      /* Trigger callback to fire and grab latest data */
      ros::spinOnce();   

      /* Send data to the state machine */
      current_state = Movement_ManageStateMachine(currentRoi, imageCenterPoint);

      debug_Print(E_PRINT_CURRENT_STATE); 
         
      rate.sleep();  
   }
}

/*****************************************************
 *              debug_Print                          *
 *                                                   *
 *                                                   *
 *****************************************************/
static void debug_Print(T_DEBUG_PRINT_MSG myMsg)
{
#ifdef DEBUG    
    switch(myMsg)
    {
        case E_PRINT_STOP: 
            ROS_INFO("STOP!");
            break;
        case E_PRINT_TURN_LEFT:
            ROS_INFO("TURN LEFT!");
            break;
        case E_PRINT_TURN_RIGHT:
            ROS_INFO("TURN RIGHT!");
            break;
        case E_PRINT_NO_ROI:
            ROS_INFO("NO FACE!");
            break; 
        case E_PRINT_ROI_DATA:
            ROS_INFO("ROI x: %d", currentRoi.x_offset);  
            ROS_INFO("ROI y: %d", currentRoi.y_offset);
            ROS_INFO("ROI w: %d", currentRoi.width);  
            ROS_INFO("ROI h: %d", currentRoi.height);
            break; 
        case E_PRINT_POINT_DATA:
            ROS_INFO("POINT x: %f", imageCenterPoint.x);  
            ROS_INFO("POINT y: %f", imageCenterPoint.y);
            break;
        case E_PRINT_CURRENT_STATE:
            ROS_INFO("CURRENT State: %s", state_array[current_state]); 
            break;
        case E_PRINT_CENTER_SERVO:
            ROS_INFO("CENTERING SERVO!"); 
            break; 
        default:
            break;
    }
#endif
}
