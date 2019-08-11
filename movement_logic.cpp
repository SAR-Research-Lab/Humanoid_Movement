
/*****************************************************
 *              Header Includes                      *
 *****************************************************/
#include <ros/ros.h>
#include "dynamixel_control/ServoPosition.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/Point.h"

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/

static dynamixel_control::ServoPosition servoPositionState; 
static ros:: Subscriber roi_sub; 
static ros:: Subscriber point_sub;
static ros:: Publisher servo_pub;
static geometry_msgs::Point imageCenterPoint;
static sensor_msgs::RegionOfInterest currentRoi; 

/*****************************************************
 *            Definitions                            *
 *****************************************************/

typedef enum
{
   E_CENTER_IN_ROI,
   E_CENTER_LEFT_OF_ROI,
   E_CENTER_RIGHT_OF_ROI,
   E_NO_ROI
}T_ROI_FROM_CENTER; 

/*****************************************************
 *              Prototypes                           *
 *****************************************************/

static T_ROI_FROM_CENTER get_CenterRelativePos(void); 
static void send_ServoPosition(T_ROI_FROM_CENTER centerRelPos); 

/*****************************************************
 *              Topic callback                       *
 *              for getting the region               *
 *              of interest where the face           *
 *              has been detected                    *
 *****************************************************/
static void roiCallback(sensor_msgs::RegionOfInterest myRoi)
{
#ifdef DEBUG    
    ROS_INFO("ROI x: %d", myRoi.x_offset);  
    ROS_INFO("ROI y: %d", myRoi.y_offset);
    ROS_INFO("ROI w: %d", myRoi.width);  
    ROS_INFO("ROI h: %d", myRoi.height);
#endif
    currentRoi = myRoi; 

}

/*****************************************************
 *              pointCallback:                       *
 *              Topic callback                       * 
 *              for the center point                 *
 *              of the image                         *
 *****************************************************/
static void pointCallback(geometry_msgs::Point myPoint)
{
#ifdef DEBUG
    ROS_INFO("POINT x: %f", myPoint.x);  
    ROS_INFO("POINT y: %f", myPoint.y); 
#endif
	imageCenterPoint = myPoint; 
}


/*****************************************************
 *              get_CenterRelativePos                *
 *              Determines relative position         *
 *              of ROI from center                   *
 *              of the image                         *
 *****************************************************/
static T_ROI_FROM_CENTER get_CenterRelativePos(void)
{
    T_ROI_FROM_CENTER result; 

    /* Local Inits */
    result = E_NO_ROI; 
    
    if((currentRoi.x_offset != 0) && (currentRoi.y_offset != 0) &&
       (currentRoi.width != 0) && (currentRoi.height != 0))
    {
        /* Check if image center is bounded in the region of interest */
        if((imageCenterPoint.x > currentRoi.x_offset) &&
           (imageCenterPoint.x < currentRoi.x_offset + currentRoi.width) &&
           (imageCenterPoint.y > currentRoi.y_offset) &&
           (imageCenterPoint.y < currentRoi.y_offset + currentRoi.height))
        {
            result = E_CENTER_IN_ROI; 
        }
        else if(imageCenterPoint.x > (currentRoi.x_offset + currentRoi.width))
        {
            result = E_CENTER_RIGHT_OF_ROI; 
        }
        else
        {
            result = E_CENTER_LEFT_OF_ROI; 
        }
    }
    
    return(result); 
}

/*****************************************************
 *              send_ServoPosition                   *
 *              Sends servo information              *
 *              based on various factors             *
 *                                                   *
 *****************************************************/
static void send_ServoPosition(T_ROI_FROM_CENTER centerRelPos)
{
    switch(centerRelPos)
    {
        case E_CENTER_IN_ROI:
            ROS_INFO("STOP!"); 
            break;
        case E_CENTER_LEFT_OF_ROI:
            ROS_INFO("TURN LEFT!");
            break;
        case E_CENTER_RIGHT_OF_ROI:
            ROS_INFO("TURN RIGHT!");
            break;
        case E_NO_ROI:
        default:
            ROS_INFO("NO FACE!");
            break;

    }

}

/*****************************************************
 *              Main Entry Point                     *
 *****************************************************/
int main(int argc, char **argv) 
{ 
  T_ROI_FROM_CENTER centerRelPos; 

  /* Local Inits */
  centerRelPos = E_NO_ROI; 
   
  /* Initialize the ROS system. */
  ros::init(argc, argv, "movement_logic_rebecca");

  /* Establish this program as a ROS node. */
  ros::NodeHandle nh;

  /* Set up the subscriber for the face detected region of interest and center point */
  roi_sub   = nh.subscribe("/roi", 100, roiCallback);
  point_sub = nh.subscribe("/geometry_msgs", 100, pointCallback);

  servo_pub = nh.advertise<dynamixel_control::ServoPosition>("dynamixel_control_rebecca", 100);

   /* Set up the subscriber for the face detection results */ 

   /* Set up the rate */
   ros:: Rate rate(10000); 
    
   while(ros::ok())
   { 
     /* Trigger callback to fire and grab latest data */
     ros::spinOnce();   
     
     /* Get the relative position of the image center point to the region of interest 
      * to determine which action to take.
      */
      centerRelPos = get_CenterRelativePos(); 
      send_ServoPosition(centerRelPos); 
         
     //ROS_INFO("Current position: %d", servoPositionState.servoGoalPosition);
     rate.sleep();  
   }
}
