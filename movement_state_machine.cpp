
/*****************************************************
 *              Header Includes                      *
 *****************************************************/

#include "movement_state_machine.h" 

/*****************************************************
 *            Definitions                            *
 *****************************************************/

#define THRESHOLD_PERCENT_MIN  0.8f
#define THRESHOLD_PERCENT_MAX  1.2f
#define FACE_DEBOUNCE_TIME     5
#define ARM_MOVEMENT_TIME      20

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/
static T_MOVEMENT_STATE movement_CurrentState; 
static sensor_msgs::RegionOfInterest LastRoi; 
static double faceTimeDetected; 
static double armMovementTime;

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
static void Movement_Idle(void); 
static void Movement_SetArmInMotion(void);
static void Movement_LookForFace(sensor_msgs::RegionOfInterest myRoi); 
static void Movement_DebounceFace(sensor_msgs::RegionOfInterest myRoi);
static void Movement_SetFaceInMotion(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint);  
static T_ROI_FROM_CENTER get_CenterRelativePos(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint); 

/*****************************************************
 *              Movement_StateMachineInit            *
 *                                                   *
 *                                                   *
 *****************************************************/
void Movement_StateMachineInit(void)
{
    movement_CurrentState = E_MOVEMENT_IDLE;
    LastRoi.x_offset      = 0; 
    LastRoi.y_offset      = 0;
    LastRoi.width         = 0;
    LastRoi.height        = 0;
    faceTimeDetected      = 0; 
    armMovementTime       = 0; 

}

/*****************************************************
 *              Movement_ManageStateMachine          *
 *                                                   *
 *                                                   *
 *****************************************************/
T_MOVEMENT_STATE Movement_ManageStateMachine(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint)
{
    switch(movement_CurrentState)
    {
        case E_MOVEMENT_IDLE: 
            Movement_Idle();  
            break;
        case E_MOVEMENT_LOOKING_FOR_FACE:
            Movement_LookForFace(myRoi); 
            break;
        case E_MOVEMENT_FACE_DEBOUNCE:
            Movement_DebounceFace(myRoi); 
            break;
        case E_MOVEMENT_FACE_IN_MOTION: 
            Movement_SetFaceInMotion(myRoi, myPoint); 
            break; 
        case E_MOVEMENT_ARM_IN_MOTION:
            Movement_SetArmInMotion(); 
            break;
        default:
            break; 
    }

    return(movement_CurrentState);  
}

/*****************************************************
 *              Movement_Idle                        *
 *                                                   *
 *                                                   *
 *****************************************************/
static void Movement_Idle(void)
{
    LastRoi.x_offset      = 0; 
    LastRoi.y_offset      = 0;
    LastRoi.width         = 0;
    LastRoi.height        = 0;
    faceTimeDetected      = 0;     
    movement_CurrentState = E_MOVEMENT_LOOKING_FOR_FACE; 
}

/*****************************************************
 *              Movement_LookForFace                 *
 *                                                   *
 *                                                   *
 *****************************************************/
static void Movement_LookForFace(sensor_msgs::RegionOfInterest myRoi)
{
    /* If we have received a non-zero region of interest, then
     * we have detected a face. Save the current data and 
     * start a timer to see if this face is stable later and the 
     * one we want to latch onto. 
     */    
    if((myRoi.x_offset != 0) && (myRoi.y_offset != 0) &&
       (myRoi.width != 0) && (myRoi.height != 0))
    {
        LastRoi               = myRoi; 
        movement_CurrentState = E_MOVEMENT_FACE_DEBOUNCE; 
        faceTimeDetected      = ros::Time::now().toSec(); 
    }


}

/*****************************************************
 *              Movement_LookForFace                 *
 *                                                   *
 *                                                   *
 *****************************************************/
static void Movement_DebounceFace(sensor_msgs::RegionOfInterest myRoi)
{
    int32_t last_area_min; 
    int32_t last_x_min;
    int32_t last_y_min;
    int32_t last_area_max; 
    int32_t last_x_max;
    int32_t last_y_max;
    int32_t current_area; 
    int32_t current_x;
    int32_t current_y; 

    /* Local inits */
    last_area_min  = LastRoi.width * LastRoi.height * THRESHOLD_PERCENT_MIN; 
    last_x_min     = LastRoi.x_offset * THRESHOLD_PERCENT_MIN;
    last_y_min     = LastRoi.y_offset * THRESHOLD_PERCENT_MIN; 
    last_area_max  = LastRoi.width * LastRoi.height * THRESHOLD_PERCENT_MAX; 
    last_x_max     = LastRoi.x_offset * THRESHOLD_PERCENT_MAX;
    last_y_max     = LastRoi.y_offset * THRESHOLD_PERCENT_MAX; 
    current_area   = myRoi.width * myRoi.height; 
    current_x      = myRoi.x_offset;
    current_y      = myRoi.y_offset; 


    /* Check if the last saved region of interest is within
     * a threshold of the current region of interest,
     * 80%-120% of last value. If so, save this new 
     * region of interest and check back in x amount of 
     * time. 
     */
    if((myRoi.x_offset != 0) && (myRoi.y_offset != 0) &&
       (myRoi.width != 0) && (myRoi.height != 0))
    {
        if((current_area > last_area_min) && (current_area < last_area_max) &&
           (current_x > last_x_min)       && (current_x < last_x_max) &&
           (current_y > last_y_min)       && (current_y < last_y_max))
        {
            if((ros::Time::now().toSec() - faceTimeDetected) > FACE_DEBOUNCE_TIME)
            {
                movement_CurrentState = E_MOVEMENT_FACE_IN_MOTION; 
            }
            
            /* Save the last region of interest */
            LastRoi = myRoi; 
        }
    }
    else
    {
        movement_CurrentState = E_MOVEMENT_IDLE; 
    }

}


/*****************************************************
 *              Movement_SetFaceInMotion             *
 *                                                   *
 *                                                   *
 *****************************************************/
static void Movement_SetFaceInMotion(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint)
{
    T_ROI_FROM_CENTER centerRelPos;
   
    centerRelPos = get_CenterRelativePos(myRoi, myPoint); 

    /* Send the servo position */    
    send_ServoPosition(centerRelPos); 

    /* Stay here until we have centered the servo to the face */
    if(centerRelPos == E_CENTER_IN_ROI)
    {
        armMovementTime       = ros::Time::now().toSec();  
        movement_CurrentState = E_MOVEMENT_ARM_IN_MOTION; 
    }

}

/*****************************************************
 *              Movement_SetArmInMotion              *
 *                                                   *
 *                                                   *
 *****************************************************/
static void Movement_SetArmInMotion(void)
{
    if((ros::Time::now().toSec() - armMovementTime) > ARM_MOVEMENT_TIME)
    {
        movement_CurrentState = E_MOVEMENT_IDLE; 
    }
}

/*****************************************************
 *              get_CenterRelativePos                *
 *              Determines relative position         *
 *              of ROI from center                   *
 *              of the image                         *
 *****************************************************/
static T_ROI_FROM_CENTER get_CenterRelativePos(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint)
{
    T_ROI_FROM_CENTER result; 

    /* Local Inits */
    result = E_NO_ROI; 
    
    if((myRoi.x_offset != 0) && (myRoi.y_offset != 0) &&
       (myRoi.width != 0)    && (myRoi.height != 0))
    {
        /* Check if image center is bounded in the region of interest */
        if((myPoint.x > myRoi.x_offset) &&
           (myPoint.x < myRoi.x_offset + myRoi.width) &&
           (myPoint.y > myRoi.y_offset) &&
           (myPoint.y < myRoi.y_offset + myRoi.height))
        {
            result = E_CENTER_IN_ROI; 
        }
        else if(myPoint.x > (myRoi.x_offset + myRoi.width))
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



