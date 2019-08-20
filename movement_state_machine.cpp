
/*****************************************************
 *              Header Includes                      *
 *****************************************************/

#include "movement_state_machine.h" 

/*****************************************************
 *            Definitions                            *
 *****************************************************/
typedef enum
{
    E_MOVEMENT_IDLE,
    E_MOVEMENT_LOOKING_FOR_FACE,
    E_MOVEMENT_FACE_DEBOUNCE,
    E_MOVEMENT_IN_MOTION

}T_MOVEMENT_STATE;

#define THRESHOLD_PERCENT_MIN  0.8f
#define THRESHOLD_PERCENT_MAX  1.2f
#define FACE_DEBOUNCE_TIME     5

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/
static T_MOVEMENT_STATE movement_CurrentState; 
static sensor_msgs::RegionOfInterest LastRoi; 
static double faceTimeDetected; 

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
static void Movement_Idle(void); 
static void Movement_LookForFace(sensor_msgs::RegionOfInterest myRoi); 
static void Movement_DebounceFace(sensor_msgs::RegionOfInterest myRoi); 

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

}

/*****************************************************
 *              Movement_ManageStateMachine          *
 *                                                   *
 *                                                   *
 *****************************************************/
void Movement_ManageStateMachine(sensor_msgs::RegionOfInterest myRoi, geometry_msgs::Point myPoint)
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
        case E_MOVEMENT_IN_MOTION:  
            break; 
    
    } 
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
                movement_CurrentState = E_MOVEMENT_IN_MOTION; 
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



