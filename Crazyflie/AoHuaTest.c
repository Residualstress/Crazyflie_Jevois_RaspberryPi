#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "TAKEOFF"


typedef struct {
    double x;
    double y;
} Coordinate;

typedef struct {
    double x;
    double y;
    double z;
} Velocity;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float vz)
{
  //setpoint->mode.z = modeAbs;
  //setpoint->position.z = z;
  
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity.z = vz; 
  setpoint->velocity_body = true;
}

static const float height_takeoff = 0.4f;
static const float height_land = 0.15f;
static const float distance_x = 1.98f;
static const float distance_y = 0.14f;
static const double velMax = 0.15f;
static const double BOX_LIMIT_high = 0.5f;
static const double BOX_LIMIT_down = 0.3f;
static Velocity velocity = {0, 0};


void appMainTask(void *param)
{
    // 执行 appMain 逻辑
    appMain();
    // 挂起任务，等待重新启动
    vTaskSuspend(NULL);

  }


bool is_at_target(Coordinate target) {
    // Compare each coordinate component with a tolerance (e.g., 0.1)
    double tolerance = 0.05;
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idZ = logGetVarId("stateEstimate", "z");
    double postiion_x = logGetFloat(idX);
    double postiion_y = logGetFloat(idY);
    double postiion_z = logGetFloat(idZ);
    double error_x = postiion_x - target.x;
    double error_y = postiion_y - target.y;
    if (fabs(error_x) <= tolerance && fabs(error_y) <= tolerance) {
    	DEBUG_PRINT("position is satisfied, distance is: %.2f ...\n", error_x);
        //return true;  // Current coordinates are close enough to the target
    } else{
    
     if (fabs(postiion_x - target.x) > tolerance) {
    		//DEBUG_PRINT("X position isn't satisfied ...\n");
    		if ((target.x-postiion_x)>0){
    			velocity.x = velMax;
    		} else {
			velocity.x = -velMax;    	
    		}
     } else{
    	velocity.x = 0;
     }
        
     if (fabs(postiion_y - target.y) > tolerance){
      		//DEBUG_PRINT("Y position isn't satisfied ...\n");	
    		if ((target.y-postiion_y)>0){
    			velocity.y = velMax;
    		} else {
			velocity.y = -velMax;    	
    		}    	
    	
     } else {
    	velocity.y = 0;
    	}	
      //return false; // Current coordinates are not at the target
     }
     
     if (postiion_z > BOX_LIMIT_high){	
    	velocity.z = -velMax*2;
    	DEBUG_PRINT("down ...\n"); 	       	
     } else if (postiion_z < BOX_LIMIT_down) {
	velocity.z = velMax*2;
	DEBUG_PRINT("up ...\n");     	
    	} 	
     return false; // Current coordinates are not at the target
     
}


static void Move_to_target(){
	static setpoint_t setpoint;
	DEBUG_PRINT("Y velocity is: %.2f ...\n", velocity.z);
	setHoverSetpoint(&setpoint, velocity.x, velocity.y, velocity.z);
        commanderSetSetpoint(&setpoint, 3);
}

void appMain()
{
  static setpoint_t setpoint;
  Coordinate target_Coordinate = {distance_x, distance_y}; 
  
  vTaskDelay(M2T(5000));
  
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");


  DEBUG_PRINT("Waiting for activation ...\n");

  uint8_t positioningInit = paramGetUint(idPositioningDeck);

  if (positioningInit) {
      if (1) {
        DEBUG_PRINT("Waiting for the client ...\n");
      	vTaskDelay(M2T(3000));
      	
      	DEBUG_PRINT("Taking off ...\n");
        vTaskDelay(M2T(10));
        
        
        while (1) {

        	if (is_at_target(target_Coordinate)){
        	   DEBUG_PRINT("bouncing!\n");
        	}
        	Move_to_target();
        	//DEBUG_PRINT("X velocity is %.2f, Y velocity is %.2f ...\n\n", velocity.x, velocity.y);
		vTaskDelay(M2T(10));
	}
	
	
	DEBUG_PRINT("Mission complete\n");
        int k = 0;
        while (k < 300){
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, 0, 0, height_land);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;    
        }
        while (1) {
        	vTaskDelay(M2T(10));     
        	memset(&setpoint, 0, sizeof(setpoint_t));
        	commanderSetSetpoint(&setpoint, 3);
        }        
      }
  } else {
    DEBUG_PRINT("No flow deck installed ...\n");
  }
}
