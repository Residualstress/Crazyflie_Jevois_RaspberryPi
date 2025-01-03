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
} Velocity;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  
  setpoint->velocity_body = true;
}

static logVarId_t idX;   
static float PositionX = 0.0f;

static logVarId_t idY;  
static float PositionY = 0.0f;

static void cpxPacketCallback(const CPXPacket_t* cpxRx);

static const float height_takeoff = 0.5f;
static const float height_land = 0.15f;
static const float distance_x = 3.5f;
static const double velMax = 0.15f;
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
    
    double postiion_x = logGetFloat(idX);
    double postiion_y = logGetFloat(idY);

    double error_x = postiion_x - target.x;
    if (fabs(error_x) <= tolerance) {
    	//DEBUG_PRINT("X position is satisfied, distance is: %.2f ...\n", error_x);
        return true;  // Current coordinates are close enough to the target
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
      return false; // Current coordinates are not at the target
     }

}


static void Move_to_target(){
	static setpoint_t setpoint;
	setHoverSetpoint(&setpoint, velocity.x, velocity.y, height_takeoff);
        commanderSetSetpoint(&setpoint, 3);
}

void appMain()
{
  static setpoint_t setpoint;
  Coordinate target_Coordinate = {distance_x, 0}; 
  vTaskDelay(M2T(10000));
  
  
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");


  DEBUG_PRINT("Waiting for activation ...\n");

  uint8_t positioningInit = paramGetUint(idPositioningDeck);


  cpxRegisterAppMessageHandler(cpxPacketCallback);	
  if (positioningInit) {
      if (1) {
        DEBUG_PRINT("Waiting for the client ...\n");
      	vTaskDelay(M2T(3000));
      	
      	DEBUG_PRINT("Taking off ...\n");
        vTaskDelay(M2T(10));
        
        
        while (1) {

        	if (is_at_target(target_Coordinate)){
        	   break;
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

static void cpxPacketCallback(const CPXPacket_t* cpxRx)
{
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    int8_t  raw_x  = (int8_t)cpxRx->data[0];
    float   divergence = ((float)raw_x) / 100.0f;

    uint8_t raw_y  = cpxRx->data[1];
    float   obstacle = (float)raw_y;

    DEBUG_PRINT("Divergence: %.2f\n", (double)divergence);
    DEBUG_PRINT("Obstacle parameter: %.2f\n", (double)obstacle);

    if(obstacle == 1.0f)
    {
        DEBUG_PRINT("Drone is landing normally.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %f deg\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        DEBUG_PRINT("PositionY is now: %f deg\n", (double)PositionY);
    }
}
