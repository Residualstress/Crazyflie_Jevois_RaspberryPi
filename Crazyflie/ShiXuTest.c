#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "app.h"
#include "cpx.h"
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

static const float height_takeoff = 1.25f;
static const float height_land = 0.15f;
static const float distance_x = 3.6f;
static const double velMax = 0.15f;
static Velocity velocity = {0, 0};
static float obstacle = 0;

void appMainTask(void *param)
{
    // 执行 appMain 逻辑
    appMain();
    // 挂起任务，等待重新启动
    vTaskSuspend(NULL);

  }


double drone_speed(double distance, double v_min, double v_max, double midpoint, double steepness) {
    double normalized_distance = 1.0 / (1.0 + exp(-steepness * (fabs(distance) - midpoint)));
    double speed = v_min + (v_max - v_min) * normalized_distance;
    
    // 关于原点对称
    if (distance < 0) {
        speed = -speed;
    }

    return speed;
}

bool is_at_target(Coordinate target) {
    // Compare each coordinate component with a tolerance (e.g., 0.1)
    double tolerance = 0.05;
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    
    double postiion_x = logGetFloat(idX);
    double postiion_y = logGetFloat(idY);
    double distance2_x =  target.x - postiion_x;
    double distance2_y =  target.y - postiion_y; 	
	
    if (fabs(postiion_x - target.x) <= tolerance &&
        fabs(postiion_y - target.y) <= tolerance) {
        return true;  // Current coordinates are close enough to the target
    } else{ 
        velocity.x = drone_speed(distance2_x, 0.05, 0.15, 0.1, 2);	
        velocity.y = drone_speed(distance2_y, 0.02, 0.15, 0.1, 2);
        return false;
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
  vTaskDelay(M2T(5000));
  
  
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");


  DEBUG_PRINT("Waiting for activation ...\n");

  uint8_t positioningInit = paramGetUint(idPositioningDeck);


  cpxRegisterAppMessageHandler(cpxPacketCallback);	
  if (positioningInit) {
      if (1) {
        DEBUG_PRINT("Waiting for the client in 5s...\n");
      	vTaskDelay(M2T(5000));
      	
      	DEBUG_PRINT("Taking off ...\n");
        vTaskDelay(M2T(10));
        int j = 0;
        while (j < 500){
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, 0, 0, height_takeoff);
        	commanderSetSetpoint(&setpoint, 3);
        	j++;    
        }       
        
        while (1) {

        	if (is_at_target(target_Coordinate)){
        	   DEBUG_PRINT("End point arrived.\n");
        	   break;
        	}
        	if (obstacle == 1.0f){
        	   DEBUG_PRINT("Drone is landing.\n");
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
    obstacle = (float)raw_y;

    DEBUG_PRINT("Divergence: %.2f\n", (double)divergence);
    //DEBUG_PRINT("Obstacle parameter: %.2f\n", (double)obstacle);

    if(obstacle == 1.0f)
    {
        DEBUG_PRINT("Obstacle detected.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %f deg\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        DEBUG_PRINT("PositionY is now: %f deg\n", (double)PositionY);
    }
}
