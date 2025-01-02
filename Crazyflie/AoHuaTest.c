#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "TAKEOFF"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

static const float height_sp = 0.5f;
static const float height_down = 0.2f;

void appMainTask(void *param)
{
    // 执行 appMain 逻辑
    appMain();
    // 挂起任务，等待重新启动
    vTaskSuspend(NULL);

  }

void appMain()
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));
  
//  logVarId_t logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
//  uint16_t testz = logGetUint(logIdStateEstimateZ);
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");


  DEBUG_PRINT("Waiting for activation ...\n");

  uint8_t positioningInit = paramGetUint(idPositioningDeck);

  if (positioningInit) {
    float height = height_sp;
    float height_land = height_down;
    float velFront = 0;
    float velSide = 0;
      if (1) {
        DEBUG_PRINT("Waiting for the client ...\n");
      	vTaskDelay(M2T(5000));
      	DEBUG_PRINT("Taking off ...\n");
 	int k = 700; // Initialize counter
 	DEBUG_PRINT("k is %d now\n", k);
 
        while (k < 1000) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }
        
 	DEBUG_PRINT("k is %d now\n", k);
 	DEBUG_PRINT("Falling ...\n");

        while (k < 1200) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height_land, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }	

        while (k < 1400) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }	

        while (k < 1600) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height_land, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }	

        while (k < 1800) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }	

        while (k < 2000) {
        	vTaskDelay(M2T(10));
        	setHoverSetpoint(&setpoint, velFront, velSide, height_land, 0);
        	commanderSetSetpoint(&setpoint, 3);
        	k++;
        }	
        
 
	
        
        DEBUG_PRINT("k is %d now\n", k);
        DEBUG_PRINT("Landing ...\n");
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
