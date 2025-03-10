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

float DivergenceActual = 0;

LOG_GROUP_START(OpticalFLow)

LOG_ADD_CORE(LOG_FLOAT, divergence, &DivergenceActual)

LOG_GROUP_STOP(OpticalFLow)

static const float output_limit = 30.0f;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
    float output_limit;
    bool initialized;
} PIDController;

static void setPIDSetpoint(setpoint_t *setpoint, float pitch, float roll, float z)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  
  
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  
  setpoint->mode.pitch = modeAbs;
  setpoint->attitude.pitch = pitch;
  
  setpoint->mode.roll = modeAbs;
  setpoint->attitude.roll = roll;

  setpoint->velocity_body = true;
}

void PIDController_init(PIDController* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->output_limit = output_limit;
    pid->initialized = true;
    //DEBUG_PRINT("pid parameter is initialized as (%.2f, %.2f)\n", (double)pid->previous_error, (double)pid->integral);
}

float PIDController_update(PIDController* pid, float error) {
    // 计算积分项和微分项
    pid->integral += error;
    if (pid->initialized){
    pid-> previous_error = error;
    pid->initialized = false;
    } 
    float derivative = error - pid->previous_error;

    // 计算控制量
    float control_output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    //DEBUG_PRINT("P is %.2f, K is %.2f, D is %.2f, Output is %.2f. \n", (double)(pid->Kp * error), (double)(pid->Ki * pid->integral), (double)(pid->Kd * derivative), (double)control_output);
    
    if (control_output > pid->output_limit) {
        control_output = pid->output_limit;
    } else if (control_output < -pid->output_limit) {
        control_output = -pid->output_limit;
    }
    
    // 更新前一个误差
    pid->previous_error = error;
    
    return control_output;
}

static const float Kp = 0.6;      // //1.2, 0.00001, 500
static const float Ki = 0; //0.1 too big
static const float Kd = 600; //0.05
static PIDController pid_pitch;
static PIDController pid_roll;

typedef struct {
    double x;
    double y;
} Coordinate;

typedef struct {
    double x;
    double y;
} Velocity;

static void setPIDSetpoint(setpoint_t *setpoint, float pitch, float roll, float z) __attribute__((unused));
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float y, float z)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.y = modeAbs;
  setpoint->position.y = y;
    
  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = vx;
  
  setpoint->velocity_body = true;
}

static void setPositionSetpoint(setpoint_t *setpoint2, float x, float y, float z)
{
  setpoint2->mode.z = modeAbs;
  setpoint2->position.z = z;
  setpoint2->mode.y = modeAbs;
  setpoint2->position.y = y;
  setpoint2->mode.x = modeAbs;
  setpoint2->position.y = x;    

  
  setpoint2->velocity_body = true;
}


static logVarId_t idX;   
static float PositionX = 0.0f;

static logVarId_t idY;  
static float PositionY = 0.0f;

static void cpxPacketCallback(const CPXPacket_t* cpxRx);

static const float height_takeoff = 0.8f;
static const float height_land = 0.15f;
static const float distance_x = 3.4f;
static const double velMax = 0.15f;
static Velocity velocity = {0, 0};
static float obstacle = 0;
static float v = 0;
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

bool is_at_target(Coordinate target, double v) {
    // Compare each coordinate component with a tolerance (e.g., 0.1)
    double tolerance = 0.05;
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idVx = logGetVarId("stateEstimate", "vx");
    
    double postiion_x = logGetFloat(idX);
    double postiion_y = logGetFloat(idY);
    double velocity_x = logGetFloat(idVx);
    //double distance2_x =  target.x - postiion_x;
    double distance2_y =  target.y - postiion_y; 	
    
    float pitch = -PIDController_update(&pid_pitch, target.x - postiion_x);
    pitch = 0;
    
    	
    if (fabs(postiion_x - target.x) <= tolerance) {
        return true;  // Current coordinates are close enough to the target
    } else{
        velocity.x = v + velocity_x;
        if (velocity.x < 0.01) {
        velocity.x = 0.01;
        }
        if (velocity.x > 0.5){
        velocity.x =0.5;
        }         
        //velocity.x = 0.2;
        DEBUG_PRINT("V: %.2f\n", (double)velocity.x);	
        velocity.y = drone_speed(distance2_y, 0.02, 0.15, 0.1, 2);
        return false;
        } 
}


static void Move_to_target(){
	static setpoint_t setpoint;
	setHoverSetpoint(&setpoint, velocity.x, 0, height_takeoff);
        commanderSetSetpoint(&setpoint, 3);
}

void appMain()
{
  static setpoint_t setpoint;
  static setpoint_t setpoint2;
  
  PIDController_init(&pid_pitch, Kp, Ki, Kd);
  PIDController_init(&pid_roll, Kp, Ki, Kd);  
  
  DEBUG_PRINT("Waiting for activation ...\n");
  Coordinate target_Coordinate = {distance_x, 0}; 
  vTaskDelay(M2T(2000));
  
  
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  uint8_t positioningInit = paramGetUint(idPositioningDeck);


  cpxRegisterAppMessageHandler(cpxPacketCallback);	
  if (positioningInit) {
      if (1) {
        DEBUG_PRINT("Waiting for the client in 5s...\n");
      	vTaskDelay(M2T(5000));
      	
      	DEBUG_PRINT("Taking off ...\n");
        vTaskDelay(M2T(10));
        int j = 0;
        while (j < 1000){
        	vTaskDelay(M2T(10));
        	setPositionSetpoint(&setpoint2, 0, 0, height_takeoff);
        	commanderSetSetpoint(&setpoint2, 3);
        	j++;    
        }       
        
        while (1) {

        	if (is_at_target(target_Coordinate, v)){
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
    int16_t raw_x = (int16_t)(((uint16_t)cpxRx->data[0]) | ((uint16_t)cpxRx->data[1] << 8));
    float   divergence = ((float)raw_x) / 1000.0f;

    uint8_t raw_y  = cpxRx->data[2];
    obstacle = (float)raw_y;

    DEBUG_PRINT("Divergence: %.3f\n", (double)divergence);
    //DEBUG_PRINT("Obstacle parameter: %.2f\n", (double)obstacle);
    
    if (divergence > 200.2f)
    {
        divergence = 0.1f;
        DEBUG_PRINT("Adjusted Divergence (upper limit): %.2f\n", (double)divergence);
    }
    else if (divergence < -200.3f)
    {
        divergence = -0.1f;
        DEBUG_PRINT("Adjusted Divergence (lower limit): %.2f\n", (double)divergence);
    }
    
    DivergenceActual = divergence;
    
    float k = 1.0f;
    float D_star = -0.3f;
    v = k * (divergence - D_star);
    
    //DEBUG_PRINT("V: %.2f\n", (double)v);
    
    if(obstacle == 1.0f)
    {
        DEBUG_PRINT("Obstacle detected.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %.2f m\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        //DEBUG_PRINT("PositionY is now: %.2f deg\n", (double)PositionY);
    }
}
