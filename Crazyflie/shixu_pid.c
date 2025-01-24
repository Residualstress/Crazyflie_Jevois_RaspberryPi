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

typedef struct {
    double y;
} Velocity;

static Velocity velocity = {0};

static const float output_limit = 30.0f;

typedef struct {
    double pitch;
    double roll;
} Attitude;

static Attitude attitude = {0, 0};

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
    DEBUG_PRINT("P is %.2f, K is %.2f, D is %.2f, Output is %.2f. \n", (double)(pid->Kp * error), (double)(pid->Ki * pid->integral), (double)(pid->Kd * derivative), (double)control_output);
    
    if (control_output > pid->output_limit) {
        control_output = pid->output_limit;
    } else if (control_output < -pid->output_limit) {
        control_output = -pid->output_limit;
    }
    
    // 更新前一个误差
    pid->previous_error = error;
    
    return control_output;
}

static const float Kp = 0.35;      // //1.2, 0.00001, 500
static const float Ki = 0; //0.1 too big
static const float Kd = 30; //0.05
static PIDController pid_pitch;
static PIDController pid_roll;
typedef struct {
    double x;
    double y;
} Coordinate;

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

static const float height_takeoff = 0.8f;
static const float height_land = 0.15f;
static const double distance_x = 3.6f;
static const double velMax = 0.15f;

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

bool is_at_target(double v) {
    // Compare each coordinate component with a tolerance (e.g., 0.1)
    attitude.pitch = -PIDController_update(&pid_pitch, v);
    
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    double postiion_y = logGetFloat(idY);
    double distance2_y =  -0.1 - postiion_y; 
    attitude.roll = -PIDController_update(&pid_roll, distance2_y);     	
    return false;
}


static void Move_to_target(){
	static setpoint_t setpoint;
	setPIDSetpoint(&setpoint, attitude.pitch, velocity.y, height_takeoff);
        commanderSetSetpoint(&setpoint, 3);
}

void appMain()
{
  static setpoint_t setpoint;
  logVarId_t idX = logGetVarId("stateEstimate", "x");
   
  PIDController_init(&pid_pitch, Kp, Ki, Kd);
PIDController_init(&pid_roll, 1.25, 0, 450);
  DEBUG_PRINT("Waiting for activation ...\n");
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
        	setHoverSetpoint(&setpoint, 0, 0, height_takeoff);
        	commanderSetSetpoint(&setpoint, 3);
        	j++;    
        }       
        
        while (1) {
		double position_x = logGetFloat(idX);	
        	if (is_at_target(v)){
        	   DEBUG_PRINT("End point arrived.\n");
        	}
        	
        	//if (obstacle == 1.0f){
        	   //DEBUG_PRINT("Drone is landing.\n");
        	   //break;
        	//}
        	if (position_x >= distance_x){
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
    float D_star = -0.05f;
    v = k * (divergence - D_star);
    
    //DEBUG_PRINT("V: %.2f\n", (double)v);
    
    if(obstacle == 1.0f)
    {
        DEBUG_PRINT("Obstacle detected.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %.2f deg\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        DEBUG_PRINT("PositionY is now: %.2f deg\n", (double)PositionY);
    }
}






SYS: ----------------------------
SYS: Crazyflie Bolt 1.1 is up and running!
SYS: Build 38:f61da11d54b6 (2023.11 +38) MODIFIED
SYS: I am 0x3530343431315104001F004B and I have 1024KB of flash!
CFGBLK: v1, verification [OK]
DECK_INFO: CONFIG_DECK_FORCE=cpxOverUART2 found
DECK_INFO: compile-time forced driver cpxOverUART2 added
DECK_CORE: 3 deck(s) found
DECK_CORE: Calling INIT on driver bcLoco for deck 0
DECK_CORE: Calling INIT on driver bcFlow2 for deck 1
ZR2: Z-down sensor [OK]
PMW: Motion chip id: 0x49:0xB6
DECK_CORE: Calling INIT on driver cpxOverUART2 for deck 2
CPX-EXT-ROUTER: Waiting for CPX External router initialization
CPX-EXT-ROUTER: CPX External router initialized, CPX_VERSION: 0
IMU: BMI088: Using SPI interface.
IMU: BMI088 Gyro connection [OK].
IMU: BMI088 Accel connection [OK]
IMU: BMP388 I2C connection [OK]
ESTIMATOR: Using Kalman (2) estimator
CONTROLLER: Using PID (1) controller
MTR-DRV: Using brushless motor driver
SYS: About to run tests in system.c.
SYS: NRF51 version: 2024.10 (CB10)
EEPROM: I2C connection [OK].
STORAGE: Storage check [OK].
IMU: BMI088 gyro self-test [OK]
DECK_CORE: Deck 0 test [OK].
DECK_CORE: Deck 1 test [OK].
DECK_CORE: Deck 2 test [OK].
SYS: Self test passed!
STAB: Wait for sensor calibration...
SYS: Free heap: 5984 bytes
AppStop is now: 1 
STAB: Starting stabilizer loop
ESTKALMAN: State out of bounds, resetting
SUP: Can not fly
SUP: Ready to fly
DWM: Automatic mode: detected TWR
Waiting for activation ...
Waiting for the client in 5s...
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.003
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.0
Divergence: 0.001
Divergence: 0.001
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.001
Divergence: 0.001
Divergence: 0.002
Divergence: 0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.001
Divergence: 0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Taking off ...
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: -0.065
Divergence: -0.097
Divergence: -0.146
Divergence: -0.142
Obstacle detected.
PositionX is now: 0.09 deg
PositionY is now: 0.09 deg
Divergence: -0.136
Obstacle detected.
PositionX is now: 0.08 deg
PositionY is now: 0.09 deg
Divergence: -0.129
Obstacle detected.
PositionX is now: 0.07 deg
PositionY is now: 0.08 deg
Divergence: -0.123
Divergence: -0.119
Divergence: -0.114
Divergence: -0.113
Divergence: -0.108
Divergence: -0.102
Obstacle detected.
PositionX is now: 0.04 deg
PositionY is now: 0.04 deg
Divergence: -0.096
Divergence: -0.091
Divergence: -0.086
Divergence: -0.082
Divergence: -0.078
Divergence: -0.073
Divergence: -0.069
Divergence: -0.065
Divergence: -0.061
Divergence: -0.058
Divergence: -0.054
Divergence: -0.051
Divergence: -0.048
Divergence: -0.046
Divergence: -0.043
Divergence: -0.040
Divergence: -0.038
Divergence: -0.033
Divergence: -0.033
Divergence: -0.027
Divergence: -0.026
Divergence: -0.023
Divergence: -0.021
Divergence: -0.019
Divergence: -0.018
Divergence: -0.016
Divergence: -0.016
Divergence: -0.013
Divergence: -0.012
Divergence: -0.009
Divergence: -0.008
Divergence: -0.007
Divergence: -0.006
Divergence: -0.005
Divergence: -0.003
Divergence: -0.003
Divergence: -0.002
Divergence: -0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.0
Divergence: 0.0
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: 0.0
Divergence: 0.001
Divergence: 0.002
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.002
Divergence: 0.003
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.002
Divergence: 0.003
Divergence: 0.002
Divergence: 0.002
Divergence: 0.0
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.002
Divergence: -0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.0
Divergence: 0.001
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.003
Divergence: 0.002
Divergence: 0.003
Divergence: 0.002
Divergence: 0.003
Divergence: 0.002
Divergence: 0.001
Divergence: 0.003
Divergence: 0.002
Divergence: 0.0
Divergence: 0.001
Divergence: 0.0
Divergence: 0.001
Divergence: 0.001
Divergence: 0.0
Divergence: 0.0
Divergence: 0.001
Divergence: 0.0
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: 0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.001
Divergence: -0.002
Divergence: -0.001
Divergence: -0.002
Divergence: -0.002
Divergence: -0.002
Divergence: -0.003
Divergence: -0.002
Divergence: -0.002
Divergence: -0.003
Divergence: -0.002
Divergence: -0.002
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.001
Divergence: -0.001
Divergence: -0.002
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.003
Divergence: -0.004
Divergence: -0.003
Divergence: -0.004
Divergence: -0.005
Divergence: -0.004
Divergence: -0.004
Divergence: -0.003
Divergence: -0.003
Divergence: -0.005
Divergence: -0.004
Divergence: -0.004
Divergence: -0.005
Divergence: -0.005
Divergence: -0.005
Divergence: -0.004
Divergence: -0.004
Divergence: -0.007
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is 0.0, Output is -0.02. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is -0.73, Output is -0.76. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is 0.01, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is -0.39, Output is -0.42. 
Divergence: -0.007
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is -0.10, Output is -0.13. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.50, Output is -0.53. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.0, Output is -0.02. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -1.38, Output is -1.42. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.36, Output is 0.33. 
Divergence: -0.009
P is 0.01, K is 0.0, D is -0.06, Output is -0.04. 
P is -0.03, K is 0.0, D is -0.93, Output is -0.96. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.25, Output is 0.22. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.64, Output is -0.68. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 1.03, Output is 1.0. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is 1.48, Output is 1.45. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.02, Output is 0.04. 
P is -0.03, K is 0.0, D is -0.32, Output is -0.36. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is 0.37, Output is 0.35. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.28, Output is -0.31. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.02, K is 0.0, D is 0.28, Output is 0.25. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.77, Output is -0.81. 
Divergence: -0.009
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.03, K is 0.0, D is 0.47, Output is 0.44. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.50, Output is -0.53. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.14, Output is 0.11. 
P is 0.01, K is 0.0, D is 0.0,77, Output is -0.80. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -1.33, Output is -1.37. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.32, Output is -0.36. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.36, Output is 0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.55, Output is -0.59. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.35, Output is 0.32. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.02, Output is 0.04. 
P is -0.03, K is 0.0, D is -0.08, Output is -0.12. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.41, Output is -0.45. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 1.69, Output is 1.66. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -1.69, Output is -1.73. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.70, Output is -0.74. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.15, Output is 0.11. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.42, Output is -0.46. 
P is 0.01, K is 0.0, D is 0.0,P is -0.04, K is 0.0, D is 0.41, Output is 0.37. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.26, Output is -0.30. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 1.49, Output is 1.46. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.28, Output is 0.24. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.06, Output is -0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.87, Output is 0.84. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.07, Output is 0.03. 
Divergence: -0.009
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.03, K is 0.0, D is -1.87, Output is -1.91. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.21, Output is -0.25. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.64, Output is 0.60. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.44, Output is -0.48. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.02, Output is 0.04. 
P is -0.04, K is 0.0, D is -0.28, Output is -0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.41, Output is -0.45. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.13, Output is 0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.16, Output is -0.21. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.43, Output is 0.39. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
85, Output is -0.89. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.13, Output is 0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.04, Output is 0.0. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.28, Output is 0.23. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.54, Output is -0.58. 
Divergence: -0.008
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.60, Output is 0.56. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.48, Output is -0.52. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.01, Output is -0.05. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.44, Output is -0.48. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.72, Output is 0.67. 
Divergence: -0.009
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.04, K is 0.0, D is -0.29, Output is -0.33. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.40, Output is 0.35. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.36, Output is -0.40. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
5, Output is 0.20. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
44, Output is -0.48. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.93, Output is 0.89. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 1.0P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.56, Output is -0.60. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.45, Output is 0.42. 
P is 0.01, K is 0.0, D is -0.03, Output is -0.01. 
P is -0.04, K is 0.0, D is -1.45, Output is -1.49. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.2P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.0, Output is -0.03. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.34, Output is 0.30. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
5, Output is 0.01. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.03, Output is 0.04. 
P is -0.03, K is 0.0, D is 0.28, Output is 0.24. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.24, Output is -0.28. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
5, Output is 0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 1.40, Output is 1.36. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.46, Output is -0.50. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.22, Output is 0.19. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.36, Output is -0.40. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.28, Output is 0.25. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.34, Output is -0.38. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.05, Output is -0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.46, Output is -0.50. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.65, Output is 0.61. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.09, Output is -0.12. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.29, Output is 0.25. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.59, Output is -0.63. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.35, Output is 0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.04, Output is -0.08. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.72, Output is 0.69. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.27, Output is -0.30. 
Divergence: -0.010
P is 0.01, K is 0.0, D is -0.03, Output is -0.01. 
P is -0.03, K is 0.0, D is 0.02, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.16, Output is -0.20. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.22, Output is 0.18. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.46, Output is -0.49. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.65, Output is 0.61. 
Divergence: -0.011
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.03, K is 0.0, D is -0.13, Output is -0.17. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.23, Output is 0.19. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.58, Output is -0.61. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.40, Output is 0.36. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
4, Output is 0.31. 
Divergence: -0.010
P is 0.01, K is 0.0, D is 0.02, Output is 0.04. 
P is -0.03, K is 0.0, D is 0.33, Output is 0.29. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -1.0, Output is -1.04. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.44, Output is 0.41. 
P is 0.01, K is 0.0, D is 0.0,P is -0.03, K is 0.0, D is -0.36, Output is -0.40. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.32, Output is 0.28. 
Divergence: -0.010
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.43, Output is -0.47. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.84, Output is 0.80. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.41, Output is -0.44. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.36, Output is -0.39. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.31, Output is -0.35. 
Divergence: -0.009
P is 0.01, K is 0.0, D is 0.03, Output is 0.04. 
P is -0.03, K is 0.0, D is 0.47, Output is 0.43. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.89, Output is -0.92. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is 0.41, Output is 0.37. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.03, K is 0.0, D is -0.60, Output is -0.64. 
Divergence: -0.010
P is 0.01, K is 0.0, D is -0.03, Output is -0.01. 
P is -0.03, K is 0.0, D is 0.12, Output is 0.08. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.56, Output is -0.60. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -1.39, Output is -1.44. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.46, Output is -0.51. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.17, Output is -0.21. 
Divergence: -0.010
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.61, Output is -0.66. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.17, Output is 0.12. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.45, Output is -0.50. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.27, Output is 0.23. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.58, Output is 0.53. 
Divergence: -0.011
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.04, K is 0.0, D is -0.14, Output is -0.19. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.28, Output is -0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.23, Output is -0.28. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.15, Output is -0.20. 
Divergence: -0.011
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.31, Output is 0.26. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 1.14, Output is 1.10. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.29, Output is -0.33. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.57, Output is 0.53. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.19, Output is -0.23. 
Divergence: -0.011
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -1.61, Output is -1.66. 
 Output is 0.01. 
P is -0.04, K is 0.0, D is -0.27, Output is -0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.14, Output is 0.09. 
 Output is 0.01. 
P is -0.04, K is 0.0, D is -0.10, Output is -0.15. 
Divergence: -0.012
P is 0.01, K is 0.0, D is -0.02, Output is -0.01. 
P is -0.04, K is 0.0, D is 0.61, Output is 0.56. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.20, Output is -0.24. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.41, Output is 0.36. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.15, Output is -0.19. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.37, Output is 0.32. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0,P is -0.04, K is 0.0, D is -0.19, Output is -0.23. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.56, Output is 0.51. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.07, Output is -0.11. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.10, Output is -0.14. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.19, Output is -0.24. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.17, Output is 0.12. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.50, Output is -0.55. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
2, Output is 0.38. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.05, Output is -0.10. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.47, Output is 0.43. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.85, Output is 0.81. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.33, Output is 0.28. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.16, Output is -0.20. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is 0.12, Output is 0.08. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -0.P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.04, K is 0.0, D is -1.89, Output is -1.93. 
 Output is 0.01. 
P is -0.04, K is 0.0, D is -0.25, Output is -0.29. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.77, Output is -0.82. 
P is 0.01, K is 0.0, D is 0.0,P is -0.05, K is 0.0, D is 0.15, Output is 0.10. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.28, Output is -0.33. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.04, Output is -0.10. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.94, Output is -1.0. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.35, Output is 0.30. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.53, Output is -0.58. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.03, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.43, Output is -0.48. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
3, Output is 0.98. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -1.49, Output is -1.55. 
Divergence: -0.012
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.08, Output is 0.02. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.41, Output is -0.47. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.19, Output is -0.25. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.13, Output is 0.07. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.16, Output is 0.10. 
Divergence: -0.013
P is 0.01, K is 0.0, D is -0.03, Output is -0.01. 
P is -0.05, K is 0.0, D is -0.26, Output is -0.32. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.04, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.72, Output is -0.78. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.4P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.02, Output is -0.08. 
Divergence: -0.013
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.0, Output is -0.06. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.07, Output is -0.13. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.52, Output is 0.47. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.03, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.34, Output is -0.40. 
Divergence: -0.015
P is 0.01, K is 0.0, D is -0.06, Output is -0.04. 
P is -0.05, K is 0.0, D is 0.43, Output is 0.37. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.56, Output is 0.50. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.22, Output is -0.28. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.09, Output is 0.04. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.46, Output is -0.52. 
Divergence: -0.015
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.17, Output is -0.23. 
 Output is 0.01. 
P is -0.05, K is 0.0, D is -0.71, Output is -0.77. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.27, Output is 0.21. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.57, Output is -0.63. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.33, Output is -0.40. 
Divergence: -0.017
P is 0.01, K is 0.0, D is -0.06, Output is -0.04. 
P is -0.06, K is 0.0, D is -0.12, Output is -0.18. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 1.2P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.01, Output is -0.04. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.01, Output is -0.04. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.33, Output is -0.39. 
Divergence: -0.017
P is 0.01, K is 0.0, D is 0.0,P is -0.05, K is 0.0, D is 0.15, Output is 0.10. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.31, Output is -0.37. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.08, Output is 0.02. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.30, Output is -0.36. 
DiveP is 0.rgence: -0.017
01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.40, Output is 0.34. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.32, Output is 0.26. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.08, Output is 0.02. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.32, Output is -0.38. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.18, Output is 0.12. 
Divergence: -0.017
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.46, Output is -0.52. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.40, Output is 0.34. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.18, Output is 0.12. 
P is 0.01, K is 0.0, D is 0.0,P is -0.05, K is 0.0, D is -0.09, Output is -0.15. 
Divergence: -0.017
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.0, Output is -0.06. 
 Output is 0.01. 
P is -0.05, K is 0.0, D is -0.24, Output is -0.30. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.1P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.43, Output is -0.49. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is 0.09, Output is 0.03. 
Divergence: -0.019
P is 0.01, K is 0.0, D is -0.05, Output is -0.04. 
P is -0.06, K is 0.0, D is 0.0, Output is -0.05. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.15, Output is 0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.91, Output is 0.86. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 1.0P is 0.01, K is 0.0, D is 0.Di0, Output is 0.01. 
P is -0.05, K is 0.0, D is -1.77, Output is -1.83. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.75, Output is 0.69. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.81, Output is -0.87. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.56, Output is 0.50. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.34, Output is -0.40. 
Divergence: -0.019
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.22, Output is -0.28. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.06, Output is 0.0. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.30, Output is 0.24. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.31, Output is -0.37. 
 Output is 0.01. 
P is -0.05, K is 0.0, D is -0.06, Output is -0.12. 
Divergence: -0.019
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.36, Output is 0.30. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.50, Output is -0.56. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
6, Output is 0.30. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.23, Output is -0.29. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.44, Output is 0.38. 
Divergence: -0.021
P is 0.01, K is 0.0, D is -0.06, Output is -0.04. 
P is -0.05, K is 0.0, D is -0.53, Output is -0.59. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.67, Output is 0.61. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.30, Output is 0.24. 
Divergence: -0.021
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.61, Output is -0.67. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.15, Output is -0.21. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.31, Output is -0.38. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is 0.37, Output is 0.31. 
Divergence: -0.021
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.18, Output is -0.24. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is 0.04, Output is -0.01. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.06, K is 0.0, D is -0.20, Output is -0.26. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.75, Output is 0.69. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.07, Output is -0.13. 
Divergence: -0.021
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.07, Output is -0.12. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.92, Output is 0.87. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.14, Output is -0.20. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.14, Output is 0.08. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.29, Output is 0.23. 
Divergence: -0.021
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.61, Output is -0.67. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.24, Output is 0.19. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is -0.17, Output is -0.23. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.14, Output is 0.09. 
P is 0.01, K is 0.0, D is 0.0, Output is 0.01. 
P is -0.05, K is 0.0, D is 0.39, Output is 0.34. 
Divergence: -0.023
P is 0.0, K is 0.0, D is -0.06, Output is -0.05. 
P is -0.05, K is 0.0, D is 0.25, Output is 0.19. 
P is 0.0, K is 0.0, D is 0.0, Output is 0.0. 
P is -0.05, K is 0.0, D is -0.25, Output is -0.31. 
P is 0.0, K is 0.0, D is 0.0, Output is 0.0. 
P is -0.05, K is 0.0, D is -0.04, Output is -0.09. 
P is 0.0, K is 0.0, D is 0.0, Output is 0.0. 
P is -0.05, K is 0.0, D is -0.43, Output is -0.48. 
P is 0.0, K is 0.0, D is 0.0, Output is 0.0. 
P is -0.05, K is 0.0, D is -0.05, Output is -0.10. 
Divergence: -0.023<F>
Y pian li tai duo. guo chong ,wu fa xiu zheng
