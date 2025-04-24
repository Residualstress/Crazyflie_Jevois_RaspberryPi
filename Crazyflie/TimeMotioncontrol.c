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
#include "pm.h" 
#include "log.h"
#include "param.h"
#define DEBUG_MODULE "APP"


typedef struct {
    float x;
    float y;
} FloatCoordinates;
typedef struct {
    double x;
    double y;
    double pixel_x;
    double pixel_y;
} Coordinate;

typedef struct {
    double x;
    double y;
} Velocity;

typedef struct {
  double x;
  double y;
  double z;
} Position;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    float P[2][2]; // 协方差矩阵
} KalmanFilter;

typedef struct {
  bool isInStraightPath; // 是否在直线路径上的标志
  double distanceToLine;  // 当前位置到直线路径的垂直距离
  double nearestPointX;   // 最近点的 X 坐标
  double nearestPointY;   // 最近点的 Y 坐标
} StraightPathResult;

static Velocity velocity = {0, 0};
static Coordinate Targetpoint = {0, 0, 0, 0};
static Coordinate nearestTree = {0, 0, 0, 0};
static const float height_takeoff = 1.25f;
static const float height_land_1 = 0.5f;
static const float height_land_2 = 0.15f;
static const float distance_x = 4.2f;
static const float distance_y = -0.1f;
static const double velMax = 0.10f;
static const double velMax_mainroad = 0.1f;



static const double SameTreeTh = 0.45; //Trees with a centre distance of less than 0.5 m are treated as the same tree.
static uint16_t numTrees = 0; // The number of saved trees
static uint16_t numtreeBlacklist = 0;
static float x =0 ;
static float y =0 ;


static KalmanFilter filter;
static uint16_t KalmantreeNum = 999;

static float dt = 0.001;
static uint16_t KalmanUpdateCount = 0;
static double real_pixel_x;
static double real_pixel_y;

bool printedReturnToPath = false;
bool printedMoveToEnd = false;
bool printedMoveToTreeTop = false;
bool checkReturnToPath = true;
bool isreplacepixel = false;
bool isUWB = false;
bool Pixeldetected = false;

void kalmanFilterInit(KalmanFilter* filter, float x, float y, float vx, float vy) {
    filter->x = x;
    filter->y = y;
    filter->vx = vx;
    filter->vy = vy;

    // 初始化协方差矩阵
    filter->P[0][0] = 0.1f; // 初始协方差矩阵
    filter->P[1][1] = 0.1f;
    filter->P[2][2] = 0.1f;
    filter->P[3][3] = 0.1f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    KalmanUpdateCount = 0;
}

void kalmanFilterUpdate(KalmanFilter* filter, float x, float y, float dt) {
    // 更新步骤
    float R = 0.1f; // 观测噪声协方差

    float Kx = filter->P[0][0] / (filter->P[0][0] + R);
    float Ky = filter->P[1][1] / (filter->P[1][1] + R);

    filter->x += Kx * (x - filter->x); // 更新位置
    filter->y += Ky * (y - filter->y);
    
    filter->vx += Kx * (x - filter->x) / (dt*KalmanUpdateCount);
    filter->vy += Ky * (y - filter->y) / (dt*KalmanUpdateCount);

    // 更新协方差矩阵
    filter->P[0][0] *= (1 - Kx);
    filter->P[1][1] *= (1 - Ky);
    filter->P[2][2] *= (1 - Kx);
    filter->P[3][3] *= (1 - Ky);
    KalmanUpdateCount = 0;
}

void kalmanFilterPredict(KalmanFilter* filter, float dt) {
    // 预测步骤
    filter->x += filter->vx * dt; // 更新位置
    filter->y += filter->vy * dt;

    // 更新协方差矩阵
    float Q = 0.02f; // 过程噪声协方差
    filter->P[0][0] += Q; 
    filter->P[1][1] += Q;
    filter->P[2][2] += Q;
    filter->P[3][3] += Q;
}



static Coordinate trees[10];
static Coordinate treeBlacklist[10];



void appMainTask(void *param)
{
    // 执行 appMain 逻辑
    appMain();
    // 挂起任务，等待重新启动
    vTaskSuspend(NULL);

  }

bool isDuplicateBlackCoordinate(double newX, double newY, Coordinate* trees, uint16_t numTrees, float threshold) {
    if (numTrees == 0) {
        return false; // 如果没有任何坐标，不进行重复检测
    }

    for (uint16_t i = 0; i < numTrees; i++) {
        float distance = sqrtf(powf(newX - trees[i].x, 2) + powf(newY - trees[i].y, 2));

        if (distance < threshold)  {
            return true; // 发现重复坐标
            }
        }
    

    return false; // 未发现重复坐标
}

bool isDuplicateCoordinate(double newX, double newY, double pixel_x, double pixel_y, Coordinate* trees, uint16_t numTrees, float threshold) {
    if (numTrees == 0) {
        return false; // 如果没有任何坐标，不进行重复检测
    }

    for (uint16_t i = 0; i < numTrees; i++) {
        float distance = sqrtf(powf(newX - trees[i].x, 2) + powf(newY - trees[i].y, 2));

        if (distance < threshold)  {
	    trees[i].pixel_x = pixel_x;
	    trees[i].pixel_y = pixel_y;
	    if (i == KalmantreeNum) {
	    real_pixel_x = pixel_x;
	    real_pixel_y = pixel_y;
	    kalmanFilterUpdate(&filter, pixel_x, pixel_y, dt);
	    Pixeldetected = true; 
	    DEBUG_PRINT("Kalman Tree %u is found, pixels update:pixel_x = %.2f, pixel_y = %.2f\n", i, trees[i].pixel_x, trees[i].pixel_y);   
	    }
            return true; // 发现重复坐标并更新
            }
        }
    
	
    return false; // 未发现重复坐标
}


static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float yaw, float z)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;
  
  setpoint->velocity_body = true;
}

double drone_speed2(double distance,double v_min, double v_max, double steepness) {
    double speed = steepness*fabs(distance) + 0.02;

    if (speed > v_max) {
    	speed = v_max;
    	}
    if (speed < v_min){
    	speed = v_min;
    }
    
    // 关于原点对称
    if (distance < 0) {
        speed = -speed;
    }

    return 0;
}

bool is_at_target(Coordinate target, double error_x, double error_y) {
    // Compare each coordinate component with a tolerance (e.g., 0.1)
    //double tolerance1 = 0.1;
    double tolerance2 = 0.7;
    double tolerance3 = 40;
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    
    double position_x = logGetFloat(idX);
    double position_y = logGetFloat(idY);
    
    double target_x = target.x -0.2;
    double target_y = target.y;
    
    double distance1_x =  target_x-position_x;
    double distance1_y =  target_y-position_y;
     
    if (fabs(distance1_x) >= tolerance2 ||
        fabs(distance1_y) >= tolerance2){
      	isUWB =true;
      	Pixeldetected = false;
      	//DEBUG_PRINT("Distance is %.2f, %.2f, uwb control start ...\n", distance1_x, distance1_y);	
     } 
    if (isUWB){
//    if (fabs(distance1_x) >= tolerance1 ||
//        fabs(distance1_y) >= tolerance1){
	
	velocity.x = drone_speed2(distance1_x, 0.05, 0.12, 0.1);    	
	velocity.y = drone_speed2(distance1_y, 0.05, 0.12, 0.1);    	
        DEBUG_PRINT("Control by uwb\n");
        isreplacepixel = true;	
        if (Pixeldetected){
      	isUWB =false;
    	}
        return false;

     }else{     

     //DEBUG_PRINT("Distance is %.2f, %.2f, vision navigation ...\n", distance1_x, distance1_y); 
     if ((fabs(error_x) > tolerance3) || (fabs(error_y) > tolerance3)) {
    		//DEBUG_PRINT("X position isn't satisfied ...\n");
            velocity.x = drone_speed2(error_x, 0, 0.1, 0.001);    	    		
            velocity.y = drone_speed2(error_y, 0, 0.1, 0.001);
            //DEBUG_PRINT("Velocity x,y is %.2f, %.2f...\n", velocity.x, velocity.y);
            return false;    		    	
     } else {
    	return true;
    	}	
     
     }

        	       
}

void removeCoordinate(Coordinate* trees, Coordinate coordToRemove) {
    for (uint16_t i = 0; i < numTrees; i++) {
	double distance = sqrtf(powf(coordToRemove.x - trees[i].x, 2) + powf(coordToRemove.y - trees[i].y, 2));    
        if (distance < SameTreeTh) {
            for (uint16_t j = i; j < numTrees - 1; j++) {
                trees[j] = trees[j + 1];
            }
            (numTrees)--;
            break;
        }
    }
}

Coordinate findNearestCoordinate(const Coordinate* trees, double currentX, double currentY) {
    double minDistance = sqrt(pow(currentX - trees[0].x, 2) + pow(currentY - trees[0].y, 2));
    Coordinate nearestCoordinate = {trees[0].x, trees[0].y};
    Coordinate nearestTree = trees[0];
    for (uint16_t i = 0; i < numTrees; i++) {
        double distance = sqrt(pow(currentX - trees[i].x, 2) + pow(currentY - trees[i].y, 2));
        if (distance <= minDistance) {
            minDistance = distance;
            nearestCoordinate.x = trees[i].x;
            nearestCoordinate.y = trees[i].y;
            nearestTree = trees[i];
            KalmantreeNum = i;
        }
    }
    return nearestTree;
}


static void Move_to_target(){
	static setpoint_t setpoint;
	setHoverSetpoint(&setpoint, 0, 0, 0, 0);
        commanderSetSetpoint(&setpoint, 3);
}

void addCoordinateToBlacklist(Coordinate tree) {
    treeBlacklist[numtreeBlacklist++] = tree;  
}


void Move_to_TreeTop() {
    TickType_t start_tick = xTaskGetTickCount();  // 获取起始 tick   
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    double position_x = logGetFloat(idX);
    double position_y = logGetFloat(idY);
    nearestTree = findNearestCoordinate(trees, position_x, position_y);
    Coordinate nearestCoord = {nearestTree.x, nearestTree.y};
    kalmanFilterInit(&filter, nearestTree.pixel_x, nearestTree.pixel_y, 0, 0);
    real_pixel_x = nearestTree.pixel_x;
    real_pixel_y = nearestTree.pixel_y;
    if (numTrees > 0) {
        DEBUG_PRINT("fly to tree top:(%.2f,%.2f)!\n", nearestCoord.x, nearestCoord.y);

            if (isreplacepixel){
            kalmanFilterInit(&filter, real_pixel_x, real_pixel_y, 0, 0);
            isreplacepixel = false;
            }	
            kalmanFilterPredict(&filter, dt);
            Targetpoint.pixel_x = filter.x;
            Targetpoint.pixel_y = filter.y;
            KalmanUpdateCount++;
            double error_x = 144-Targetpoint.pixel_x;
            double error_y = 176-Targetpoint.pixel_y;
            //DEBUG_PRINT("Treepixel output: pixel_x = %.2f, pixel_y = %.2f, error_x = %.2f, error_y = %.2f\n", Targetpoint.pixel_x, Targetpoint.pixel_y, error_x, error_y);    

	    if (is_at_target(nearestCoord, error_x, error_y)){
            checkReturnToPath = true;     
            DEBUG_PRINT("Arrived on tree top:(%.2f,%.2f)", nearestCoord.x, nearestCoord.y);
            removeCoordinate(trees, nearestCoord);
            addCoordinateToBlacklist(nearestCoord);
            if (numtreeBlacklist > 0){
            for (uint16_t i = 0; i < numtreeBlacklist; i++) {
DEBUG_PRINT("Tree %u: x = %.2f, y = %.2f\n", i, (double)treeBlacklist[i].x, (double)treeBlacklist[i].y);
        } 
            }			
            printedMoveToEnd = false;

            }
                                    
	    if (!printedMoveToTreeTop) {
	    DEBUG_PRINT("Move to tree top:(%.2f,%.2f)", nearestCoord.x, nearestCoord.y);
	    printedMoveToTreeTop = true;
	    }        	
            Move_to_target();
	    vTaskDelay(M2T(10));


    }
    TickType_t end_tick = xTaskGetTickCount();    // 获取结束 tick

    // 每个 tick 通常为 1ms，视 configTICK_RATE_HZ 而定
    uint32_t duration_ms = (end_tick - start_tick);  

    DEBUG_PRINT("Execution time: %lu ms\n", (unsigned long)duration_ms);
}


void appMain() {

    double postiion_x = 0.2;
    double postiion_y = 0.2;	

    // 检查是否还有空间存储新的坐标

    x = (144-100)*(0.0031*1.2)+postiion_x+0.2;  //1.5m-0.0045
    y = (176-100)*(0.0031*1.2)+postiion_y;
     if (fabs(x) < 5.5 && fabs(y) < 2.5) {
       if (!isDuplicateBlackCoordinate(x, y, treeBlacklist, numtreeBlacklist, SameTreeTh)){
        if (!isDuplicateCoordinate(x, y, 120, 120, trees, numTrees, SameTreeTh)) {
            // 假设数据包的格式是 [x, y]
            DEBUG_PRINT("Oil palm detected, drone position is  (%.2f, %.2f)\n", (double)postiion_x, (double)postiion_y);

            trees[numTrees].x = x;
            trees[numTrees].y = y;
	    trees[numTrees].pixel_x = 100;
	    trees[numTrees].pixel_y = 100;
	    DEBUG_PRINT("New added Oil palm location is (%.2f, %.2f), pixel_x = %.2f, pixel_y = %.2f\n", (double)x, (double)y, (double)trees[numTrees].pixel_x, (double)trees[numTrees].pixel_y);
            DEBUG_PRINT("numTrees is (%u)\n", numTrees+1);
            numTrees++; // 增加树的数量
            
            for (uint16_t i = 0; i < numTrees; i++) {
DEBUG_PRINT("Tree %u: x = %.2f, y = %.2f, pixel_x = %.2f, pixel_y = %.2f\n", i, (double)trees[i].x, (double)trees[i].y, trees[i].pixel_x, trees[i].pixel_y);
        }            
       }
      } 
     } else {
     DEBUG_PRINT("New Oil palm detection, but coordinates out of bounds: (%.2f, %.2f)\n", (double)x, (double)y);
     }
    TickType_t start_tick = xTaskGetTickCount();  // 获取起始 tick    
    Move_to_TreeTop(); 
    TickType_t end_tick = xTaskGetTickCount();    // 获取结束 tick

    // 每个 tick 通常为 1ms，视 configTICK_RATE_HZ 而定
    uint32_t duration_ms = (end_tick - start_tick);  

    DEBUG_PRINT("Execution time: %lu ms\n", (unsigned long)duration_ms);
    
    	
    
}
