#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "cpx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"

static logVarId_t idX;
static float PositionX = 0.0f;

static logVarId_t idY;
static float PositionY = 0.0f;

static void cpxPacketCallback(const CPXPacket_t* cpxRx);

void appMainTask(void *param)
{
    // 执行 appMain 逻辑
    appMain();
    // 挂起任务，等待重新启动
    vTaskSuspend(NULL);
}

void appMain(void)
{
    DEBUG_PRINT("Hello! I am the stm_gap8_cpx app\n");

    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");

    cpxRegisterAppMessageHandler(cpxPacketCallback);

    while(1)
    {
        vTaskDelay(M2T(3000));
        DEBUG_PRINT("waiting for data \n");
    }
}

static void cpxPacketCallback(const CPXPacket_t* cpxRx)
{
    int16_t raw_x = (int16_t)(((uint16_t)cpxRx->data[0]) | ((uint16_t)cpxRx->data[1] << 8));
    // 缩放以获得三位小数的 divergence
    float divergence = ((float)raw_x) / 1000.0f;

    // 读取第三个字节作为 obstacle 标志
    uint8_t raw_y = cpxRx->data[2];
    float obstacle = (float)raw_y;

    // 对 divergence 应用上下限
    if (divergence > 0.200f)
    {
        divergence = 0.200f;
        DEBUG_PRINT("Adjusted Divergence (upper limit): %.3f\n", (double)divergence);
    }
    else if (divergence < -0.300f)
    {
        divergence = -0.300f;
        DEBUG_PRINT("Adjusted Divergence (lower limit): %.3f\n", (double)divergence);
    }

    // 计算速度参数 v
    float k = 5.0f;
    float D_star = -0.01f;
    float v = k * (divergence - D_star);

    // 打印接收到的和计算出的值，保留三位小数
    DEBUG_PRINT("Divergence: %.3f\n", (double)divergence);
    //DEBUG_PRINT("Obstacle parameter: %.3f\n", (double)obstacle);
    DEBUG_PRINT("v: %.3f\n", (double)v);

    // 如果检测到障碍物，执行额外操作
    if(obstacle == 1.0f)
    {
        DEBUG_PRINT("Drone is landing normally.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %.3f deg\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        DEBUG_PRINT("PositionY is now: %.3f deg\n", (double)PositionY);
    }
}
