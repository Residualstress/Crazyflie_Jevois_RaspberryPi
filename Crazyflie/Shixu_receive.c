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
    // 解析前两字节获得 divergence
    int16_t raw_x = (int16_t)(((uint16_t)cpxRx->data[0]) | ((uint16_t)cpxRx->data[1] << 8));
    float divergence = ((float)raw_x) / 1000.0f;

    // 读取第三个字节作为 obstacle 标志
    uint8_t raw_y = cpxRx->data[2];
    float obstacle = (float)raw_y;

    // 解析第4-5字节获得 chi-square 值，保留两位小数
    int16_t raw_chi = (int16_t)(((uint16_t)cpxRx->data[3]) | ((uint16_t)cpxRx->data[4] << 8));
    float chi_square = ((float)raw_chi) / 100.0f;

    // 解析第6-7字节获得 delta 值，保留两位小数
    int16_t raw_delta = (int16_t)(((uint16_t)cpxRx->data[5]) | ((uint16_t)cpxRx->data[6] << 8));
    float delta = ((float)raw_delta) / 100.0f;

    // 对 divergence 应用上下限
    if (divergence > 200.200f)
    {
        divergence = 0.200f;
        DEBUG_PRINT("Adjusted Divergence (upper limit): %.3f\n", (double)divergence);
    }
    else if (divergence < -200.300f)
    {
        divergence = -0.300f;
        DEBUG_PRINT("Adjusted Divergence (lower limit): %.3f\n", (double)divergence);
    }

    // 计算速度参数 v
    float k = 5.0f;
    float D_star = -0.1f;
    float v = k * (divergence - D_star);

    // 打印接收到的数据和计算的结果
    DEBUG_PRINT("Divergence: %.3f\n", (double)divergence);
    DEBUG_PRINT("Chi-Square: %.2f\n", (double)chi_square);
    DEBUG_PRINT("Delta: %.2f\n", (double)delta);
    DEBUG_PRINT("v: %.3f\n", (double)v);

    // 如果检测到障碍物，执行额外操作
    if (obstacle == 1.0f)
    {
        DEBUG_PRINT("Drone is landing normally.\n");
        PositionX = logGetFloat(idX);
        DEBUG_PRINT("PositionX is now: %.3f deg\n", (double)PositionX);
        PositionY = logGetFloat(idY);
        DEBUG_PRINT("PositionY is now: %.3f deg\n", (double)PositionY);
    }
}
