#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "cpx.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

// Callback that is called when a CPX packet arrives
static void cpxPacketCallback(const CPXPacket_t* cpxRx);


void appMain() {
  DEBUG_PRINT("Hello! I am the stm_gap8_cpx app\n");

  // Register a callback for CPX packets.
  // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
  cpxRegisterAppMessageHandler(cpxPacketCallback);

  while(1) {
    vTaskDelay(M2T(3000));
    DEBUG_PRINT("waiting for data \n");

  }
}

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
  //DEBUG_PRINT("Oil palm location is (%u, %u)\n", cpxRx->data[0], cpxRx->data[1]);
  DEBUG_PRINT("Get number from Jevois is (%u)\n", cpxRx->data[0]);
}
