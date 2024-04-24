
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart2.h"
#include "system.h"


#include "debug.h"

static uint8_t uartRxp;

static void receiveByteTask(void *param)
{
  systemWaitStart();
  vTaskDelay(M2T(10));
  while (1)
  {    	  
    uart2GetDataWithTimeout(10, &uartRxp, M2T(200));
    if (uartRxp != 0){
    	DEBUG_PRINT("Get number from Jevois is (%u)\n", uartRxp);
    	uartRxp = 0;
    	}
    vTaskDelay(M2T(10));	
  }
}


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  uart2Init(115200);
  xTaskCreate(receiveByteTask, "JEVOIS UART READOUT", AI_DECK_TASK_STACKSIZE, NULL, AI_DECK_TASK_PRI, NULL);
  vTaskDelay(M2T(100));
  while(1) {
    vTaskDelay(M2T(5000));
    DEBUG_PRINT("Waiting for data!\n");
  }
}
