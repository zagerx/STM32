#include "printf_log.h"
#ifdef ENABLE_DEBUG
    #ifndef ENABLE_DEBUG_RTT
        #include "usart.h"
        int _write(int file, char *data, int len)
                {
                HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);
                return (status == HAL_OK ? len : 0);
        }
    #else
        void User_Printf_Init(void)
        {
            SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);            
        }
    #endif
#endif






