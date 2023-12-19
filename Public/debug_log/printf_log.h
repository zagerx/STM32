#ifndef __PRINTF_LOG__H
#define __PRINTF_LOG__H

#define DEBUG_INFO_NORMAL    (1)
#define DEBUG_INFO_WARING    (2)
#define DEBUG_INFO_ERR       (3)
#define DEBUG_LEVEL          (DEBUG_INFO_NORMAL)

#ifdef	ENABLE_DEBUG
    #ifdef ENABLE_DEBUG_RTT
        #include "SEGGER_RTT.h"
        #if(DEBUG_INFO_NORMAL >= DEBUG_LEVEL)
            #define USER_DEBUG_NORMAL(format,...)	           SEGGER_RTT_printf(0,format,##__VA_ARGS__)
            #define USER_DEBUG_WARING(format,...)	           SEGGER_RTT_printf(0,format,##__VA_ARGS__)
            #define USER_DEBUG_ERR(format,...)	               SEGGER_RTT_printf(0,format,##__VA_ARGS__)
        #elif(DEBUG_INFO_WARING >= DEBUG_LEVEL)
            #define USER_DEBUG_NORMAL(format,...)	
            #define USER_DEBUG_WARING(format,...)	           SEGGER_RTT_printf(0,format,##__VA_ARGS__)
            #define USER_DEBUG_ERR(format,...)	               SEGGER_RTT_printf(0,format,##__VA_ARGS__)
        #elif(DEBUG_INFO_ERR >= DEBUG_LEVEL)
            #define USER_DEBUG_NORMAL(format,...)	
            #define USER_DEBUG_WARING(format,...)       
            #define USER_DEBUG_ERR(format,...)	               SEGGER_RTT_printf(0,format,##__VA_ARGS__)
        #else
            #define USER_DEBUG_NORMAL(format,...)
        #endif
        void User_Printf_Init(void);
    #else
        #include "stdio.h"
        #define USER_DEBUG_NORMAL(format,...)	printf(format,##__VA_ARGS__)
    #endif
#else
	#define	USER_DEBUG_NORMAL(format,...)
#endif
#endif
