
#include "_boot_cfg.h"

typedef  void (*pFunction)(void);
void Iap_jumpToApp(void) {
    pFunction JumpToApplication;
    
    JumpToApplication = (pFunction)(*(__IO uint32_t*) (FLASH_JUMP_ADDR + 4));
    
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*)FLASH_JUMP_ADDR);
    
    /* Jump to user application */
    JumpToApplication();
}

