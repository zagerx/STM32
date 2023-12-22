## gcc常用的编译选项
    https://blog.csdn.net/bandaoyu/article/details/115419255

## .ld文件

### .ld文件中的地址问题
   获取'.txt'section的末尾\
    在任意的.c文件内声明一个指针变量。
    `extren void *_edata_load;`
    在.ld文件内，定义该变量并对其赋值

``` C
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
    _edata_load = LOADADDR(.data) + SIZEOF(.data) -4;    /* Define address of the last word of this section, which is also the last word in the flash. */
  } >RAM AT> ROM
```

在.c文件中打印该指针变量的地址

``` C
#define TEST_ROM_END_ADDR  (uint32_t)&_edata_load
USER_DEBUG_NORMAL("TEST_ROM_END_ADDR = 0x%08x\r\n",TEST_ROM_END_ADDR);
TEST_ROM_END_ADDR = 0x08031A17
```  

### LOADADDR关键字

VMA(Vitual Memory Address)运行地址\
LMA(Load   Memory Address) 装载地址

`LOADADDR(.data) = `

### NOLOAD关键字
(NOLOAD):该section在程序运行时，不被载入内存
``` C
backup_buffer_section (NOLOAD): { *(backup_buffer_section) } >RAM  
```
