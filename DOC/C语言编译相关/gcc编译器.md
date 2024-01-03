## gcc常用的编译选项
https://blog.csdn.net/bandaoyu/article/details/115419255

### .hex/bin/dis文件的生成
`arm-none-eabi-objcopy -O ihex xxx.elf xxx.hex`

`arm-none-eabi-objcopy -O binary  -S xxx.elf xxx.hex`

`arm-none-eabi-objdump -D xxx.elf > xxx.dis`