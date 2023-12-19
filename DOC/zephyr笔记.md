# Zephyr学习笔记
--------------------------------------
## 一、开发环境的搭建
west工具:类似git \
west init+west updtate可以从github上下载zephyr源码。\

west工具也可扩展：\
West  build:扩展指令\
```mkdir build & cd build cmake -GNinja -DBOARD=<boards> ninja ...```    
```cmake -B build_dir -S source_dir -G Ninja -DBOARD = board```\
[参考链接](https://blog.csdn.net/My_CSDN_IT/article/details/118180074)

## SDK结构
```
.
├── bootloader
├── modules
├── small_project
├── tools
└── zephyr

```
其中```small_project```为用户程序

## 用户程序```small_project```

```
.
├── CMakeLists.txt
├── boards
├── build_dir
└── src
```
boards：        用户PCBA板\
build_dir:      各种输出文件\
src：           用户程序\
在当前目录下,使用```west build -b stm32f030_demo -d build_dir```命令编译，stm32f030_deom是zephyr本身支持的开发板

### ```CMakeLists.txt```分析:
```CMake
1、make_minimum_required(VERSION 3.13.1)
2、find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
3、project(zephyr_prj)
4、target_sources(app PRIVATE src/main.c)
5、zephyr_include_directories(src)
```

<font color=red>第二行：</font>是表示在 ZEPHYR_BASE 目录中寻找命名为 Zephyr 的模块。
```$ENV{ZEPHYR_BASE}```设计了 zephyr的根目录路径。
可以通过```message("--------_DIR=$ENV{ZEPHYR_BASE}--------")```打印出来```_DIR=/home/zager/softproject/zephyr_test/zephyr```


