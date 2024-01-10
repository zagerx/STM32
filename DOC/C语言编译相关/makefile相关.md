## 特殊符号

`$<`：第一个依赖文件 `$@`：目标文件 `$^`:所有的依赖文件

## 常用函数

|函数|解释|示例|
|:----:|:----:|:-:|
|notdir|剥离文件的绝对路径，只保留文件名|
|addprefix|剥离文件的绝对路径，只保留文件名|$(addprefix src/,foo bar) <br> 返回:“src/foo src/bar”|
|wildcard|获取所有.c文件|$(wildcard ../YYY/*.c)
