

## J-Scope使用方法

0. 在使用前，需要生成对应的 `keil project` 的调试信息，因此需要在 Options for Target 的 Output 中选中 Debug Information。

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20201201174737899.png" alt="image-20201201174737899" style="zoom: 80%;" />  

   

1. 设定

   由于是盗版的 `JLink`，所以采样频率最大为 `1KHz`。**其次要注意，如果采样多个变量的值的话，有时采样频率为 `1KHz` 会导致卡顿，此时需要降低采样频率，可取500,  200HZ** 。

   **每次编译后重新烧写到单片机**，最好重新选择 `*.elf/*.axf`。否则，某些变量的值会出错，特别是删除了一些变量后。

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20200223211422353.png"  style="zoom: 50%;" />     <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20201203200332029.png" style="zoom:50%;" /> 

    图 基本需要设定的内容 | 采样频率设定为250Hz，可顺畅的采集10个变量

   

   当 Jlink 设备只使用 SWD 相关的口与单片机连接 时，`Target Interface speed` 需要选择 SWD

   

2. 选择设备名称 - STM32F103RB, STM32F427VI

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\J-Scope使用方法.png" style="zoom: 50%;" /> 



3. 工具栏

<img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20200223211743681.png" alt="image-20200223211743681" style="zoom:67%;" /> 

**放大器** 可以放大水平轴的分辨率。或者选中对应的曲线后直接鼠标滚轮+Ctrl；

**触发方式** 选择双边触发可以提高采样点数

红色的点是运行开关，注意运行前查看 `Jlink` 是否在 `Keil` 中调式，两者是不能同时使用的



4. 示波器显示界面

<img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20200223211938627.png"  style="zoom: 50%;" /> 

​	可用于测量时间，中间有刻度的轴是零轴和交叉点是原点。移动浮标可以计算浮标到零轴的差值。

​	**鼠标按住拖动**，可以精准的将波形边沿移动到零轴上，进行测量

​	由于软件问题，无法将原点移除界面外，所以观察较大的值，不利于观察



5. 显示器下部

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20200223212207208.png"   style="zoom:67%;" /> 

   紫色区： 显示浮标所对应的值

   橙色区：波形的上下限

   蓝色区：按下+，-可以增大减小y轴的分辨率

   

6. 添加追踪变量

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20200223212519695.png"   style="zoom: 50%;" /> 

   注意

   * static类型 和 float类型 显示会一些问题，会出现莫名其妙的问题，绝对不要使用。static可以改为全局变量，float可以乘上较大的数，如1000，转化为 int 类型
   * 比较大的数难以观察，如32位的时间戳，最好别观察，可以通过 取模运算%，将变量转化到**0-合适的值**上
   * 改变过变量名后，比如上面的一些操作，必须要重新添加追踪变量

   > 最多只能添加10个变量

7. 导出数据

   <img src="H:\0-重要文档\robotmaster\电控\1-培训\Readme.assets\image-20201203221802043.png" style="zoom:33%;" /> 

   Jscope 可以导出为csv格式的数据文件，数据之间，数据标签之间以“；”间隔。

   如果要将数据导入到excel 表格中进行处理，可以将“；”替换为 tab 制表符，直接黏贴到excel文件中。 


