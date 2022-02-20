

## 1.组件介绍

ADAMS是一个多体动力学软件，说白了，就是对机构做动力学分析的软件。说得更直截了当一些，它是理论力学以及机械原理，机械振动的辅助分析软件。理论力学中的绝大部分静力学，运动学，动力学问题，可以用ADAMS轻而易举的解决；对于机械原理中的机构分析和设计，ADAMS也是绝佳的辅助工具；至于机械振动中的单自由度系统，多自由度系统的振动，乃至连续体的振动问题，都可以用ADAMS来解决。

除了做分析以外，ADAMS也可用于优化设计，它提供了一整套做试验设计，优化设计的工具。无需任何编程，就可以轻松的对机构做优化设计，因此，它也是机构优化设计的工具。

那么，谁可以学习ADAMS?如何学习ADAMS呢？
先回答第一个问题，谁可以学习ADAMS.
首先，在大二上学期，在学习理论力学的同时，就可以学习ADAMS，用它来做一些动画，帮助自己理解机构；
其次，在大二下学期，在学习机械原理的时候，可以用ADAMS来帮助自己做连杆机构，凸轮机构，齿轮机构的分析与设计；
再次，在大三，学习机械振动的时候，可以用ADAMS来帮助自己理解单自由度系统，两自由度系统，多自由度系统的振动行为。
最后，在学习优化设计的时候，可以用ADMAS来帮助自己做优化设计，学习如何先做试验设计，再做优化设计的流程，从而对于机构的方案设计有一个完整的概念。
第二个问题，如何学习ADAMS呢？
ADAMS是由很多模块组成的，如下图。



<img src="Adams.assets\clip_image001.png"  style="zoom: 33%;" />

### **ADAMS VIEW** 

> 这是我们用得最多的模块，用于建模，分析。

ADAMS/View二进制数据库bin文件以“ .bin”为文件名后缀，文件中记录了从ADAMS启动后到存储为bin文件时的全部信息－包含模型的完整拓扑结构信息、模型仿真信息以及后处理信息。可以包含多个模型、多个分析工况和结果。可以保存ADAMS/View的各种设置信息。文件为二进制不能阅读、编辑，只能通过ADAMS/View调阅，由于信息全面一般文件都比较大。

 

ADAMS/View二进制数据库biq是上一步保存的二进制文件，为防止被覆盖，扩展名由bin改为biq，其实也就是备份的二进制文件。 

 

ADAMS/View命令cmd文件以“ .cmd”为文件名后缀，是由ADAMS/View命令编写的模型文件，可以包含模型的完整拓扑结构信息（包括所有几何信息）、模型仿真信息，为文本文件，可读性强，可以进行编程，是ADAMS的二次开发语言，不包含ADAMS/View的环境设置信息，不包含仿真结果信息，只能包含单个模型。



### **ADAMS/Solver** 

> 是求解器，我们几乎不用直接与它打交道，一个按钮就可以调用。

ADAMS/Solver模型语言（ADAMS Data Language）adm文件，以“ .adm”为文件名后缀，文件中包含模型中拓扑结构信息，但有些几何形体如 link等不能保留。ADAMS/View的环境设置不能保留。ADAMS/Solver可以读取adm文件，与ADAMS/Solver仿真控制语言acf文件配合可以直接利用ADAMS/Solver进行求解。

 

ADAMS/Solver**仿真控制语言acf**文件，以“ .acf”为文件名后缀，文件中可以包含ADAMS/Solver命令对模型进行修改和控制的命令，从而控制仿真的进行。 

 

ADAMS/Solver将**仿真分析结果**中用户定义的输出变量输出到**req文件**，“ .req”；

 

ADAMS/Solver将**仿真分析结果**中将模型的缺省输出变量输出到res文件，“ .res”；

 

ADAMS/Solver将仿真分析结果中**图形部分结果**输出到gra文件，“ .gra”。

 

ADAMS/Solver将仿真分析结果中**用户定义的输出变量以列表的形式输出**到out文件，“ .out”。

 

ADAMS/Solver将仿真过程中的**警告信息**、错误信息输出到msg文件，“.msg”。 



### **POSTPROCESSOR**

后处理模块，用的很多。

### **附加模块** 



<img src="Adams.assets\clip_image001-1603724676979.png" alt=" " style="zoom:50%;" />

Adams flex ——创立柔性单元,柔性体模块

**Adams control ——控制模块，可以与matlab联合仿真**

Adams vibration——振动仿真

**Insight ——是优化设计模块，评价机械系统的性能，实验优化设计，多目标优化。**

Car/Driveline /Classis —— 主要是与汽车某个零部件设计相关的专业模块，





### 基本操作

使用ADMAS基本上就是三步：

* 建模。在VIEW里面。

  机构是由构件通过运动副连接而成的运动链，其中有一个构件是固定在地面上的。因此，先创建构件，然后创建运动副，接着施加驱动，这就可以得到机构。

  构件可以是实体或者是点，线这样的几何元素；运动副就是在机械原理中出现的转动副，移动副这些东西；而驱动无非就是施加转动速度，移动速度。另外，也可以施加驱动力和驱动力偶。这样，机构的模型就建成了，然后就可以开始做分析。

* 分析。在VIEW里面，按动一个按钮启动计算，这实际上是ADAMS的核心。不过我们不需要了解其细节。

* 后处理.在VIEW里面，按动一个按钮就可以进入到该后处理软件，然后也可以在该后处理软件中按一个按钮迅速回到VIEW这个软件。

  查坎构件或者构件上的某个点，它的位移，速度和加速度曲线。或者，是考察运动副上的约束力是如何变化的。所以，它最后求解的实际上是理论力学最关心的内容。





这就是ADAMS。



## 2.建模操作



### 界面view

<img src="Adams.assets\clip_image001-1603725646865.png" style="zoom: 45%;" />

<img src="Adams.assets\clip_image001-1603725686139.png" style="zoom: 50%;" />

### 工具

<img src="Adams.assets\clip_image001-1603725728879.png" style="zoom:75%;" />

<img src="Adams.assets\clip_image001-1603725742754.png" style="zoom:75%;" />





### 设置

<img src="Adams.assets\clip_image001-1603725848867.png" style="zoom: 33%;" />

<img src="Adams.assets\clip_image001-1603726099055.png" style="zoom:33%;" />

<img src="Adams.assets\clip_image001-1603725793573.png" style="zoom: 33%;" />



### 修改模型属性

质量分布

<img src="Adams.assets\clip_image001-1603726143288.png" style="zoom:50%;" /><img src="Adams.assets\clip_image001-1603726160480.png" style="zoom:50%;" />

### 添加驱动

角位移

![](Adams.assets\clip_image001-1603725938458.png)

**齿轮构建**

![](Adams.assets\clip_image001-1603726059335.png)



## 仿真控制



### **测量      参数理解**



**实体**

我们可测量的参数有：

* 质心位置，(角)速度，(角)加速度。

* (角)动能

XYZ 分别表示测量参数在**XYZ方向**上投影的值，而**这个XYZ方向，是下面选择的坐标系的XYZ方向**。即  *指出坐标系在*，不同的坐标系会导致X,Y,Z输出结果不同。如果不选择，默认是大地的坐标即左下角的显示的坐标方向(绿y红x蓝z)

​            <img src="Adams.assets\image-20201027211625884.png" style="zoom: 50%;" />                  ![](Adams.assets\image-20201027212635644.png)  大地坐标

​         如图，圆盘以质心的z轴旋转，我们要检测圆盘的质心角速度则需要选择坐标系为cm，分量为Z 或者默认大地坐标，分量也选择Z。

**注意：虽然在旋转的过程中圆盘上的质心坐标也是跟随着圆盘进行旋转 [参考后处理中的动画-动画控制功能]，以质心坐标为参考坐标系的话，圆盘是不动的。但是根据计算原则，将角速度矢量投影到质心坐标系上也是有值的。**



**移动副** 

可以测量

* 相对速度，相对加速度
* 力，力矩
* 功耗

从/在  选择框

> 由于一个运动副往往连接的是两个实体，所以Adams在创建运动副的时候会新建两个marker到两个实体中，在初始时两个marker时重合的，随着运动，可能会存在一定的偏差。比如移动副就会在marker的原点上有差异，旋转副的话就会在角度上存在一定的差异。检测就是检测这两者的差异

此处选择marker19表明，相对加速度的矢量是以marker19为起点，marker20为终点。

​                          <img src="Adams.assets\image-20201027213645957.png" alt="image-20201027213645957" style="zoom:70%;" />                   <img src="Adams.assets\image-20201027214603055.png" alt="image-20201027214603055" style="zoom: 60%;" />

如图，移动副一个连地，一个连 translate，且 translate 运动方向为大箭头方向。 测量选择 **相对加速度的矢量**是以marker19【地】为起点，marker20【translate】为终点。所以ACC测量时正的。 翻过来就是负的了。





**旋转副** 

我们可测量旋转副的参数有：

* 相对角速度，相对角加速度
* 力，力矩
* 功耗





### Sensor

多次使用sensor，达到某个角度停止仿真，实践测得

1. 输出参数设置，角度逆时针为正，from A to B 代表从A坐标到B坐标的角度，正负由上述转向判断
2. 我们可以添加在ground上的坐标，来方便的定义输出参数，



## 3.后处理



### 动画

动画控制

​                                                                  <img src="Adams.assets\image-20201027213316115.png" alt="image-20201027213316115" style="zoom:70%;" />             <img src="Adams.assets\image-20201027213352519.png" alt="image-20201027213352519" style="zoom:70%;" />                 

动画控制可以一帧一帧的播放仿真动图，并且在运动的过程中显示每个坐标是否跟随着实体进行移动 [勾选图标]。





adams如何将回放过程保存为AVI格式的电影文件

点击plotting(或是f8)进入postprocessor,右键——load ANIMATION,设置record选项卡。按红色的R键，点击play 

图像太垃圾了，还不如录制

<img src="Adams.assets\clip_image001-1603726069628.png" style="zoom:50%;" />



### **Plot Builder** 

> 重中之重是参数的意义

Plot Builder左侧的列表包含可用于绘图的模拟结果。 这些包括对象，度量，请求，结果集和系统模式。 该列表包含您已加载的模型或结果，并设置为查看对象特征。 如果您加载了三种不同的模型，则模型列表如下所示：

.model_1

.model_2

.model_3

 

如果您正在查看请求，度量或结果，则列表将包含您导入Adams PostProcessor的所有模拟的名称。 例如，如果您在model_3上有三个不同的模型和两个模拟，则列表如下所示：

.model_1.Last_Run

.model_2.Last_Run

.model_3.Last_Run

.model_3.Run_001

 

因为您可以一次看到所有的模拟结果，所以您可以轻松地在模拟运行之间绘制结果，甚至在单独模型的模拟之间绘制结果（例如，绘制一个模型与另一个模型的图形加速度）。

 





 

要显示本地最大数据点：

按住Shift键并使用左右箭头键从一个本地最大数据点移动到另一个。

 

要显示本地最小数据点：

按住Ctrl键并使用左右箭头键从一个本地最小数据点移动到另一个。

 

要确定两个数据点之间的距离：

 

1。选择第一个数据点并按住鼠标左键。

2。将光标拖到下一个数据点。

Adams PostProcessor在统计工具栏中显示两个数据点之间的距离。 它将D置于坐标值的前面。 Adams PostProcessor还显示一个MAG文本框，显示光标位移的大小。 幅度是两个坐标值的平方和的平方根。

3。将光标拖到另一个数据点或释放鼠标按钮。

<img src="Adams.assets\clip_image001-1603726347574.png" style="zoom:33%;" />



**导出曲线数据**

<img src="Adams.assets\clip_image001-1603727630284.png" style="zoom: 33%;" />

 



## 4. 破解

adams无法打开的解决方法

<img src="Adams.assets\clip_image001-1603727720499.png" style="zoom:50%;" />







## 5. 实例

### 5.1 内收外展的仿真



http://muchong.com/html/201408/7756883.html 

http://blog.sina.com.cn/s/blog_9e19c10b0101kuhh.html 

 