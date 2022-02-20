

## 0. AD 库

*.bxl文件转化为AD库
https://blog.csdn.net/cleverhorse/article/details/100732366

**file - new - project  created lib**

**new** **原理图，** **PCB** 

### 0.1 元器件封装



#### 0.1.3 3D模型

place - 3D body

<img src="Altium Designer.assets\image-20201028204158714.png" style="zoom:20%;" />           <img src="Altium Designer.assets\image-20201028204236846.png" alt="image-20201028204236846" style="zoom:40%;" />

导入时的坐标系和建模时的坐标系是一致的，所以最好在建模的时候在基准面上建模





​       <img src="Altium Designer.assets\image-20201028195917590.png" style="zoom: 33%;" />            <img src="Altium Designer.assets\image-20201028200044651.png" style="zoom: 25%;" />

如果 standoff Height 为0，或者较小。则，在布置PCB元件的时候，会报错。



**关于AD里面3d的制作**

https://blog.csdn.net/jhn199388/article/details/74182278



**从AltiumDesigner导出电路完美3D模型至Solidworks的方法**

https://blog.csdn.net/quinn1994/article/details/89437259



### 0.2 元件图

#### 0.2.1 基操

AD 软件要求器件需要与格子对齐。摆放前，设置越小的间隔，移动精度越大。

<img src="Altium Designer.assets/image-20201204194148642.png" style="zoom: 50%;" /> 



放置芯片引脚时，可以按tab键进行设置，注意摆放时要热点朝外

<img src="Altium Designer.assets/image-20201204194138812.png" style="zoom: 50%;" />     <img src="Altium Designer.assets/image-20201204194124701.png" style="zoom:50%;" />    

 

#### 0.2.1 **超大元件原理图的绘制**

对于有上百个引脚的芯片，我们不可能一个个的手动去布置。我们可以用excel软件等，帮我们快速的建立芯片的各个引脚的位置表格，然后进入SCHLIB List ，智能栅格粘贴，然后将表格复制过去就可以了。

同样的PCBLIB List 也能绘制超大的元件PCB封装



#### 0.2.5 绑定封装

<img src="Altium Designer.assets/image-20201204194351392.png" style="zoom:67%;" /> 



## 1. PCB

### 1.0 布置元件

**一般原则** 

* 一个模块的元器件在美观的情况下**尽可能靠近** 



> **PCB 外框确定** 

在 keepout 层 ( 粉色 ) , 用普通线 Line 绘制 PCB 封闭区域。然后 [右键 - Find Similar Objects...] 选择Keepout 层

上的图形，最后选择 [Design  ->  Board Shape  -> Define from selected objects ]



> E+O+S  设置参考点

用于绘制精确的长度，特别是PCB板子形状，还有与其他板子之间的接口，很有帮助。



> Tool – component placement – arrange within rectangle

可以方便的在原理图中选中对应的元件，然后在PCB文件中，使用这种方法布置，快速的将对应的元件定位到，鼠标所画的方框中



> 垂直、水平镜像翻转、旋转元件

移动器件的时候按键盘的“X”键即可水平翻转器件，按“Y”键为垂直翻转。按空格“Space”可以旋转90度。同时别忘了关掉中文输入法。

https://jingyan.baidu.com/article/93f9803f40d24ee0e46f5503.html



> 旋转任意元器件角度布置

Edit --->Move ---> Rotate Selection 或者使用快捷键
E+M+O



> 隐藏 PCB 栅栏图案  PCB界面中右键，选择Option - Grid Manager   双击在 Display 界面选择 

<img src="Altium Designer.assets/image-20200917001029692.png"  style="zoom: 50%;" /> 



### 1.1 布线准备

> PCB 布线之前需要对，某些器件或者某些功能的布线的属性需要进行特殊配置。便于布线，也便于查错.



#### 1.1.0 PCB层设置

> 导线厚度和层数设置

Design-layer stack manager



#### 1.1.1 RULE 作用域

你新建的每一个 rule 都是有对应的作用域的,只是默认为所有的 net 。

* 你可以新建多个线宽的 rule 分别作用于不同的作用域。
  **比如，信号线宽20mil**，电源线宽**40mil**；信号线**via 25mil 18mil**，电源线 **via 50mil 38mil**。
  
* 作用域可以用 class (net 的集合) 表示。表示这个 net 集合中的 布线属性。
  
  Class 的新建在 PCB 文件的 design-class 中。
  
  注意从原理图更新到 PCB 时,不要勾上 remove class
  
* **RULE在检查的过程中是按优先级高到低进行的**。所以优先级高的往往设置成针对某个net的rule规则，优先级低的设置成针对所有的net的规则，这样可以实现某些net之外的net的属性定义。*譬如要设置信号线宽  20mil ,电源线宽 40mil。则设置优先级高的为电源相关 NET (GND,VCC,VEE,VDD) 的线宽为 40mil，优先级低的可以是针对所有 NET的线宽为 20mil。* 



#### 1.1.2 RULE配置

通孔 pad 比布线宽稍大。——大小 25mil ,布线宽 20mil

直插焊接孔, 铜直径 / 孔直径 = 1.5mil ～ 3.0mil

低速板,布线宽可以大一些

通常电源线和地线可以宽一些,汇流

<img src="Altium Designer.assets\image-20201029103033079.png" alt="image-20201029103033079" style="zoom: 50%;" /> 



<img src="Altium Designer.assets\image-20201029105821435.png" alt="image-20201029105821435" style="zoom: 50%;" /> 





### 1.2 布线

> 隐藏部分元器件的飞线

View - Connection - Hide Component Net    然后对应的元器件，即可屏蔽元器件的飞线



#### 1.2.1 自动布线

自动布线前，必须配置的RULE参数为：

 <img src="Altium Designer.assets\image-20201029110638940.png" alt="image-20201029110638940" style="zoom:67%;" /> 

RULE——规则的最小值和最大值必须设置为一样，否则无法进行自动布线

 <img src="Altium Designer.assets\图片1.png"  style="zoom: 33%;" /> 



**手动修改：手动布线，消去自动布线的一些不合理的地方**

<img src="Altium Designer.assets\image-20201029110743963.png" alt="image-20201029110743963" style="zoom: 25%;" />  |    <img src="Altium Designer.assets\image-20201029110154924.png" alt="image-20201029110154924" style="zoom: 33%;" />

图 两个通孔可以合并的需要合并 |  重合的两根线需要删去一根



#### 1.2.2 手动布线



手动布线的法则

•布线前，原理图的连接方式必须十分清楚，这是手动布线的关键

•PCB按照原理图的方式排列封装，方便布线

•使用ctrl+左键点击引脚，光亮显示相同标号的引脚和连线，方便布线

​     <img src="Altium Designer.assets\image-20201029110846283.png" style="zoom:50%;" />     <img src="Altium Designer.assets\image-20201029110904594.png" style="zoom:50%;" />

**designer** **找标号的芯片： e+j+c**



1、 快捷键 ：**T+C**

这个快捷键在原理图和PCB图中都适用，这是**交叉查找**指针的快捷键，使用了这个功能后，会在鼠标上生成一个十字叉。用这个十字叉，在原理图中任意选择一个元件后，就可以找到元件在PCB中对应的封装。 在PCB文件中任意选择一个元件后，就可以找到原理图中对应的元件。

操作如下图所示：

退出此模式：在原理图点 先右键点击一下，此时十字叉消失，然后任意单击一个元器件即可退出此模式

2、 快捷键：**T+S**

这个快捷键是交叉显示功能的快捷键，它只在原理图中可以使用，在PCB文件中也有交叉显示这个功能，但是快捷键被占用了，所以只能手动选择。



首先，在原理图中选中一个元件，此时元件会被一个绿色的框框住，然后按下T+S，就会在PCB文件中显示对应的元件了。显示的位置一般会靠右边一点



#### Via 和 Pad 的区别

https://blog.csdn.net/pkuyjxu/article/details/6733804

Via   用于电气连接不同层，但是它不与元器件相连。

Pad 用于连接元器件还起到机械固定的作用



### 1.3 铺铜

E-M-G 选择要修改的铺铜区域，然后单击锚点，放开拖动即可。

2. ALtium designer快捷键

隐藏铺铜：1.ctrl+D或者按L选择show/hide选项卡



### 1.4 检验

检验PCB板子  [Reports - Board Information] 

检验所有Net 是否都连接成功，板子大小，板子层数和各层厚度，以及遍布多隔层的via个数(在Multi-Layer上)



### 1.5 Gerber加工文件





## 2. 原理图



**ALtium原理图更新时，未改变元件，但PCB出现改动的问题**

https://blog.csdn.net/abc87891842/article/details/52538660?utm_medium=distribute.pc_relevant.none-task-blog-title-3&spm=1001.2101.3001.4242

https://blog.csdn.net/jiangchao3392/article/details/79918772 



**修改图纸大小** 

方法一：

双击原理图纸框外的空白区域

方法二：

在工具栏中 -> 点击Design -> 点击Document Options -> 找到Standard style 模块 -> 调整原理图尺寸A4/A3/A2…



**给每个元件标号**　

<img src="Altium Designer.assets/image-20201127213327719.png" alt="image-20201127213327719" style="zoom: 30%;" />  <img src="Altium Designer.assets/image-20201127213312229.png" style="zoom: 40%;" />  <img src="Altium Designer.assets/image-20201127213428963.png" style="zoom:30%;" />

图 方法一: 快捷标号  |  方法二: 设定标号顺序 | 执行标号



**如何批量修改属性** 

<img src="Altium Designer.assets/image-20201127213605454.png" style="zoom:40%;" />    <img src="Altium Designer.assets/image-20201127213616053.png" style="zoom:30%;" />  <img src="Altium Designer.assets/image-20201127213701131.png" style="zoom:40%;" />

图 1. 选择单一元件 | 2. 选择同属性的元件  |  3. SCH INspector中批量修改属性



**序号（designator）去查找元件**

按下jc，然后输入序号就可以了



**查找错误** 

新建工程然后添加入 .SchDoc 草图文件和PcbDoc封装文件才能进行编译！！！

<img src="Altium Designer.assets/image-20201127213930786.png" style="zoom:50%;" />  <img src="Altium Designer.assets/image-20201127214059985.png" style="zoom:33%;" /> <img src="Altium Designer.assets/image-20201127214108546.png" style="zoom:23%;" /> <img src="Altium Designer.assets/image-20201127214122045.png" style="zoom:23%;" />

图 编译原理图 |  net float at xxx，xxx。某个标签没有连接上，那串数字表示该标签的坐标 | 对应的悬浮的VCC标签 | 其次要注意这种连线

 

 

## 3. 快速上手



#### 通用操作

按住鼠标右键拖动可平移

按住鼠标中键拖动可放大

tab 键设置属性

右键取消正在执行的命令



#### 快捷键



**自定义快捷键**

Ctrl+ 对应的功能按钮,就能弹出对应功能的快捷键设置
很方便





#### 原理图

1.  使用原件序号( designator )查找原件，如 R10, U5；按下 j+c，弹出查询输入框，然后输入序号就可以了。




