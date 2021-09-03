



### 介绍

SolidWorks 属于建模仿真类的软件。其建模界面方便简洁，提供渲染功能和运动仿真功能。便于展示精美的结构和使用方法动画。

美中不足的是，不同的版本互相不兼容。只能到处IGES等类型才能互相传输文件。







### 零件

**螺纹** 

[三种方法创建螺纹](SolidWorks.assets\三种方法创建螺纹.txt) 





### **装配体**

镜像实体后，右键单击可以解除镜向零部件特征。就会保留镜向实体，重建匹配特征。

<img src="SolidWorks.assets\clip_image001-1603679961273.png" style="zoom:50%;" />



**装配体出错修改** 

零件修改后，装配体出错。查看**出错**的地方一般有

* 配合出错。

* 草图几何模型缺失。

  重绘草图，修改褐色线条

  选中褐色线条或者点，删除褐色约束就可以了。**褐色的点有时候很难看到**

* 草图平面失缺。

  右键草图，编辑草图平面

注：有时错误过多时，可以拖动下方的手柄控制线，向上拖动到需要修改的地方，然后一点一点的向下拖动，一步一步修改出现的错误

​	实在看不出来就右键草图，举例外部参考，断开所有的外部参考



**装配体修改** 

- 装配体配合需要局部改动（移动位置）

  压缩配合，添加新的配合以便于移动，修改好后，解压配合。

- 需要依靠其他的零件确定尺寸的，可以在装配体中新建一个草图，绘制草图比较尺寸

  

### **标准件** 



**添加标准件**

将toolbox中的标准件拖动到装备体中，会自动弹出参数设置窗口，设置好参数，可以点击添加（方便之后重新选择这个零件使用），然后打钩，即可。  

有时之前生成的标准件会莫名其妙的变得和后来添加的零件参数相同，这时可以在[标准件目录](E:\SOLIDWORKS Data\CopiedParts)中找到标准件，打开后复制草图自己做一个零件，这样就不用各种依赖关系搞得一不小心就会改掉原来设置好的参数。



**修改标准件** 

选中标准件右键，必须要先打开 _toolbox_ ! 选择编辑标准件

<img src="SolidWorks.assets\未命名图片.png" alt="未命名图片" style="zoom:40%;" />



### **工程图** 

导入装配体或者零件，最右边的按钮或者视图布局。里面可以选择标准视图，各种视图



**标注尺寸** 

***注解***  选择 ***模型项目***，可以将装配体中的尺寸标注转移到视图上。选择自动添加或者手动添加

按住Ctrl可将尺寸标注拖动到其他视图上，就可以实现尺寸标注对象的转移



**线型**

选中一个视图，可以改变显示样式以显示内部的线段（虚线）。

更改尺寸标注数字的大小，可以框选所有尺寸，选择其他，改变字体高度

 

<img src="SolidWorks.assets\clip_image001.png" style="zoom: 45%;" />

 

断开剖视图

​     <img src="SolidWorks.assets\clip_image002.png" style="zoom: 33%;" /><img src="SolidWorks.assets\clip_image003-1603678792690.png" style="zoom: 45%;" />

 

<img src="SolidWorks.assets\clip_image004.png" style="zoom:40%;" />

当然如果不知道参数的话，可以在深度参考选项中选择俯视图中孔的轮廓线，也可以进局部剖视，然后刷新一下，就完成剖视

<img src="SolidWorks.assets\clip_image005.png" alt="。  饧 到 勿  0 " style="zoom: 40%;" />



**材料明细表** 

<img src="SolidWorks.assets\clip_image001-1603680632201.png" style="zoom: 50%;" />



<img src="SolidWorks.assets\clip_image001-1603680688645.png" alt=" " style="zoom: 33%;" />



<img src="SolidWorks.assets\clip_image003-1603680649638.png" style="zoom: 35%;" />



怎么将solidworks材料明细表变成两列 ？

鼠标点到欲分割的栏目，选定；

按右键出现菜单；

到分割--横向上  点击；

把已经分开的上部明细表移动到需要的位置；



### **动画** 

solid动画只要拖动零件，就可以计算出响应约束的其他部件的运动

 

红色时间轴表示运动解算器故障。如果时间轴是红色，尝试以下之一：

- 压缩或移除没有必要的配合。
- 简化配合。
- 以其它方法指定运动，如通过使用马达，而不是通过使用零部件键码。



**配合**

只要展开配合，就可以设置每一种配合的持续时间。可以在某一时刻删除某个配合，该点就会自动生成一个节点。节点之间如果有色带连接，说明这一段改零件或者配合是有效的。选中色带，压缩配合就可以使配合在这一条色带中失效。

注意：每次修改之后需要计算之后才会显示正确的运动方式



### **Simulation 受力分析**

在工具（tool）中选择插件 `Solidwork Simulation` / 或者选中 `SolidWorks Add-Ins` 中的 `Solidwork Simulation `.

选择 `Simulation` 选项卡，然后就可以使用 `simulation `来进行受力分析了。



添加载荷，选力，此时弹出选项菜单，选择分割，点击生成草图，选择你要添加力的位置面，用草图功能在面上画你需要添加受力的区域面积，后退出草图，选择面，生成分割，完成添加力



### **技巧和常识** 



> 将含有标准件的装配体发给别人时，选中装配体文档，右键打包，就可以了



> 如果你想要将一个装配体修改一点点，安装在二个不同物体上。

注意，任何在总装配体中的操作都会直接影响子装配体，比如在装配体中删除一个子装配体中的一个零件。那么这个零件在这个子装配体中就会永远不存在了。

要独立的改变子装配体（仅仅是在这个总装配体中显示某种特性），必须在装配体中拷贝一份，另存为一个新的子装配体，然后在新的装配体中修改。或者右键选择独立保存

 

高级方法：

<img src="SolidWorks.assets\clip_image001-1603680351571.png" style="zoom:50%;" />



零件名字有[ ]方括号代表零件保存在装配体内部，可以右键指定外部存储地址

**** 

**SolidWorks提示默认模板无效如何解决**

https://jingyan.baidu.com/article/597a06430756ca312b5243cd.html