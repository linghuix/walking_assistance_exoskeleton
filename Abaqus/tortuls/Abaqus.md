Abaqus 自底至顶网格划分后怎么将网格映射到实体上 ？

Abaqus理论检测



单位问题

> 仿真软件并不会涉及单位，在仿真之前需要**规定所有的单位为国际标准单位**，譬如 **帕斯卡，米，牛顿，秒**。



## 仿真步骤[1]

​                      <img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\model tree.png" style="zoom: 67%;" />                   <img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\1.建模.png" style="zoom:67%;" />     <img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\image-20201101160808886.png" style="zoom:67%;" />

​                                          **Fig 1**																				**Fig 2**													**Fig 3**



1. 通过 cae 软件建立模型，**Fig2** 利用拉伸可以实现类似 solidworks 的功能，组成复杂的模型。

2. 材料属性定义
3. 材料横截面定义，添加横截面到步骤一建立的实体中（刚体不需要）
4. 将不同的part装配 **Fig 3**，只有一个 part 时也必须组合
5. 网格划分。
   * 自底向上
     * 默认为自顶向下，需要从 `划分网格控制中` 选择 **region** ，然后选择 `from bottom to up`
     * 需要注意的是，采用自底向上 `from bottom to up` ，网格划分后，网格和实体是分开的，要将网格映射到实体上之后，实体才会消失，划分的网格也才能用于后续的计算 ，就必须使用如下菜单中的`链接功能`
   * 自顶向下
     * 该模式中也可以通过散布seed，来干预自动的网格划分密度。

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\5 mesh.png" style="zoom: 80%;" />

6. 添加分析步和其中的力和约束

   

7. 输出配置

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\7 输出 结果.png" style="zoom:67%;" />

8. 结果展示

   legend字体调节 ： Viewport - Viewport Annotation Options

   视角精确调整 ： view - Specify...     选择Viewpoint - Method， Viewpoint是面向你的向量，Upvector是朝向上的向量。

   二维平面变三维，以增强某种属性的在表面的分布区域 ： View - ODB Display Options - Sweep - extrude elements

   查看每个step的执行结果和每个增量步的数据 ： Output Databases - odb - steps

   显示某个属性的分布图，场数据的分布图 ： result - feild output

   显示某个属性随时间变化的曲线 ： result - history output

   显示某个属性分布图随时间变化的曲线 ： animate - time history / 点击上下文面板中的动画按钮

   <img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\results" alt="image-20191224185813655" style="zoom: 67%;" />

   | 属性     | 描述                                       |
   | -------- | ------------------------------------------ |
   | CFN1,2,3 | contact force  接触面反力，分别代表XYZ方向 |
   | CFNM     | 接触面合力(不加上摩擦力)                   |
   | CPRESS   | 接触面压强                                 |
   | CFS1,2,3 | 接触面的摩擦力分量                         |
   | CFSM     | 接触面的摩擦合力                           |
   | CFN      | CFNM+CFSM 接触面总的合力                   |
   |          |                                            |




关于ABAQUS动力分析中初速度的两种设置方式   http://muchong.com/t-7324144-1-pid-16

在显示分析中，不能用场的方式进行速度设定，只能用约束的方式设定速度，但是这样就物体就只能以一定的速度运动了，而不是遇到阻碍之后衰减到零，与实际情况不符合。



## 实际仿真测试 

模型:150mmx5mmx2.5mm的长条状物体。

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\模型.png" style="zoom:50%;" />

理论计算结果：rou = Pl^3/(3EI) = 3.09mm  其中 I = bd^3/12,b宽为2.5mm，高5mm

仿真结果：输入弹性模量单位Pa，力N，模型尺寸mm，最终输出的变形位移为1E-6mm.

单元类型：C3D8R

稀疏网格：**最大形变量158.6mm，结果与理论值完全不同** 

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\稀疏.png" style="zoom: 50%;" />

低密度网格：**最大形变量4.088mm，结果接近理论值**

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\低密度.png" style="zoom: 50%;" />

中密度网格：**最大形变量3.215mm，结果符合理论值**

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\中密集.png" style="zoom:50%;" />

超密集网格划分：**最大形变量3.096mm，结果很好的符合理论值**

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\超密集划分网格.png" style="zoom:50%;" />



单元类型：C3D20R 

无法运行





单元类型：C3D20

**最大形变量3.086mm，结果很好的符合理论值**

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\稀疏-C3D8R.png" style="zoom:50%;" />

三角形

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\三角形.png" style="zoom:50%;" />





## 冲压仿真

1. 建立模型

   新建一个可变形的部件作为冲压对象， 三个刚体部件作为冲压头，冲压台，夹具

2. 设定材料

   弹性模量，塑性形变

3. 设定材料截面属性，赋值给冲压对象

   solid homogeneous

4. 建立冲压对象坐标

5. 装配

   添加定位约束、

6. 创建点集合

7. 创建分析步骤

   step1 ：

8. 设定监控输出  

   监控冲头下方的冲压件上的位移

9. 设定接触属性和接触面对

   冲头与冲压件没有摩擦，夹具与冲压件有摩擦，模具与冲压件有摩擦

<img src="E:\101-WorkShop\simulation\Abaqus\tortuls\Abaqus.assets\interaction1.png" alt="image-20191223223414844" style="zoom: 67%;" />

10. 定义个分析步骤的边界条件
11. 划分网格



[1] 基于abaqus 的有限元分析和应用  庄茁

