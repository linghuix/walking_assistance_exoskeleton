Abaqus 自底至顶网格划分后怎么将网格映射到实体上 ？

Abaqus理论检测



### 仿真步骤

​     <img src="Abaqus.assets\model tree.png" style="zoom: 45%;" />        <img src="Abaqus.assets\1.建模.png" style="zoom:40%;" />     <img src="Abaqus.assets\image-20201101160808886.png" style="zoom:50%;" />

​       **Fig 1**													**Fig 2**											**Fig 3**



1. 通过 cae 软件**建立模型**，**Fig2** 利用拉伸可以实现类似 solidworks 的功能，组成复杂的模型。

2. **材料属性**定义
3. **材料横截面**定义，添加横截面到步骤一建立的实体中（刚体不需要）
4. 将不同的 part **装配** **Fig 3**，只有一个 part 时也必须组合
5. **网格**划分
   * 自底向上
     * 默认为自顶向下，需要从 `划分网格控制中` 选择 **region** ，然后选择 `from bottom to up`
     * 需要注意的是，采用自底向上 `from bottom to up` ，网格划分后，网格和实体是分开的，要将网格映射到实体上之后，实体才会消失，划分的网格也才能用于后续的计算 ，就必须使用如下菜单中的`链接功能`
   * 自顶向下
     * 通过散布seed，来干预自动的网格划分密度
     * 自顶向下划分即可自动的实现划分

网格划分好之后需要对**赋予网格单元类型** 

​             <img src="Abaqus.assets\5 mesh.png" style="zoom: 60%;" />   <img src="Abaqus.assets\7 输出 结果.png" style="zoom:55%;" />  <img src="Abaqus.assets\results" style="zoom: 55%;" />

​                             自顶向下划分网格   |    输出配置   |   输出展示

6. 添加分析步和其中的力和约束

   

7. 输出配置

   

8. 结果展示

   legend字体调节 ： Viewport - Viewport Annotation Options

   视角精确调整 ： view - Specify...     选择Viewpoint - Method， Viewpoint是面向你的向量，Upvector是朝向上的向量。

   二维平面变三维，以增强某种属性的在表面的分布区域 ： View - ODB Display Options - Sweep - extrude elements

   查看每个step的执行结果和每个增量步的数据 ： Output Databases - odb - steps

   显示某个属性的分布图，场数据的分布图 ： result - feild output

   显示某个属性随时间变化的曲线 ： result - history output

   显示某个属性分布图随时间变化的曲线 ： animate - time history / 点击上下文面板中的动画按钮

   

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





### 单位问题

> 仿真软件并不会涉及单位，在仿真之前需要**规定所有的单位为国际标准单位**，譬如 **帕斯卡，米，牛顿，秒**。



### 网格类型划分

Mesh controls 可以查看当前能够划分的网格类型



橙色 - 只能划分四面体/三角形

绿色- 表示可划分结构化网格



**案例 新建的 shell Part 的四边形网格划分** 

<img src="Abaqus.assets\image-20201114162558822.png" alt="image-20201114162558822" style="zoom:20%;" /> <img src="Abaqus.assets\image-20201114162455698.png" style="zoom:33%;" /> <img src="Abaqus.assets\image-20201114162529321.png" style="zoom:20%;" /> <img src="Abaqus.assets\image-20201114162253690.png" alt="image-20201114162253690" style="zoom: 17%;" />

图 shell类型的part | 四边形网格划分控制面板 | part呈现绿色表明可获得结构化网格 | 结构化网格



 可赋予 S8, S8R, S8R5 , S4R, S4单元类型

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114165351626.png" alt="image-20201114165351626" style="zoom:25%;" />   <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114165421323.png" alt="image-20201114165421323" style="zoom:22%;" />  <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114165532423.png" alt="image-20201114165532423" style="zoom:19%;" />

图 S4R  |  S8R  |  S8R5 （显示方法 - Part-sets - node - 选择node） 



**案例 新建的 shell Part 的三边形网格划分** 

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114165755851.png" alt="image-20201114165755851" style="zoom:33%;" /> <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114165830820.png" alt="image-20201114165830820" style="zoom: 20%;" />

图  三边形网格划分控制面板 | 结构化网格



可赋予 S3 , STRI65  单元类型

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114170026385.png" alt="image-20201114170026385" style="zoom:25%;" />   <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114170124680.png" alt="image-20201114170124680" style="zoom:28%;" />

图 STRI65  | S3 







### 网格单元类型

线性(一阶)四面体和三角形精度很烂。



<img src="Abaqus.assets\image-20201114155457345.png" style="zoom: 50%;" /> 



#### **一阶单元** 

**非协调模式** 

**C3D8I**  融合了一阶缩减积分和完全积分的优势。 但是不能承受扭曲分析[1 4应用实体单元]。 

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114160318359.png" style="zoom:67%;" />  



#### **二阶单元**

完全积分要慎用，结果更加精确，但是会存在体积锁定。

**致命缺陷** 

二阶单元的致命缺陷绝对不能用于接触分析，中间节点会出现异常应力。

**C3D10M** 是例外，原因是 C3D10M 是修正四面体单元，包含10个节点。在复杂模型中使用。因为，复杂模型不能进行一阶六面体划分，而一阶四面体又没有精度，所以只能用二阶的四面体。唯一缺点是计算成本较大。





#### 单元选择

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114174254286.png" alt="image-20201114174254286" style="zoom: 50%;" /> 



**沙漏**

主要出现在 **一阶缩减积分单元** 收到弯曲作用时。由于单元类型是属于缩减单元，因此仅在单元的中心点有一个积分点，上图的虚线焦点。当受力矩弯曲时，虚线长度没有变，相交角度也没有变化，在仿真中认为是没有发生应变，因此也就没有应力(抵抗外力的力)，导致一点点外力矩就会出现很大的变形量。

**缓解沙漏方法**

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114203732105.png" alt="image-20201114203732105" style="zoom: 25%;" />    <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114203828749.png" alt="image-20201114203828749" style="zoom:25%;" />

图 增强减弱刚度 | 自定义刚度

* 增加 abaqus 中**一阶缩减单元**的刚度  效果可以 
* 加密网格划分  效果显著 
* 施加的力/约束不要集中到一个node上，最好分布在多个node上  决定沙漏优化的上限 

**验证是否发生沙漏** 

* 观察单元格形状是否是梯形，类似沙漏的形状

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114200613701.png" alt="image-20201114200613701" style="zoom: 33%;" /> 

* 伪应变能 Artificial strain energy 和 内能 internal energy 之间的比值。官方要求小于 2% 为佳，实际工程一般小于 5%。

<img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114195641958.png" style="zoom: 20%;" />  <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114195859552.png" alt="image-20201114195859552" style="zoom: 20%;" />  <img src="E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114195912948.png" style="zoom: 25%;" />

图  历史输出中的伪应变能和内能 | 两者比值计算 | 比值百分比曲线，达到了100%，几乎没有精度



**剪切锁定** 

主要出现在 **一阶完全积分单元** 收到纯弯曲作用时。此单元的有四个积分点，收到弯曲发生形变时，由于垂直的虚线发生了角度变化，导致出现了额外的剪切力（实际不存在）。导致单元在较大的外力矩作用下，出现低于实际的变形。



**体积锁定 **

出现于二阶完全积分单元的不可压缩材料受到大应力的仿真中。



**选择总结** 

接触分析  二阶六面体会出现中间节点异常应力，因此不适合

大弯曲分析  一阶完全积分单元会出现剪切锁定，一阶不完全积分会出现沙漏；二阶单元精度高，一阶非协调单元时间成本低

大弯曲，接触分析  除了大弯曲无法使用的单元，还有二阶单元。一阶非协调单元融合了一阶的完全积分和缩减积分特性，因此适合

分析中存在应力集中  二阶单元精度更好更适合

不可压缩材料，存在大应变  二阶完全积分单元存在体积锁定不合适

不可压缩单元（橡胶） 杂交单元[1 4.14杂交单元]

 

![](E:\101-WorkShop\simulation\software-manual\manual\Abaqus.assets\image-20201114192841113.png)



### 仿真设置

**仿真增量步长设置** 

* 时间周期：表明该仿真 Step 的仿真总时间；

* 需要**大变形** (材料变形，几何变形，边界/[状态改变非线性](https://zhuanlan.zhihu.com/p/28212568)) [1 8.1非线性来源] 迭代时，在basic 中可以开启 Nlgeom；

* 初始时间增量 0.1，如果只需要运行一个时间增量步，则可以设置初始时间增量为仿真总时间[1 P178]；



  <img src="Abaqus.assets\image-20201113173925843.png"   style="zoom:30%;" />    <img src="Abaqus.assets\image-20201113171846924.png"   style="zoom: 30%;" />    <img src="Abaqus.assets\image-20201113172311900.png"  style="zoom: 30%;" />

   Steps-basic     |     Increment      |      Other

**设置对实际仿真的影响**

> jobs - monitor

Step 1 - time period = 1 | inital increment size = 1

Step 2 - time period = 1 | inital increment size = 1

Step 3 - time period = 1 | inital increment size = 1

Step 4 - time period = 1 | inital increment size = 0.1

Step 5 - time period = 1 | inital increment size = 0.0001

<img src="Abaqus.assets\image-20201113201233862.png" style="zoom:25%;" /> 



​        

### 实际仿真测试

模型:150mmx5mmx2.5mm的长条状物体。

<img src="Abaqus.assets\模型.png" style="zoom:30%;" /> 

理论计算结果：rou = Pl^3/(3EI) = 3.09mm  其中 I = bd^3/12,b宽为2.5mm，高5mm

仿真结果：输入弹性模量单位Pa，力N，模型尺寸mm，最终输出的变形位移为1E-6mm.



**单元类型：C3D8R** 

稀疏网格：**最大形变量158.6mm，结果与理论值完全不同** 

低密度网格：**最大形变量4.088mm，结果接近理论值** 

中密度网格：**最大形变量3.215mm，结果符合理论值** 

超密集网格划分：**最大形变量3.096mm，结果很好的符合理论值** 

<img src="Abaqus.assets\稀疏.png" style="zoom: 15%;" />  <img src="Abaqus.assets\低密度.png" style="zoom: 15%;" /> <img src="Abaqus.assets\中密集.png" style="zoom: 15%;" /> <img src="Abaqus.assets\超密集划分网格.png" style="zoom:15%;" /> 

稀疏网格 | 低密度 |  中密度网格 |  超密集



**单元类型：C3D20R** 

无法运行!



**单元类型：C3D20** 

**最大形变量3.086mm，结果很好的符合理论值** 

<img src="Abaqus.assets\稀疏-C3D8R.png" style="zoom:15%;" /> 

**单元类型：三角形** 

<img src="Abaqus.assets\三角形.png" style="zoom:15%;" /> 





### 凹槽成型 [[1](Abaqus.assets\基于abaqus的有限元分析和应用-庄茁.pdf)] 

基本步骤如下[1 12接触-P297] ：

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

<img src="Abaqus.assets\interaction1.png" alt="image-20191223223414844" style="zoom: 67%;" />

10. 定义个分析步骤的边界条件
11. 划分网格





[1] 基于abaqus 的有限元分析和应用  庄茁

