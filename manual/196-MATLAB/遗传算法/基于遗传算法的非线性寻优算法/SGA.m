%%遗传算法  一元函数的最优解
% 
% * 用遗传算法求简单一元函数  sin(10*pi*x)/x ,[1,2] 的最小值

clc
clear all
close all
%%
 
% * 画出函数图
figure(1);
hold on
ib=1;ub=2;   %函数自变量范围[1,2]
ezplot('sin(10*pi*x)/x',[ib,ub]);%画出函数曲线
xlabel('自变量/X')
ylabel('函数值/Y')

%%
%定义遗传算法参数
nind=40;%种群大小
maxgen=20;%最大遗传代数
preci=20;%个体长度--与最后得到的最优值的精度有关
ggap=0.95;%代沟--指出代沟部分种群被复制
px=0.7;%交叉概率
pm=0.01;% 变异概率
trace=zeros(2,maxgen);%用于储存每一代的最优个体
fieldd=[preci;ib;ub;1;0;1;1];%函数区域描述
chrom=crtbp(nind,preci);%创建任意离散随机种群
%%
%%优化
gen=0;% 种群遗传代数
x=bs2rv(chrom,fieldd);%初始种群二进制转化为十进制
objv=sin(10*pi*x)./x;%计算目标函数值
while gen<maxgen
    fitv=ranking(objv);%分配适应度值
    selch=select('sus',chrom,fitv,ggap);%选择
    selch=recombin('xovsp',selch,px);%重组
    selch=mut(selch,pm);%变异
    x=bs2rv(selch,fieldd);%子代个体的十进制转换
    objvsel=sin(10*pi*x)./x;%子代个体的目标函数值
    [chrom,objv]=reins(chrom,selch,1,1,objv,objvsel);%将子代个体插入到父代，得到新的种群
    x=bs2rv(chrom,fieldd);
    gen=gen+1;
    [y,i]=min(objv);%获取每一代的最优解及其序号，y为最优解，i为个体的序号
    trace(1,gen)=x(i);%记下每一代的最优自变量值
    trace(2,gen)=y;%记下每一代的最优目标函数值
end
plot(trace(1,:),trace(2,:),'ro');%画出每一代的最优解
grid on;
plot(x,objv,'b*');%画出最后一代的种群分布
hold off
%%
% *画进化图、
figure(2)
plot(1:maxgen,trace(2,:));
grid on;
xlabel('遗传代数')
ylabel('解的变化')
title('进化过程')
besty=trace(2,end);
bestx=trace(1,end);
fprintf(['最优解：\n x=',num2str(bestx),'\n y=',num2str(besty),'\n'])





