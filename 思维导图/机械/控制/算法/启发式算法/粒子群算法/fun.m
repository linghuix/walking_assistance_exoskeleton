%适应度函数代码如下:
function y = fun(x)
%该函数计算粒子适应度值 
%x	input	输入粒子位置
% y	output	粒子适应度值
% 最优值位于 （0,0）

y = sqrt(x(1).^2+x(2).^2)./sqrt(x(1).^2+x(2).^2) + exp((cos(2*pi*x(1))+cos(2*pi*x(2)))/2) -2.71289 ;