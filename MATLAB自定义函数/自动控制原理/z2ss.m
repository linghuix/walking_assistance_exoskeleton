function [F,ft] = z2ss(f)
% [F,ft] = z2ss(f)
% 将包含拉普拉斯算子s的传递函数，转化为离散的脉冲响应函数
% 输入
%     f：离散的脉冲响应函数
% 输出
%     F：转化后的包含拉普拉斯算子s的传递函数
%     ft： 转化的中间产物，系统的时域函数
% 使用时要注意将离散的脉冲响应函数的采样周期全部化为1,
%   See also ss2z
%   Copyright 2017 POEWR-LJR.
% 
syms n z t
% subs(f,T,1);
ft = iztrans(f,t);
ft = simplify(ft);
% f = subs(F,n*Ts);
F = laplace(ft);
end

