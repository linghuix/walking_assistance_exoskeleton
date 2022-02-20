function [ztran,F] = ss2z(f ,Ts)
% [ztran,F] = ss2z(f ,Ts)
% 将离散的脉冲响应函数转化为包含拉普拉斯算子s的传递函数。
% 输入
%     Ts表示采样的时间间隔，f为拉普拉斯传递函数
% 输出
%     ztran：转化后的离散脉冲响应函数
%     F： 转化的中间产物，系统的时域函数
% 
%   See also z2ss
%   Copyright 2017 POEWR-LJR.
% 
syms n z
f = ilaplace(f);
F = simplify(f);
f = subs(F,n*Ts);
ztran = ztrans(f,n,z);
end

