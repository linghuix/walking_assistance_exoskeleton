%% kalman_filter.m 

function [X,P] = kalmanFilter(u,y,N,W,x0,P0) 

% u - 输入参数
% y - 传感器观测到的数据
% N - 系统噪声方差
% W - 传感器测量噪声方差
% x0 - 系统状态初始值
% P0 - 系统内部误差协方差初值


% 前提假设
% 输出传感器包含高斯噪声W
F = 1;C = 1;H = 0;

NN = length(u); 
K = zeros(1,NN); X = zeros(1,NN) ;P = zeros(1,NN); 

for i = 1:NN 
    % 预测
    if(i==1)
        X(i) = F*x0 + H*u(i);
        P(i) = F*P0*F' + N;
        continue
    else 
        X(i) = F*X(i-1) + H*u(i);
        P(i) = F*P(i-1)*F' + N;
    end
    
    % K增益的计算
    K(i) = P(i)*C'*(C*P(i)*C'+W)^-1;
    
    % 校正
    X(i) = X(i) + K(i)*(y(i) - C*X(i));
    P(i) = (1-K(i)*C)*P(i);
end
