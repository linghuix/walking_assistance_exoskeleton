% 信号观测
data= SignalBuilder('G',50,5,20);
plot(data,'*')
hold on
plot( 1:length(data),5*ones(1,length(data)),'black' )


% 卡尔曼滤波
% 已知
% 系统输入为常数，系统内部噪声方差为0

% 输入 1*N 的常数值 5 
% 传感器观测到的数据data
% 系统噪声方差为0
% 传感器测量噪声方差为20
% 系统状态初始值为30
% 系统内部误差协方差初值 20
[data_filiter,P] = kalmanFilter( 5*ones(1,length(data)), data, 0, 0.00001, 30, 20 );

plot(data_filiter,'gp--');
plot(P,'r-');
legend('sensor output','ground value','filiter','P');
