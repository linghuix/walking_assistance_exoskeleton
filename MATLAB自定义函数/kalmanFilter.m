%参数 A和 H也简单地取1。
%控制量是没有的,B·u(k) ≡ 0
%x和P只需要赋初值，每次迭代会产生新值；K用不着赋初值；Q和R赋值以后在之后的迭代中也可以改
%x和P的初值是可以随便设的

%{
while(新观测值)
{
    K = P / (P + R);
    x = x + K * (新观测值 - x);
    P = (1 - K) · P + Q;
}

	测试白噪声代码
	data =random('Normal',0,5,1,100); kalmanFilter(data,1e-6,4e-4,-60,1); 
	i = 1:length(data); 
	plot(i,result,'r',i,data,'b');
 
 
%}


% kalman_filter.m 
function X = kalmanFilter(data,Q,R,x0,P0) 
% data数据，Q过程噪声的协方差，R测量噪声，x0初值，P0误差协方差。
%

N = length(data); 
K = zeros(N,1); 
X = zeros(N,1); 
P = zeros(N,1); 
X(1) = x0; 
P(1) = P0; 
for i = 2:N 
	K(i) = P(i-1) / (P(i-1) + R); 
	X(i) = X(i-1) + K(i) * (data(i) - X(i-1)); 
	P(i) = P(i-1) - K(i) * P(i-1) + Q; 
end


