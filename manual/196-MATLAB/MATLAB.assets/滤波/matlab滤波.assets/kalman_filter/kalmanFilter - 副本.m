%{ 
设该系统的状态方程和观测方程为：
x(k) = A · x(k-1) + B · u(k) + w(k)
z(k) = H · x(k) + y(k)
x(k) —— k时刻系统的状态
u(k) —— 控制量
w(k) —— 符合高斯分布的过程噪声，其协方差在下文中为Q
z(k) —— k时刻系统的观测值
y(k) —— 符合高斯分布的测量噪声，其协方差在下文中为R
A、B、H —— 系统参数，多输入多输出时为矩阵，单输入单输出时就是几个常数
注意在后面滤波器的方程中我们将不会再直接面对两个噪声w(k)和y(k)，而是用到他们的协方差Q和R。至此，A、B、H、Q、R这几个参数都由被观测的系统本身和测量过程中的噪声确定了。
%}


%{
卡尔曼操作：
“系统状态”(x)和“误差协方差”(P)。卡尔曼增益K(k)
x(k|k-1) = A · x(k-1|k-1) + B · u(k)
P(k|k-1) = A · P(k-1|k-1) · AT + Q
K(k) = P(k|k-1)HT(HP(k|k-1)HT + R)-1
x(k|k) = x(k|k-1)+K(k) (z(k) - H x(k|k-1))
P(k|k) = (I - K(k)H) P(k|k-1)
%}


%{
假设我们是在测温度、加速度或者记录蓝牙RSSI，那么此时控制量是没有的控制量是没有的, B·u(k) ≡ 0参数 A和 H也简单地取1。
x和P只需要赋初值，每次迭代会产生新值；K用不着赋初值；Q和R赋值以后在之后的迭代中也可以改
x和P的初值是可以随便设的，一般设为需要滤波的序列的初值
%}


%{
while(新观测值)
{
    K = P / (P + R);
    x = x + K * (新观测值 - x);
    P = (1 - K) · P + Q;
}
	测试白噪声代码
	data =randn(1,100); 
	result=kalmanFilter(data,1e-6,4e-4,-60,1); 
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


