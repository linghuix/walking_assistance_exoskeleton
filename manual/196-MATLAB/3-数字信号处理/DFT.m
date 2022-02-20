% 离散傅里叶变换：输入 N_t采样点数，X_t采样值，Fs 采样信号
function freq = DFT(N_t,X_t)

% matlab的数组最小值从1开始，所以需要变化一下。
W = exp(-j*2*pi/N_t);
X = zeros(1,N_t-1);
for k = 1:N_t
	for n = 1:N_t
		X(k) = X(k) + X_t(n)*W^(k*(n-1));
	end
end


% 离散傅里叶逆变换：输入 N_t采样点数，X_t采样值，Fs 采样信号
function X_t = DFT(N_t,freq)

% matlab的数组最小值从1开始，所以需要变化一下。
W = exp(-j*2*pi/N_t);
X = zeros(1,N_t-1);
for k = 1:N_t
	for n = 1:N_t
		X(k) = X(k) + X_t(n)*W^(-(k-1)*(n-1));
	end
	
	X(k) = X(k)/N_t;
end



% 自相关函数：输入 N_t采样点数，X_t采样值，Fs 采样信号
function freq = relax_self(N_t,X_t)

% matlab的数组最小值从1开始，所以需要变化一下。
Rxx = zeros(1,N_t);
for r = 1:N_t
	for n = 1:N_t-r+1
		Rxx(r) = Rxx(r) + X_t(n)*X_t(n+r-1);
	end
	
	Rxx(r) = Rxx(r)/(N_t-r);
end
	

	
% 互相关函数：输入 N_t采样点数，X_t采样值，Fs 采样信号
function freq = relax_self(N_t,X_t,Y_t)

% matlab的数组最小值从1开始，所以需要变化一下。
Rxy = zeros(1,N_t);
for r = 1:N_t
	for n = 1:N_t-r
		Rxy(r) = Rxy(r) + X_t(n)*Y_t(n+r);
	end
	
	Rxy(r) = Rxy(r)/(N_t-r);
end

