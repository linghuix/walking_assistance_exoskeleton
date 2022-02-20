% 离散傅里叶变换：输入 N_t采样点数，X_t采样值，Fs 采样信号
%                    N
%      X(k) =       sum  x(n)*exp(-j*2*pi*(k-1)*(n-1)/N), 1 <= k <= N.
%                   n=1
function freq = my_DFT(X_t)
    N_t = length(X_t);
    % matlab的数组最小值从1开始，所以需要变化一下。
    W = exp(-1j*2*pi/N_t);
    X = zeros(1,N_t);
    for k = 1:N_t
        for n = 1:N_t
            X(k) = X(k) + X_t(n)*W^((k-1)*(n-1));
        end
    end
    freq = X;
end
% Fs=1000;
% f1=50;
% f2=125;
% f3=135;
% n=0:1/Fs:1;
% xn=cos(2*pi*f1*n)+1.5*cos(2*pi*f2*n)+cos(2*pi*f3*n)+1.5*randn(size(n));
% 
% N=1024;Nsec=512;							%数据的长度和FFT所用的数据长度
% Pxx1=fft(xn(1:512),Nsec); 
% Pxx2=my_DFT(Nsec,xn(1:512)); 
% yn=cos(2*pi*f1*n)