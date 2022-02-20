%% 采样
Fs = 128;      %% 采样频率，单位Hz
L = 1;       %% 采样时间，单位s
t = 0:1/Fs:L;
n = Fs*t+1;  %% 采样序号，采样点需要加一，n=length(y)
y = sin(10*pi*t)+3;
ezplot('sin(10*pi*t)',[0 L]);
hold on
stem(t,y,'.r');


%% fft变换
% NFFT = 2^nextpow2(length(n)) 
% % 取不小于L的最小的2的幂。之所以这样取值是因为FFT算法要求信号的长度为2的幂，当NFFT大于信号长度时，fft函数以零补齐。
% % 可以直接看这个
A = abs(fft(y));    %% 频域幅值序号
stem(n,A);          %% 频域幅值序号从1开始

figure
f = Fs/2*linspace(0, 1, length(n)/2+1);%%采样点数决定了频率分辨力
A_f = 2*A(1:length(n)/2+1);
stem(f,A_f,'.r');

%% Ifft变换
figure
[yy] = ifft(A); %% 时域幅值序号
ezplot('sin(10*pi*t)',[0 L]);
hold on
plot(t,y,'--r',t,yy(1:length(t)),'g');

