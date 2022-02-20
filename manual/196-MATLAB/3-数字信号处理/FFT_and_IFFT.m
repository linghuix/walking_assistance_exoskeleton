%% 采样,绘制原信号和采样点值
Fs = 1024;                  % 采样频率，单位Hz
% L = 7.9921875;            % 输入采样时间，单位s
N = 2048                    % 输入采样点数目
L = (N-1)/Fs                % 时间段长度

t = 0:1/Fs:L;
% n = Fs*t+1;               % 采样序号，采样点需要加一，n=length(y)
y = 9*sin(10*pi*t)+50+20*sin(20*pi*t);
% ezplot('sin(10*pi*t)',[0 L]);
hold on
plot(t,y,'.-');         

%% fft变换
% % 算法，将双边改为单边，幅值要*2
% % 将序列号，转为为实际的采样频率点，采样间隔由采样时间长度决定，最大能够采样频率由采样时间间隔决定
% % 取不小于L的最小的2的幂。之所以这样取值是因为FFT算法要求信号的长度为2的幂，当NFFT大于信号长度时，fft函数会以零补齐

NFFT = 2^nextpow2(length(t));       % 频率图的点数

A = fft(y,NFFT);
A_abs = abs(A);                     % 频域幅值
stem(1:NFFT,A_abs);                 % 频域幅值序号从1开始

figure(2)

subplot 211
f = Fs*linspace(0, 1, NFFT);        %采样点数决定了频率分辨力
stem(f,A_abs,'.r');
title('FFT(x)直接计算结果的向量图')

subplot 212
f = Fs/2*linspace(0, 1, NFFT/2);    %采样点数决定了频率分辨力
A_f = [A_abs(1)  2*A_abs(2:NFFT/2)];
stem(f,A_f/NFFT,'.r');
title('真实频谱，幅值为该频率正弦波对应的幅值')
%% Ifft变换
% 注意，ifft如果直接输入A_abs得到的结果会超前一个单位，还会存在一定的误差

figure(1)
[yy] = ifft(A);                     % 时域幅值序号
plot(t,y,'-*',t,yy(1:length(t)),'r');
legend('original','ifft');

% 直接通过傅里叶变换修改均值
figure(2)
mean = 600;
B = A;
B(1) = mean*NFFT;                   % 修改均值到mean
[yy] = ifft(B);
plot(t,y,'--r',t,yy(1:length(t)),'g');



