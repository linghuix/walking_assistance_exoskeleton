function [f ,A_f] = my_fft(y,Fs,L)    
% Plot single-sided amplitude spectrum.
% A_f为输出频率幅值，f为对应的频率

%只适用于相同采样频率的信号
%y为离散数据向量，以时间为横坐标的数据，fs为采样频率Sampling frequency
% L Length of signal时域上的信号长度，信号个数，采样点数

% L是采样点数，与截取时间长度有关，仅影响频率分辨力。
% 采样频率Fs与频域上的区间长度[-Fs/2 ,Fs/2]有关，需要包括感兴趣的频段。

T = 1/Fs;                       % Sample time                   
t = (0:L-1)*T;                  % Time vector
NFFT = 2^nextpow2(L);           %取不小于L的最小的2的幂。之所以这样取值是因为FFT算法要求信号的长度为2的幂，当NFFT大于信号长度时，fft函数以零补齐。  
Y = fft(y,NFFT)/L;              % fft函数直接计算得到的X（k）并不是频谱幅值。
f = (Fs/2)*linspace(0,1,NFFT/2+1);%采样点数决定了频率分辨力
A_f = 2*abs(Y(1:NFFT/2+1));

% Plot single-sided amplitude spectrum.
%plot(f,2*abs(Y(1:NFFT/2+1)));                       
%title('Single-Sided Amplitude Spectrum of y(t)')
%xlabel('Frequency (Hz)')
%ylabel('|Y(f)|')




%{
 Sum of a 50 Hz sinusoid and a 120 Hz sinusoid
x = 0.7*sin(2*pi*50*t) + sin(2*pi*120*t); 
y = x + 2*randn(size(t));     % Sinusoids plus noise
subplot(2,1,1)
plot(Fs*t(1:50),x(1:50))
title('Sinusoids Signal')
xlabel('time (milliseconds)')
subplot(2,1,2)
plot(Fs*t(1:50),y(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('time (milliseconds)')

NFFT = 2^nextpow2(L); %取不小于L的最小的2的幂。之所以这样取值是因为FFT算法要求信号的长度为2的幂，当NFFT大于信号长度时，fft函数以零补齐。         
Y = fft(y,NFFT)/L;    %                                            (1)
f = Fs/2*linspace(0,1,NFFT/2+1);
% Plot single-sided amplitude spectrum.
figure
plot(f,2*abs(Y(1:NFFT/2+1)))       %                               (2)
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
%}