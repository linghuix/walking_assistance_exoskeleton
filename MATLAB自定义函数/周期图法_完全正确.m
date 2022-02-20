%% 时域信号
Fs=1000;                        %采用频率
f1=50;f2=125;f3=135;
N=1024;Nfft=N;n=0:N-1;t=n/Fs;   %采用的时间序列
xn=cos(2*pi*f1*t)+1.5*cos(2*pi*f2*t)+cos(2*pi*f3*t)+1.5*randn(size(n));
figure;
plot(t,xn);grid on;title('时域信号 t——y ');
%% 
Pxx=(abs(fft(xn,Nfft))).^2/N;	%Fourier振幅谱平方的平均值，未转化为dB
                                % f=(0:length(Pxx)-1)*Fs/length(Pxx);
t = 0:round(Nfft/2-1);          %给出从0开始的序列
f = t*Fs/Nfft;                  %给出频率序列
figure;
plot(f',10*log10(Pxx(t+1)));	%绘制功率谱曲线，电流电压对数使用20lg，功率使用10lg
xlabel('频率/Hz');ylabel('单边功率');
title('周期N=128');grid on;
std(Pxx)^2                      % 求解标准偏差
%%
hold on 
[pxx,f] =  psd(xn,N,Fs,boxcar(N),0,'none') % 系统自带周期图法
plot(f,10*log10(pxx),'--y') 
ylabel('功率谱密度，dB')
pxx(1:512)'-10*log10(Pxx(t+1))