
%% 时域信号
Fs=1000;                        %采用频率
f1=50;f2=125;f3=135;
N=5012;Nfft=N;n=0:N-1;t=n/Fs;   %采用的时间序列
xn=cos(2*pi*f1*t)+1.5*cos(2*pi*f2*t)+cos(2*pi*f3*t)+1.5*randn(size(n));
figure;
plot(t,xn);grid on;title('时域信号 t——y ');
%% 
% 该函数计算的是功率谱归一化后的结果，计算公式中还除了Fs，量纲为dB/Hz。
% 比普通的周期图法大大约3dB
window= boxcar(length(xn));                     % 窗函数，选择所有的采样点数
[pxx,f]=periodogram(xn,window,length(xn),Fs);   % 功率谱归一化函数，输入采样长度和采样值，采样频率，截取长度
plot(f,10*log10(pxx*Fs),'r') 
ylabel('功率谱密度，dB/Hz')
plot(f,pxx,'r') 