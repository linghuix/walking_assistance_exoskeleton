%平均周期图法求功率谱密度函数
% 使用重叠0% 的方式计算平均周期法
clear;
Fs=1024;
f1=50;
f2=125;
f3=135;
n=0:1/Fs:1;
xn=cos(2*pi*f1*n)+1.5*cos(2*pi*f2*n)+cos(2*pi*f3*n)+1.5*randn(size(n));

N=1024;Nsec=256;                                    %数据的长度和FFT所用的数据长度
Pxx1=abs(fft(xn(1:Nsec),Nsec)).^2/Nsec;             %第一段功率谱
Pxx2=abs(fft(xn((Nsec+1):Nsec*2),Nsec)).^2/Nsec;      %第二段功率谱
Pxx3=abs(fft(xn((Nsec*2+1):Nsec*3),Nsec)).^2/Nsec;	%第三段功率谱
Pxx4=abs(fft(xn((Nsec*3+1):Nsec*4),Nsec)).^2/Nsec;	%第四段功率谱
Pxx=(Pxx1+Pxx2+Pxx3+Pxx4)/4;						%Fourier振幅谱平方的平均值，并转化为dB
f=(0:length(Pxx)-1)*Fs/length(Pxx);                 %给出频率序列
hold on
% plot(f(1:Nsec/2),10*log10(Pxx(1:Nsec/2)));		%绘制功率谱曲线
plot(f(1:Nsec/2),Pxx(1:Nsec/2));
xlabel('频率/Hz');ylabel('单边功率谱');
title('N = 4X256 分辨力 Fs/256=4Hz');grid on;
(std(Pxx))^2
%% 使用重叠50% 的方式计算平均周期法
N=1024;Nsec=256;                                    %数据的长度和FFT所用的数据长度
Pxx1=abs(fft(xn(1:2*Nsec),2*Nsec)).^2/Nsec; 		%第一段功率谱
Pxx2=abs(fft(xn((Nsec+1):Nsec*3),2*Nsec)).^2/Nsec;	%第二段功率谱
Pxx3=abs(fft(xn((Nsec*2+1):Nsec*4),2*Nsec)).^2/Nsec;	%第三段功率谱
Pxx=(Pxx1+Pxx2+Pxx3)/3;                             %Fourier振幅谱平方的平均值，并转化为dB
f=(0:length(Pxx)-1)*Fs/length(Pxx);                 %给出频率序列
hold on
% plot(f(1:Nsec/2),10*log10(Pxx(1:Nsec/2)));		%绘制功率谱曲线
plot(f(1:Nsec),Pxx(1:Nsec));
xlabel('频率/Hz');ylabel('单边功率谱');
title('N = 3X512 分辨力 Fs/512=2Hz');grid on;
(std(Pxx))^2
