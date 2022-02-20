function my_fft( y , Fs )     %y为离散数据向量，以时间为横坐标的数据，fs为采样频率
% % fs = 10000;                 %采样频率
% % 
% % t= [0:0.0001:0.2];
% % y= sin(50*(2*pi)*t);
% close all
% Y=fft(y);
% n =length(y);
% f = [0 : n/2]*fs/n;            %采样频率必定大于实际频率的二倍
% pow = abs(Y(1:n/2+1));
% plot(f,pow,'-',f,pow,'o');
% %  axis([0,300,0,1000]);

% NFFT = 2^nextpow2(length(y));       % 频率图的点数
NFFT = length(y);

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