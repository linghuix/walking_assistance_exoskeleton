function my_fft( y , fs )     %y为离散数据向量，以时间为横坐标的数据，fs为采样频率
% fs = 10000;                 %采样频率
% 
% t= [0:0.0001:0.2];
% y= sin(50*(2*pi)*t);
% close all
Y=fft(y);
n =length(y);
f = [0 : n/2]*fs/n;            %采样频率必定大于实际频率的二倍
pow = abs(Y(1:n/2+1));
plot(f,pow,'-',f,pow,'o');
%  axis([0,300,0,1000]);