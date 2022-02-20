ts=0.001
t = 0:0.01:100;
y2=100*sin(2*t);
yin=100*sin(2*t+pi/6)+100*sin(20*t)+100*sin(0.1*t);

plot(y2.*yin)

a = relationship(yin,y2);%a(1) 得到的就是 A0A1/2*cos(wt-fai)，令w=0就是相位-fai 
plot(a/1e2*2)
hold on
plot(y2)
% 计算要求，知道对应输入输出的幅值大小

plot(yin)
plot(a.*y2/5000/100*2)
fxy = fft(a);
b = relationship(yin,yin);
fx = fft(b);



f = fxy./fx
n_f = (1/ts/2)*linspace(0,1,length(t)/2+1)
plot(n_f,abs(f(1:length(n_f))))
title('abs')
figure
plot(n_f,angle(f(1:length(n_f))))