
%t = 0:0.00001:0.1;
%f = sin(2*pi*60*t)+cos(2*pi*25*t)+cos(2*pi*30*t);
%plot(t,f,'r');
fy = 'sin(2*pi*60*t) + cos(2*pi*25*t) +cos(2*pi*30*t)';
% 第一个输入变量函数是原信号函数，信号函数fy以字符串的格式输入
fs0 = 10000;
tp  = 0.1;
t   = -tp:1/fs0:tp;  %001个样本点
k1  = 0:999;
k2  = -999:-1; 
m1  = length(k1);
m2  = length(k2);
f   = [fs0*k2/m2,fs0*k1/m1];%f 1999
w   = [-2*pi*k2/m2,2*pi*k1/m1];%w = 2*pi*K/N,这里少算了头尾两点
fx1 = sin(2*pi*60*t)+cos(2*pi*25*t)+cos(2*pi*30*t)
FX1 = fx1*exp(-1i*((1:length(fx1))./10)'*w); %除10好像得到了补0的效果
%求原信号的离散时间傅里叶变换
figure(1);
%h画出原信号波形
subplot(1,2,1);plot(t,fx1,'r');
title('原信号');
xlabel('时间t/s');
axis([min(t),max(t),min(fx1),max(fx1)]);       %画原信号幅度频谱
subplot(1,2,2);
plot(f,abs(FX1),'r');
title('原信号幅度频谱');
xlabel('频率f');
axis([-1000,1000,0,max(abs(FX1)+5)]);          %画原信号幅度频谱

    %其实把上面代码改一改就可以用到求采样信号的了。这里的卷积我试过很多种，以下的是我目前试出来唯一一个可以用的。

% 对信号进行采样
fs   = 150;   % 采样频率
Ts   = 1/fs;
ws   = Ts*w;
ts   = Ts*(1:length(t));
fst  = sin(2*pi*60*ts)+cos(2*pi*25*ts)+cos(2*pi*30*ts);
Fs1 = fst*exp(-1i*((1:length(fst))./10)'*ws);
length(fst)
figure(2);
subplot(1,2,1);plot(ts,fst,'r');
title('采样信号');
xlabel('时间t/s');
axis([min(ts),0.5,min(fst),max(fst)]);       %画原信号幅度频谱
subplot(1,2,2);
plot(f,abs(Fs1),'r');
title('幅度频谱');
xlabel('频率f');

t1 = 1:length(t);
%这里的冲击信号这么大是为了和采样信号的相匹配，最后结果因为用的是卷积所以这样
%求h(t)
ht = Ts*sin(2*pi*t1/Ts)./(t1*pi);
Ft = conv(ht,fst);
n = 1: max([length(fst)+length(ht)-1,length(fst),length(ht)]);
figure(3);
plot(n,Ft,'b');
axis([200,450,min(Ft),max(Ft)]);

    %当然自己求卷积的话还有很多方法。一下就介绍一种方法：
% 
% y = -0.1:0.001:0.1;
% n = -0.1/T:0.1/T;
% M=ones(length(n),1)*y-n'*T*ones(1,length(y));
% fs = fg*sinc(frequency*M);