ezplot('sin(5*t)',[0 2])
% 离散化
hold on
Fs = 10/pi;
tn = 0:1/Fs:2
yn=sin(5*tn);
plot(tn,yn,'o')
%% 真实的离散序列
figure
Fs = 10;
n = 0:1:2*Fs
wn = 5/Fs;
yn=sin(wn*n);
plot(n,yn,'o')
