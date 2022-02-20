cd D:\matlab\神经网络算法代码
load data.mat
% net = feedforwardnet(hiddensizes,trainfcn)

% % 设置训练参数
net.trainParam.epochs = 1000;
net.trainParam.goal = 1e-3;
net.trainParam.lr = 0.01;

net = feedforwardnet([10 10]);
% net.layers{1}.transferFcn = 'tansig';
% net.layers{2}.transferFcn = 'tansig';
[net,tr] = train(net,data(1:2:51,:)',output(1:2:51)');%,每一列是一个样本。
% [net,tr] = train(net,data(1),data(2),output);
% view(net);

%% 直接利用net函数计算输出
y = net(data');
% perf = perform(net,y,data');
% plottrainstate(tr);
% %mean square error
% plotperform(tr)

%%提取权重矩阵进行计算
% LW=net.LW; IW=net.IW; b=net.b;
% % 归一化操作！！
% [x2,s]= mapminmax(x(n));
% y4 = [];
% for k = 1:size(x2,2)
% y1 = tansig(x2(k).*IW{1,1}+b{1});
% y2 = tansig(LW{2,1}*y1+b{2});
% y4 = [y4 y2];
% end
% % 反归一化操作！！
% y5 =mapminmax('reverse',y4,s);
plot(output,'-b')
hold on
% filery = kalmanFilter(y,1,5e1,y(1),0)
plot(1:2:51,y(1:2:51),'or')
plot(2:2:51,y(2:2:51),'og')

%% fft
con = fft(output);
plot(abs(con));
plot(output)
hold on;
con(7:46) = 0;
plot(ifft(con),'-r')
%%
% ezplot('589.26+(exp( - x*2.11i))+(100*exp(-3.7 + x*2.11i))')
i = [0:50];
syms x
% yy = @(x) 1/51*(sum(con(i+1)'.*exp(j*2*pi/51.*i.*x)));
yy = @(x) 1/51*(sum(con(i+1)'.*(cos(2*pi/51.*i.*x)+j*sin(2*pi/51.*i.*x))));
q = [];
for k=51:1
    q = [q yy(k)];
end
plot(1:51,real(q),'y')

%% 傅里叶级数展开
% a = [589.26;521.43 + 114.44i;-72 + 56.23i;-3.18 - 39.14i;4.53 + 5.99i;-3.7 - 2.107i];
a = con;
i=0:5;
gg = @(x) (sum((2*real(a(i+1))'.*cos(2*pi/51*i*x)-2*imag(a(i+1))'.*sin(2*pi/51*i*x)))-a(1))/51;
q=[];
for k = 1:51
    q = [q gg(k)];
end
plot(q,'y')
