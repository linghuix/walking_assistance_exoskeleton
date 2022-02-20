cd D:\matlab\神经网络算法代码\傅里叶变换系数生成函数
% y = [y;q(1)*cos(q(2).*x)+5*sin(q(3).*x)+0.6*log10(abs(q(4).*x))]输入q，来预测输出的曲线


%function,data sample 
x = 1:50;

% data set number
nn=1000;

% t = [1 0.2 0.3 2]
y = [];
t = [];
yout = [];
for i = 1:nn
    % get q = 1X4 ,para of function q(1)*cos(q(2).*x)+5*sin(q(3).*x)+0.6*log10(abs(q(4).*x))
    q = rand(1,4);
    % y is the point (x,y) on curve
    y = [y;q(1)*cos(q(2).*x)+5*sin(q(3).*x)+0.6*log10(abs(q(4).*x))];  
    % actual output fft coefficient
    yout = [yout;sine_coff(y(i,:)',10)'];
    % save data
    t = [t;q];
end
%%
% test whether the length of FFt coefficient is OK
% generate the curve from FFt coefficient 
i = round(rand(1,1)*nn)         % automatical select data
j = yout(i,:);                  % fft coefficient
le = length(y(i,:));

con = j(1);                     %constant
re = j(2:(length(j)+1)/2);      %cos(2*pi/le*i*x) i from 1 to (le-1)/2
im = j((length(j)+1)/2+1:end);  %sin(2*pi/le*i*x) i from 1 to (le-1)/2

q=[];n = (1:length(re));
gg=@(x)(sum(re.*cos(2*pi/le.*n*x)+im.*sin(2*pi/le.*n*x))+con)/le;
for k = (1:le)-1
    q = [q gg(k)];
end
% see in the picture
plot(y(i,:))
hold on;
plot(q,'r')
%%
%       network(numInputs,numLayers,biasConnect,inputConnect,layerConnect,
%       outputConnect) takes additional optional arguments and returns a neural
%       network with the following properties defined:
%      numInputs     - Number of inputs, 0.
%      numLayers     - Number of layers, 0.
%      biasConnect   - numLayers-by-1 Boolean vector, zeros.
%      inputConnect  - numLayers-by-numInputs Boolean matrix, zeros.
%      layerConnect  - numLayers-by-numLayers Boolean matrix, zeros.
%      outputConnect - 1-by-numLayers Boolean vector, zeros.

% choose train set
input_train=t(1:2:nn,:)'
output_train=yout(1:2:nn,:)'

% normalization 
[inputn,inputps]=mapminmax(input_train);
[outputn,outputps]=mapminmax(output_train);

% % 设置训练参数
% net.trainParam.epochs = 1000;
% net.trainParam.goal = 1e-3;
% net.trainParam.lr = 0.1;
% net = feedforwardnet([30 30]);
% % pick the even number for , train odd number for test
% [net,tr] = train(net,inputn,outputps);

% input layer = 20; hidden layer = 20
net=newff(inputn,outputn,[20,20]);

net.trainParam.epochs=100;
net.trainParam.lr=0.1;
net.trainParam.goal=0.00004;
view(net)
%网络训练
net=train(net,inputn,outputn);

% predict all the data 
N = 1:nn;
input_test = t(N,:)';
inputn_test=mapminmax('apply',input_test,inputps);  % input test = 4*nn
yout_yu = net(inputn_test);                         % 每一列是一个样本
BPoutput=mapminmax('reverse',yout_yu,outputps);
yout_yu = BPoutput';
%%
%网络输出
close all
% choose one prediction
i = N(round(rand(1,1)*nn)) 
j = yout_yu(i,:)
le = length(y(i,:));
con = j(1);
re = j(2:(length(j)+1)/2);
im = j((length(j)+1)/2+1:end);
q=[];n = (1:length(re));
gg=@(x)(sum(re.*cos(2*pi/le.*n*x)+im.*sin(2*pi/le.*n*x))+con)/le;
for k = (1:le)-1
    q = [q gg(k)];
end
plot(y(i,:),'b')
hold on;
plot(q,'g')

%实际傅里叶变化的输出
j = yout(i,:);
le = length(y(i,:));
con = j(1);
re = j(2:(length(j)+1)/2);
im = j((length(j)+1)/2+1:end);

q=[];n = (1:length(re));
gg=@(x)(sum(re.*cos(2*pi/le.*n*x)+im.*sin(2*pi/le.*n*x))+con)/le;
for k = (1:le)-1
    q = [q gg(k)];
end
hold on;
plot(q,'r')

legend('实际曲线','网络输出','傅里叶反变换')
%%
% input_test0 = rand(1,4)';
input_test0 = [10 5 2 2]';
T = input_test0;
input_test0 = mapminmax('apply',input_test0,inputps);
BPoutput0 = net(input_test0);
BPoutput0 = mapminmax('reverse',BPoutput0,outputps)'

%网络输出
le = 50;
close all
j = BPoutput0;
con = j(1);
re = j(2:(length(j)+1)/2);
im = j((length(j)+1)/2+1:end);
q=[];n = (1:length(re));
gg=@(x)(sum(re.*cos(2*pi/le*n*x)+im.*sin(2*pi/le*n*x))+con)/le;
for k = (1:le)-1
    q = [q gg(k)];
end
plot(q,'g')

x=1:50;
Y = T(1)*cos(T(2).*x)+5*sin(T(3).*x)+0.6*log10(abs(T(4).*x)) ;
hold on ; plot(Y,'b')