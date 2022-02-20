clc
clear
% net = feedforwardnet(hiddensizes,trainfcn)
    [x,t] = simplefit_dataset;
    n=1:5:90;
    net = feedforwardnet([5 2]);
    net.layers{1}.transferFcn = 'tansig';
    net.layers{2}.transferFcn = 'tansig';
    [net,tr] = train(net,x(n),t(n));
    view(net);

%% 直接利用net函数计算输出
    y = net(x);
    % perf = perform(net,y,t);
    % plottrainstate(tr);

%% 提取权重矩阵进行计算
    LW=net.LW; IW=net.IW; b=net.b;
    % 归一化操作！！
    [x2,s]= mapminmax(x(n));
    y4 = [];
    for k = 1:size(x2,2)
        y1 = tansig(x2(k).*IW{1,1}+b{1});
        y2 = tansig(LW{2,1}*y1+b{2});
        y4 = [y4 y2];
    end
    % 反归一化操作！！
    y5 =mapminmax('reverse',y4,s);
    plot(x,y,'ob')
    hold on
    plot(x(n),y5,'or')
    plot(x,t,'g')
