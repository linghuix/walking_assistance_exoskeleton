% 调用filter解差分方程 y(n)-ay(n-1)=x(n)
a = 0.5; 
ys = 0;%初始条件y(-1)=1
xn = [1,1, zeros(1,30)];
B = 1;A=[1 -a];
xi = filtic(B,A,ys);% 初始条件对应的等效输入序列x(0)
yn = filter(B,A,xn,xi);

n = 0:length(yn)-1;
stem(n, yn,'.');
title('(a)');
xlabel('n');ylabel('y(n)')