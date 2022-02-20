function [k,s]=flds(G)
G=tf(G);
num=G.num{1};
den=G.den{1};
dt =get(G,'inputdelay');%获取滞后项exp()
syms ns ds x
a=conv(den,polyder(num));% polyder(num)%求导derivative
%原理
%根轨迹方程k*num/den*exp(-dt*s) = -1
%k=-den/num*exp(dt*s)
%d(k)=0
% d(-den/num)*exp(dt*s)-den/num*(dt)*exp(dt*s)
b=conv(num,polyder(den));
a1=([zeros(1,length(b)-length(a)),a]-b);
a2=dt*conv(num,den);
ss=roots([zeros(1,length(a2)-length(a1)),a1]-a2);
k=[];
s=[];
for i=1:length(ss)
ns = poly2sym(num,x);
ds = poly2sym(den,x);
K(i)=subs(-ds/ns,'x',ss(i));
if(K(i)>0&&imag(ss(i))==0)
    k=[k K(i)];
    s=[s,ss(i)];
else
    S='此系统没有分离点！';
   
end
k=vpa(k,6);
end