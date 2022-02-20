
P1 = 3; % kW
n2 = 70;% r/min
T2 = 9.55e6*P1*0.8/n2;


disp('******************************************')
T = 84.84;%支撑泵的最大扭矩
yita = 0.8;%效率
Fs = 5*9.8;%人能持续提供的力
R = 0.05;%手柄的半径5cm
M = 56; %kg
g = 10;

%%确定传动比,
i = T/(Fs*R*yita);disp(sprintf('传动比i = %.2f',i)); %需要的传动比
i = round(i);disp(sprintf('校正传动比i = %.2f',i));
z1 = (i>=30)*1+(i<30)*2;disp(sprintf('蜗杆齿数 = %d',z1));%蜗杆的齿数
z2 = z1*i;disp(sprintf('涡轮齿数 = %d',z2));%涡轮的齿数
if(z2>80) 
    disp('涡轮齿数超过80，涡轮轮齿弯曲强度无法保证');
end
if(z2<28)
   disp('涡轮齿数少于28，传动不平稳');
end
%%确定模数等尺寸
rou_H = 250 ;%ZCuAl10Fe3 材料强度
K = 1.05; %载荷平稳
disp(sprintf('m^2d = %.2f',(480/z2/rou_H)^2*K*T*1e3));


m = 2;d1 = 35.5;disp(sprintf('模数m = %.2f,蜗杆直径%.1fmm',m,d1));%由m^2d确定，蜗杆的尺寸确定
d2 = m*z2;disp(sprintf('蜗轮直径d2 = %.2dmm',d2));
a = 0.5*(d1+d2);disp(sprintf('中心距a = %.2fmm',a))

% % % % % 铁板弯曲强度校核
Ft = Fs*R*1000/d1;
Fr = tan(20/180*pi)*2*T*1000/d2;disp(sprintf('蜗杆所受压力 = %.1f N',sqrt(Ft^2+Fr^2)))

L = 0.430; % 侧板长度
d = 0.010; % 侧板宽度
P = M*g/(L*d) + T/(L*(d)^3/12)*d/2;disp(sprintf('侧板压强 = %.2e Pa',P))

