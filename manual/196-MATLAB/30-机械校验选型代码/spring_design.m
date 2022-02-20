function spring_design(F1,F2,h,d,rou_b_C)
%{

% 本函数是关于圆柱螺旋压缩弹簧和拉伸弹簧设计的

参考书籍——机械设计，主编陈秀宁，顾大强，第十六章

输入参数——最小工作载荷F1 N，最大工作载荷F2 N，弹簧工作行程h mm，

输入参数——根据表格确定弹簧丝直径 d，获取抗拉强度rou_b_C，有一定的随意性，需要进行后续的校核

%}

%% 计算弹簧尺寸

% F1 = 20  % 最小工作载荷N ***
% F2 = 100 % 最大工作载荷N ***
% h  = 40  % 弹簧工作行程mm ***

F1 = 0         %N
F2 = 100        %N
% k = 35          %N/mm
h = 30  %mm

% 1.确定材料碳素弹簧B、C、D级，抗拉强度依次增大。暂定第二类，重复次数要求1万次
% 2.确定使用65Mn材料，重要用途
% tao 许用切应力，rou_b_C C级碳素抗拉强度，G 剪切模量，d 弹簧丝直径
material = 'BCD'
class = 1               %弹簧类型  1代表第一类，重复次数10^6,   2代表10^3~10^6,  3代表10^3以下

if strcmp(material,'BCD')||strcmp(material,'65Mn')
    rou = [0.3 0.4 0.5];    %不同类别对应的正应力
    tao = rou(class)*rou_b_C
    G = 81500*(d <= 4)+80000*(d>4)
elseif strcmp(material,'60Si2Mn')||strcmp(material,'60Si2MnA')
    rou = [480 640 800];    %不同类别对应的正应力
    tao = rou(class)
    G = 80000
end


% 2.确定弹簧丝直径——tao和G确定
d = 1.6           % mm ***
rou_b_C = 2110  % ***

% 3.弹簧尺寸标准，确定D2, 旋绕比C
D2 =  10;  %空间限制1.5cm直径以内   ***
C = D2/d;
K = (4*C-1)/(4*C-4)+0.615/C;% 曲率系数
if(d < 1.6*sqrt(K*F2*C/tao))
    fprintf('预设的d无法满足强度要求,d = %d,强度要求 = %d \n',d,1.6*sqrt(K*F2*C/tao));
end

%%
% % 4.弹簧刚度
% k = (F2-F1)/h/1000

% 5.有效工作圈数 n
n = G*h*d/(8*(F2-F1)*C^3);
fprintf('计算得到有效工作圈数n = %.3f \n', n);
n = input('请按照标准规格输入有效工作圈数')   %13.5


% 5.重新计算最小载荷F1（最大载荷要求不变，行程不变），和有效工作圈数 n 
lamda2 = 8*F2*C^3*n/G/d;% lamda2 为最大载荷下的形变位移
lamda1 = lamda2 - h;    % lamda1 为最小载荷下的形变位移
F1 = F2*lamda1/lamda2;  % F2/lamda2 = F1/lamda1 = k
n = n+2;                % 实际圈数
fprintf('实际圈数  %d \n', n);

% 6.确定弹簧尺寸
fprintf('最大载荷下的弹簧圈之间的间隙 delta要求大于 %f \n', 0.1*d);
delta = input('请按照标准规格输入弹簧圈之间的间隙'); 

t = lamda2/n+d+delta;                                                              % 自由状态下的弹簧圈之间的节距
if(t < 0.3*D2 || t > 0.5*D2)
    fprintf('节距t 不符合要求,t = %f ,0.3*D2 = %f ,0.5*D2 = %f\n ，请增大弹簧圈之间的间隙',t,0.3*D2,0.5*D2);
end

H0 = n*t-1.5*d;
fprintf('自由高度H0计算得 %f \n', H0);
H0 = input('请按照标准规格输入自由高度H0');  %45

alpha = atan(t/D2/pi)/pi*180
if(alpha > 9 )
    fprintf('自由状态下的螺旋升角不符合要求,alpha = %f\n',alpha);
end

b = H0/D2
if(b > 2.6)
    fprintf('稳定性不符合要求,b = %f\n， 请增大直径D2或者减小自由高度H0',b);
end

L= pi*D2*(n-2)/cos(alpha/180*pi)
H2 = H0-lamda2
H1 = H0-lamda1
% F1_design = 
% F2_design = 
