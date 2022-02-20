syms d1 d2 d4 		% 板的厚度,夹具厚度，下底板的厚度,
d1 = 30e-3;
d4 = 30e-3;
% d2 = 15e-3

d3=275e-3;		% m
% d5=180e-3;
d5 = 280e-3;
% d6=203.56e-3;
d6 = 300e-3;
d7=240e-3;
G= 250*9.8;
Gi=400*9.8;
%material
E = 180e9;

%%%%%%%%%%%%%%%%%%%%%%%%L板的校核
B = 430e-3;
A_L_up   = d1*B;		%B为 L板的纵向长度
A_L_down = d4*B;        %B为 L板的下板面积
M = d6*Gi;
Iz = B*d1^3/12;
disp('***************************************************');
%%%强度校核
	%L上板
	rou_Fn = Gi/A_L_up;
	rou_bend = (M*d1/2) / Iz;
	rou_L = rou_Fn+rou_bend;
    disp(sprintf('强度校核:L形板上板的厚度 %.0f mm',d1*1e3));
    disp(sprintf('强度校核:L形板上板的最大正应力 %.2e',rou_L));
	%L下板
    Iz_x = B*d4^3/12;
	rou_Ft = Gi/A_L_down;
	rou_bend = (M*d4/2)/Iz_x;
	rou_down = sqrt(rou_Ft^2+rou_bend^2);
    disp(sprintf('强度校核:L形板下板的厚度 %.0f mm',d4*1e3));
    disp(sprintf('强度校核:L形板下板的最大正应力 %.2e',rou_down));
    
    disp(sprintf('safty factor equals %.1f',400/160));
    
%%%刚度校核
	theta = (M/(Iz*E)*d3)*180/pi;	%偏转角度
    theta = vpa(subs((M/(Iz*E)*d6)*180/pi,'d1',10e-3),5);
    disp(sprintf('刚度校核:L形板偏转角 %.2e',theta));
	disp(sprintf('泵末端位移 %.2f mm',vpa(600*tan(theta/180*pi),6)))
	
%%%稳定性判据
%     lamba = pi*sqrt(E/rou_p)
%     theta = vpa(subs(-M/sqrt(E*Iz*Gi)*sin(sqrt(Gi/(E*Iz))*d3)*180/pi,'d1',15e-3),5)
disp(sprintf('泵的稳定性 %.2E N',pi^2*E*Iz/(2*d3)^2))


    %% 
%%%%%夹紧机构板子应力校核
    % d1 = 15e-3;         %定L型板宽度为15mm           
	B_clamp = 32.4e-3;  %夹紧板的宽度
	d = 12.4e-3;		%槽孔直径
	h =12e-3;           %夹紧板的高度
    disp(sprintf('夹紧板的高度取 %.0f mm',h*1e3));
	Izz = (B_clamp-d)*h^3/12;
    M = 30;  %夹紧板受力
	rou_bend = (M*h/2) / Izz;
    
    F = 500;
    rou = F/((B_clamp-d)*h);
    
    n = 2;      %安全系数
    
    disp(sprintf('应力校核:夹紧机构板子的最大正应力 %.2e',2*sqrt(rou^2+4*rou_bend^2)));
	disp(sprintf('安全系数取 %.0f',n));
    vpa(n*rou_down,5);
	
