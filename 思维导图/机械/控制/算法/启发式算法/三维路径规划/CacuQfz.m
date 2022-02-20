function qfz = CacuQfz(Nexty,Nexth,Nowy,Nowh,endy,endh,abscissa,a,b,h,x_grid)
%%该函数用于计算各点启发值
% Nexty,Nexth	input	下个点坐标
% Nowy,Nowh		input	当前点坐标
% endy,endh		input	终点坐标
% abscissa		input	横坐标
% HeightData	input	地图高度
% qfz			output	启发值

%参数
K = 50;
P = 30;
Nexty = (Nexty-1)*a;
Nexth = (Nexth-1)*h;
Nowy = (Nowy-1)*a;
Nowh = (Nowh-1)*h;
endy = (endy-1)*a;
endh = (endh-1)*h;
abscissa = (abscissa-1)*b;
%%判断下个点是否可达 
if HeightData(Nexty,abscissa) < Nexth
	%空间坐标点位于水空间中，表示可达到
	S=1;
else
	S=0;
end

%%计算启发值 
%距离50/（到达下一点的距离+达到目的地的距离）距离都化为1km
D = K/(sqrt(b^2+(Nowh-Nexth)^2+(Nexty-Nowy)^2)+sqrt(((x_grid+1-abscissa)*b)^2+(endh-Nexth)^2+(endy-Nexty)^2)); 
%计算高度
M = 1/abs(Nexth+1)/h;
%计算启发值 ,使蚂蚁的运动趋向于 《离目的地、前进点近，高度小》的下一点
qfz = S*M*D;