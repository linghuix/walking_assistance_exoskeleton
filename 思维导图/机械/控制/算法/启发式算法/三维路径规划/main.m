clc
clear


%算法参数 
PopNumber= 30; 	%种群规模
BestFitness= [];%最佳个体

% % % % % % % % % % %一格对应0.1
%y方向比例尺
a = 0.1;
%x方向比例尺
b = 0.1;
%h方向比例尺
h = 0.01;
%搜索范围[0 10]
x = 10;
y = 10;
z = 10;
% 计算划分格数
x_grid = x/b;
y_grid = y/a;
z_grid = z/h;

%% 建立序号矩阵，保存信息
%初始化信息素 
pheromone= ones(x_grid+1,y_grid+1,z_grid+1);
% 开始序号
starty =1;
% 未满一格的进上一格
starth = round(HeightData(starty*a,1*b)/h+0.5);
endy = 90;
endh = round(HeightData(endy*a,(x_grid+1)*b)/h+0.5);


 
%%初始搜索路径
[path,pheromone] = searchpath(PopNumber,x_grid,z_grid,pheromone,starty,starth,endy,endh,a,b,h);
fitness= CacuFit(path,a,b,h);			%适应度值计算
[bestfitness,bestindex] = min(fitness);	%最佳适应度值
bestpath = path(bestindex,:);			%最佳路径
BestFitness = [BestFitness;bestfitness];%适应度值记录

%%信息素更新 
rou =0.2;
cfit = 100/bestfitness; 
for i = 2:x_grid
	pheromone(i,bestpath(i*2-1),bestpath(i*2)) = (1-rou)*pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
end

%%循环寻找最优路径 
for kk= 1:200
	%%路径搜索
	[path,pheromone] = searchpath(PopNumber,x_grid,z_grid,pheromone,starty,starth,endy,endh,a,b,h);
		
	%%适应度值计算
	fitness = CacuFit(path,a,b,h);
	[newbestfitness,newbestindex] = min(fitness); 
	if newbestfitness < bestfitness
		bestfitness = newbestfitness;
		bestpath= path(newbestindex,:);
	end
	BestFitness= [BestFitness;bestfitness];
	
	%%更新倌息素 
	cfit = 100/bestfitness; 
	for i = 2:x_grid
		pheromone(i,bestpath(i*2-1),bestpath(i*2)) = (1-rou)*pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
	end
end

figure
plot(1:kk+1,BestFitness);

figure
[X,Y] = meshgrid(0:0.1:11);
R = sqrt((X+5).^2 + (Y+5).^2);
Z = sin(R)./R + 0.28;
mesh(X,Y,Z)
hold on
x=[];
y=[];
z=[];
for i = 1:x_grid+1
	j = (i-1)*b;
	jj = (bestpath(i*2-1)-1)*a;
	jjj = (bestpath(i*2)-1)*h;
%     plot3(j,jj,jjj,'o');
    hold on
    x = [x j];
    y = [y jj];
    z = [z jjj];
end
plot3(x,y,z,'--');