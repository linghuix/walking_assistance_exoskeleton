function[path,pheromone] = searchpath(PopNumber,x_grid,z_grid,pheromone,starty,starth,endy,endh,a,b,h)
% 该函数用于蚁群箅法的路径规划
% x_grid		input	横向划分格数
% z_grid		input	纵向划分格数
% pherormone	input	信息家
% starty starth	input	开始点
% path			output	规划路径
% pheromone		output	信息家

% 搜索参数		
ycMax = 1;		% 蚂蚁最大横向变动格数
hcMax = 1;		% 蚂蚁最大纵向变动格数
decr = 0.9;		% 信息素衰减概率

% 循环搜索路径
for ii = 1: PopNumber
	path(ii,1:2) = [starty,starth];	% 记录路径序号，下同样是序号
	NowPoint = [starty,starth];			% 当前坐标点序号
	%% 计算下一点所有情况的适应度值 
	%横坐标变化,不允许走回头路
	for abscissa = 2: x_grid
       kk= 1;
	   %视觉范围内变化
		for i = - ycMax:ycMax
			%高度方向变化，或者说是海底深度的方向变化
			for j = - hcMax:hcMax
				NextPoint(kk,:) = [NowPoint(1)+i,NowPoint(2)+j];
				% 判断下一个点是否在区间内，两个坐标都位于[0 10]
				if (NextPoint(kk,1)<10/a)&&(NextPoint(kk,1)>0)&&(NextPoint(kk,2)<10/h)&&(NextPoint(kk,2)>0)
					%保存启发概率
					qfz(kk) = CacuQfz(NextPoint(kk,1),NextPoint(kk,2),NowPoint(1),NowPoint(2),endy,endh,abscissa,a,b,h,x_grid);
					%与该点信息素作用,此处NextPoint必须为整数
 
					qz(kk) = qfz(kk)*pheromone(abscissa,NextPoint(kk,1),NextPoint(kk,2));
					kk=kk+1;
				else
					qz(kk) = 0; 
					kk = kk+ 1;
				end
			end
        end

		%%选择下一个点
		%视觉范围内的所有情况的概率计算
		sumq = qz./sum(qz);
		pick = rand ;
		%最多只有25(5*5)种情况
		for i= 1:9
			pick= pick - sumq(i); 
			if pick <= 0 
				index = i;
				break;
			end
		end
		% 选中下一点
		oldpoint = NextPoint(index,:);
		%局部更新信息素，下一点信息素少一半
        % oldpoint
		pheromone(abscissa+1,oldpoint(1),oldpoint(2)) = 0.5*pheromone(abscissa+1,oldpoint(1),oldpoint(2));
		%路径保存
		path(ii,abscissa*2-1:abscissa*2) = [oldpoint(1),oldpoint(2)];
		NowPoint= oldpoint;
	end
	% 最后一点添加
	path(ii,(x_grid+1)*2-1:(x_grid+1)*2) = [endy,endh];
end
