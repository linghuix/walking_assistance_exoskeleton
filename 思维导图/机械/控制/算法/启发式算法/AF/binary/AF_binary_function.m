clc
clear 
close all
tic
figure(1) ; 
hold on

%参数设置
	fishnum= 100;
	MAXGEN = 50;
	try_number = 100;
	%感知距离
	visual = 2.5 ;
	%拥挤度因子
	delta =0.618;	
	%移动步长
	step =0.3;	
%初始化鱼群 
lb_ub = [-10,10,2;];
X = AF_init(fishnum,lb_ub);
LBUB=[];
for i= 1:size(lb_ub,1)
	LBUB= [LBUB;repmat(lb_ub(i,1:2),lb_ub(i,3),1)];
end
gen = 1 ;
%每步中最优的函数值
BestY= - 1 * ones(1,MAXGEN);	
BestX= - 1 * ones(2,MAXGEN);
%最优函数值
besty = - 100;	
Y= AF_foodconsistence(X); 
while gen <= MAXGEN
	fprintf(1, '%d\n',gen) 
	for i = 1 :fishnum
		%聚群行为
		[Xi1,Yi1]=AF_swarm(X,i,visual,step,delta,try_number,LBUB,Y); 
		%追尾行为
		[Xi2,Yi2]=AF_follow(X,i,visual,step,delta,try_number,LBUB,Y); 
		if Yi1>Yi2
			X(:,i) = Xi1;
			Y(1,i) = Yi1;
		else
			X(:,i) = Xi2;
			Y(1,i) = Yi2;
		end
	end
	[Ymax, index] = max(Y); 
    
    % 每一代的鱼群最优个体绘制，颜色由黑变红
	figure(1);
	plot(X(1,index),X(2,index),'.', 'color',[gen/MAXGEN ,0,0]);
	%记录最优解
	if Ymax>besty 
		besty = Ymax ; 
		bestx = X( :, index);
		BestY(gen) = Ymax;
		[BestX(:,gen)] = X(:,index);
	else
		BestY(gen) = BestY(gen - 1);
		
disp(['最优解 X:',num2str(bestx','  %1.5f')]);
disp(['最优解 Y:',num2str(besty,'%1.5f')]);
		BestX(:,gen) = BestX(:,gen - 1);
	end
	gen = gen+1 ;
    
%    记录最后一次迭代最优解所在方位
    if gen == MAXGEN
        plot(X(1),X(2),'.', 'color',[gen/MAXGEN ,0,0],'markerSize',20);
    end
end


% 用红色的大圆醒目的标明最优值所在坐标
plot(bestx(1) ,bestx(2), 'ro', 'MarkerSize', 80);
xlabel('x')
ylabel('y')
title('鱼群算法迭代过程中最优坐标移动')

% 绘制鱼群算法迭代过程中的历史最优值的变化图
figure
plot(1:MAXGEN,BestY);
xlabel('迭代次数');
ylabel('历史最优值'); 
title('鱼群算法迭代过程');
toc
