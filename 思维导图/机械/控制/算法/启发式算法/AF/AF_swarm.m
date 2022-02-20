 function [Xnext,Ynext] = AF_swarm(X,i,visual,step,delta,try_number,LBUB,lastY)  
% 聚群行为：朝伙伴中心位置靠拢

% 输入: 
% x	所有人工鱼的位置 
% i	当前人工鱼的序号 
% visual	感知范围
% step	最大移动步长
% delta	拥挤度
% try_number	最大尝试次数
% LBUB	各个数的上下限
% lastY	上次的各人工鱼位置的食物浓度

% 输出:
% Xnext	Xi人工鱼的下一个位置
% Ynext	Xi人工鱼的下一个位迸的食物浓度

Xi = X(:,i);
%计算鱼群个体之间的距离
D = AF_distance(Xi,X);
%视野内的个体
index = find(D>0 & D<visual); 
nf = length(index); 
if nf>0   
	for j = 1:size(X,1)
		Xc(j,1) = mean(X(j,index));
	end
	Yc= AF_foodconsistence(Xc);
	Yi = lastY(i); 
	%如果鱼群密度比较小
	if Yc/nf>delta*Yi
		Xnext = Xi+rand*step*(Xc-Xi)/norm(Xc-Xi);
		for i = 1:length(Xnext)
			if Xnext( i)>LBUB( i, 2)
				Xnext(i) = LBUB(i,2);
			end
			if Xnext(i)<LBUB(i,1)
				Xnext(i) = LBUB(i,1),
			end
		end
		Ynext= AF_foodconsistence(Xnext);
	else
	%如果鱼群密度比较大，则觅食行为
		[Xnext,Ynext] = AF_prey(Xi,i,visual,step,try_number,LBUB,lastY);
	end
else
%如果视野范围内没有个体，觅食行为
	[Xnext,Ynext]= AF_prey(Xi,i,visual,step,try_number,LBUB,lastY);
end









