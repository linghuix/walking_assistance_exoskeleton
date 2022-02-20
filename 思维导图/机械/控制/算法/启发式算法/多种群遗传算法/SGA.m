%%标准遗传算法SGA
%与MPGA个体相同400个，但是收敛效果不太好，结果每次运行不同，收敛速度也较慢
clear
clc
pc = 0.7 ;
pm = 0.05;
%定义遗传算法参数
NIND = 400;		%个体数目
MAXGEN = 100;	%最大遗传代数
NVAR = 2;		%变量的维数
PRECI = 20;		%变量的二进制位数
GGAP = 0.9;		%代沟
trace = zeros(MAXGEN,1);
FieldD = [rep(PRECI,[1,NVAR]);[-3,4.1;12.1,5.8];rep([1;0;1;1],[1,NVAR])];		%建立区域描述器
Chrom = crtbp(NIND,NVAR*PRECI);	%创建初始种群
gen = 0;						%代计数器
maxY =0;						%最优值
ObjV = ObjectFunction(bs2rv(Chrom, FieldD)) ;	%计棘初始种群个体的目标函数值
while gen<MAXGEN										%迭代
	FitnV = ranking( -ObjV);							% 分配适应度值assign fitness va1ues)
	SelCh = select('sus', Chrom, FitnV, GGAP);			% 选择
	SelCh = recombin( 'xovsp', SelCh, pc);				% 重组
	SelCh = mut(SelCh,pm);								%变异
	ObjVSel = ObjectFunction(bs2rv(SelCh, FieldD));		%计算子代目标函数值
	[Chrom ObjV] = reins(Chrom, SelCh, 1, 1, ObjV, ObjVSel) ;	% 重插人
	gen = gen + 1;	%代计数器增加
	if maxY < max(ObjV) 
		maxY = max(ObjV);
	end
	trace(gen, 1) = maxY;
end

%%进化过程图 
plot(1:gen,trace(:,1));

%%输出最优解 
[Y,I] = max(ObjV);
X = bs2rv(Chrom, FieldD);
disp(['最优值为:',num2str(Y)]);
disp(['对应的自变量取值:',num2str(X(I,:))])