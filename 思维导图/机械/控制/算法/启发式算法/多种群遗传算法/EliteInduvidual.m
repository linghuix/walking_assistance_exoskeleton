function [MaxObjV,MaxChrom] = EliteInduvidual(Chrom,ObjV, MaxObjV, MaxChrom) 
%%人工选择算子
%			变量名		类型	意义
%输人参数	Chrom		Cell	每个元胞为一个种群的编码(移民前的）
%			ObjV		Cell	每个元胞为一个种群所有个体的目标值（移民前的）
%			MaxObjV		Double	各个种群当前最优个体的目标值(选择前的）
%			MaxChrom	Double	各个种群当前最优个体的编码（选择前的）
%输出参数	MaxObjV		Double	各个种群当前最优个体的目标值(选择后的）
%			MaxChrom	Double	各个种群当前最优个体的编码（选择后的）
	
MP = length (Chrom) ;	% 种群数
for i = 1:MP
	%找出第i种群中最优个体
	[MaxO,maxI] = max(ObjV{i}) ;	
	if MaxO> MaxObjV(i)
		%记录各种群的精华个体
		MaxObjV(i) = MaxO;	
		%记录各种群精华个体的编码
		MaxChrom(i,:) = Chrom{i}(maxI,:);	
	end
end