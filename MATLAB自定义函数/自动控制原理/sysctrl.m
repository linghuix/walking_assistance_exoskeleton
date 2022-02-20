function [Co] = sysctrl(A,B,C,D)
Co = ctrb(A,B);
T = [C*Co D];
Ty = rank(T);
n = size(C,1);
r = rank(Co);
L = length(A);
if(n==Ty)
    disp('仅供参考，一切以矩阵为依据，系统输出完全可控');
else
    disp('仅供参考，一切以矩阵为依据，系统输出不完全可控');
end

if(r==L)
    disp('系统是状态完全可控');
else
    disp('系统是状态不完全可控');
end

end