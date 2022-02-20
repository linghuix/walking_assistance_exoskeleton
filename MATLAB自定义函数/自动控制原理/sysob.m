function Co = sysob(A,B,C,D)
n = size(A,1);
Co = obsv(A,C);
r = rank(Co);

if(r==n)
    disp('仅供参考，一切以矩阵为依据，系统是状态完全可观');
else
    disp('仅供参考，一切以矩阵为依据，系统是状态不完全可观');
end

end