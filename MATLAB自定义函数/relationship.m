% 互相关函数：输入 X_t采样值，Y_t采样值，Fs 采样信号
function rela = relationship(X_t,Y_t)
    N_t = length(X_t);
    % matlab的数组最小值从1开始，所以需要变化一下。
    Rxy = zeros(1,N_t);
    for r = 1:N_t
        for n = 1:N_t-r+1
            Rxy(r) = Rxy(r) + X_t(n)*Y_t(n+r-1);
        end

        Rxy(r) = Rxy(r)/(N_t-r+1);
    end
    rela = Rxy;
end