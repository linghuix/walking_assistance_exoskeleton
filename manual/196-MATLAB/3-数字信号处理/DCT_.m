clc
clear
x =  (1:100) + 50*cos((1:100)*2*pi/40);
X = dct(x);
[XX,ind] = sort(abs(X),'descend');
i = 1;
while norm(X(ind(1:i)))/norm(X) < 0.99
   i = i + 1;
end
needed = i

% Reconstruct the signal and compare it to the original signal.

X(ind(needed+1:end)) = 0;
xx = idct(X);

plot([x;xx]')
legend('Original',['Reconstructed, N = ' int2str(needed)], ...
       'Location','SouthEast')