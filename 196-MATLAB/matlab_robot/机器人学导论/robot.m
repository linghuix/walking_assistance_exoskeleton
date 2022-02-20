syms th1 th2 d3 L0 d3
rz = @(a) [rotz(a) [0 0 0]';0  0 0 1]; 
rx = @(a) [rotx(a) [0 0 0]';0  0 0 1]; 
ry = @(a) [roty(a) [0 0 0]';0  0 0 1]; 
tz = @(a3) [eye(3) a3';0  0 0 1]; 
tx = @(a3) [eye(3) a3';0  0 0 1]; 
ty = @(a3) [eye(3) a3';0  0 0 1]; 

T_01 = rz(pi/2+th1)*tz([L0 0 0])*rx(pi/2);

T_12 = rz(pi/2+th2)*rx(pi/2);
T_02 = T_01*T_12;

T_23 = tz([d3 0 0]);
T_03 = T_02*T_23;

T_01 = vpa(T_01,4)
T_02 = vpa(T_02,4)
T_03 = vpa(T_03,4)