clear  
%  
%DEFINE  
%  
% '---------------------------------------------'   
% '-----------PART 1-------------------------'   
% '---------------------------------------------'   
clear  
DRG=pi/180;  
l1 = 4;  
l2 = 3;  
T0H =  [1    0    0   9 ;    
        0    1    0   0 ; 
        0    0    1   0 ; 
        0    0    0   1];    
T3H =  [1    0    0   2 ;    
        0    1    0   0 ;
        0    0    1   0 ; 
        0    0    0   1];  
NRBA =     [-1    0    0     
            0    -1    0  
            0     0   -1] ;  
PBORG = [2;0;0];    
Ptemp = NRBA*PBORG;  
Ptemp2 = [Ptemp;1];     
R3HT =   [1    0    0  
         0    1    0   
		  0    0    1   
		  0    0    0 ];  
T3HT = [R3HT,Ptemp2];  
T03 = T0H*T3HT  
cphi = T03(1,1);  
sphi = T03(2,1);  
x = T03(1,4);  
y = T03(2,4);  
k = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);  
if k>=-1 && k<=1  
'----------------solution--1------------'  
theta21 = atan2((1-k^2)^0.5,k);  
k11 = l1+l2*cos(theta21);  
k21 = l2*sin(theta21);  
theta11 = atan2(-(k21*x)+k11*y,k11*x+k21*y);  
theta31 = atan2(sphi,cphi)-theta11-theta21;  
degree11 = theta11/DRG  ;
degree21 = theta21/DRG  ;
degree31 = theta31/DRG  ;
'----------------solution--2------------'  
theta22 = atan2(-((1-k^2)^0.5),k);  
k12 = l1+l2*cos(theta22);  
k22 = l2*sin(theta22);  
theta12 = atan2(-(k22*x)+k12*y,k12*x+k22*y);  
theta32 = atan2(sphi,cphi)-theta12-theta22;  
degree12 = theta12/DRG  
degree22 = theta22/DRG  
degree32 = theta32/DRG  
else  
'THERE HAVE NO SOLUTIONS, BECAUSE COSINE THETA 2 NOT IN -1 TO 1'  
end  
'---------------------------------------------'   
'-----------PART 2-------------------------'   
'---------------------------------------------'   
clear  
DRG=pi/180;  
l1 = 4  
l2 = 3  
T0H =  [0.5  -0.866   0   7.5373     
       0.866  0.5    0   3.9266  
       0    0    1   0  
       0    0    0   1]  
T3H =  [1    0    0   2     
       0    1    0   0  
       0    0    1   0  
       0    0    0   1]  
   
NRBA =   [-1    0    0     
       0    -1    0    
       0     0    -1] ;  
   
PBORG = [2;0;0];      
Ptemp = NRBA*PBORG;   
Ptemp2 = [Ptemp;1];         
R3HT =   [1    0    0  
       0    1    0   
       0    0    1   
       0    0    0 ];      
T3HT = [R3HT,Ptemp2];  
    
T03 = T0H*T3HT      
cphi = T03(1,1);  
sphi = T03(2,1);  
x = T03(1,4);  
y = T03(2,4);  
k = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);  
l1 = 4  
l2 = 3    
if k>=-1 && k<=1     
%  '----------------solution--1------------'  
 theta21 = atan2((1-k^2)^0.5,k);  
 k11 = l1+l2*cos(theta21);  
 k21 = l2*sin(theta21);  
 theta11 = atan2(-(k21*x)+k11*y,k11*x+k21*y);  
 theta31 = atan2(sphi,cphi)-theta11-theta21;  
   
 degree11 = theta11/DRG  
 degree21 = theta21/DRG  
 degree31 = theta31/DRG  
   
   
%  '----------------solution--2------------'  
 theta22 = atan2(-((1-k^2)^0.5),k);  
 k12 = l1+l2*cos(theta22);  
 k22 = l2*sin(theta22);  
 theta12 = atan2(-(k22*x)+k12*y,k12*x+k22*y);  
 theta32 = atan2(sphi,cphi)-theta12-theta22;  
   
 degree12 = theta12/DRG  
 degree22 = theta22/DRG  
 degree32 = theta32/DRG  
else  
%  'THERE HAVE NO SOLUTIONS, BECAUSE COSINE THETA 2 NOT IN -1 TO 1'  
%  end  
%  '---------------------------------------------'   
%  '-----------PART 3-------------------------'   
%  '---------------------------------------------'  
 clear  
 DRG=pi/180;  
 l1 = 4  
 l2 = 3     
 T0H =  [0    1    0   -3     
       -1   0    0   2  
       0    0    1   0  
       0    0    0   1]  
   
 T3H =  [1    0    0   2     
       0    1    0   0  
       0    0    1   0  
       0    0    0   1]  
   
 NRBA =   [-1    0    0     
       0    -1    0    
       0     0    -1] ;  
   
 PBORG = [2;0;0];      
   
 Ptemp = NRBA*PBORG;  
   
 Ptemp2 = [Ptemp;1];  
         
 R3HT =   [1    0    0  
       0    1    0   
       0    0    1   
       0    0    0 ];  
       
 T3HT = [R3HT,Ptemp2];  
    
 T03 = T0H*T3HT  
        
 cphi = T03(1,1);  
 sphi = T03(2,1);  
 x = T03(1,4);  
 y = T03(2,4);  
 k = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);  
 l1 = 4  
 l2 = 3  
   
 if k>-1 && k<1  
   
%  '----------------solution--1------------'  
 theta21 = atan2((1-k^2)^0.5,k);  
 k11 = l1+l2*cos(theta21);  
 k21 = l2*sin(theta21);  
 theta11 = atan2(-(k21*x)+k11*y,k11*x+k21*y);  
 theta31 = atan2(sphi,cphi)-theta11-theta21;  
   
 degree11 = theta11/DRG  
 degree21 = theta21/DRG  
 degree31 = theta31/DRG  
   
   
%  '----------------solution--2------------'  
 theta22 = atan2(-((1-k^2)^0.5),k);  
 k12 = l1+l2*cos(theta22);  
 k22 = l2*sin(theta22);  
 theta12 = atan2(-(k22*x)+k12*y,k12*x+k22*y);  
 theta32 = atan2(sphi,cphi)-theta12-theta22;  
   
 degree12 = theta12/DRG  
 degree22 = theta22/DRG  
 degree32 = theta32/DRG  
   
   
 else  
   
%  'THERE HAVE NO SOLUTIONS, BECAUSE COSINE THETA 2 NOT IN -1 TO 1'  
   
 end  
%  '---------------------------------------------'   
%  '-----------PART 4-------------------------'   
%  '---------------------------------------------'   
 clear  
 DRG=pi/180;  
 l1 = 4  
 l2 = 3   
 T0H =  [0.866  0.5    0   -3.1245     
       -0.5   0.866  0   9.1674  
       0    0    1   0  
       0    0    0   1]  
   
 T3H =  [1    0    0   2     
       0    1    0   0  
       0    0    1   0  
       0    0    0   1]  
   
 NRBA =   [-1    0    0     
       0    -1    0    
       0     0    -1] ;  
   
 PBORG = [2;0;0];      
   
 Ptemp = NRBA*PBORG  
   
 Ptemp2 = [Ptemp;1]  
         
 R3HT =   [1    0    0  
       0    1    0   
       0    0    1   
       0    0    0 ];  
       
 T3HT = [R3HT,Ptemp2]  
    
 T03 = T0H*T3HT  
        
 cphi = T03(1,1)  
 sphi = T03(2,1)  
 x = T03(1,4)  
 y = T03(2,4)  
 k = (x^2+y^2-l1^2-l2^2)/(2*l1*l2)  
 l1 = 4  
 l2 = 3  
   
 if k>-1 && k<1  
   
%  '----------------solution--1------------'  
 theta21 = atan2((1-k^2)^0.5,k)  
 k11 = l1+l2*cos(theta21)  
 k21 = l2*sin(theta21)  
 theta11 = atan2(-(k21*x)+k11*y,k11*x+k21*y)  
 theta31 = atan2(sphi,cphi)-theta11-theta21  
   
 degree11 = theta11/DRG  
 degree21 = theta21/DRG  
 degree31 = theta31/DRG  
   
   
%  '----------------solution--2------------'  
 theta22 = atan2(-((1-k^2)^0.5),k)  
 k12 = l1+l2*cos(theta22)  
 k22 = l2*sin(theta22)  
 theta12 = atan2(-(k22*x)+k12*y,k12*x+k22*y)  
 theta32 = atan2(sphi,cphi)-theta12-theta22  
   
 degree12 = theta12/DRG  
 degree22 = theta22/DRG  
 degree32 = theta32/DRG  
   
 else  
   
%  'THERE HAVE NO SOLUTIONS, BECAUSE COSINE THETA 2 NOT IN -1 TO 1'  
   
 end  
   
%  '---------------------------------------------'   
%  '-----------PART 5-------------------------'   
%  '---------------------------------------------'   
%  '--------using---ikine------------------------'  
%  ' '   
%  '-----------example 1-------------------------'   
%  '---------------------------------------------'   
 clear  
 l1 = 4;  
 l2 = 3;  
 l3 = 2;  
 DRG=pi/180;  
 t1 = 0*DRG;  
 t2 = 0*DRG;  
 t3 = 0*DRG;  
 LK1 = LINK([ 0 0 t1  0 0]);  
 LK2 = LINK([ 0 l1 t2 0 0]);  
 LK3 = LINK([ 0 l2 t3 0 0]);  
 LKH = LINK([ 0 l3 0  0 0]);  
 r0H = ROBOT({LK1,LK2,LK3,LKH})  
 r0H.name = 'gripe'  
 q0H=[0 t1 t2 t3]  
 T0H=fkine(r0H,q0H)  
 M = [1,1,1,1,0,0]  
 Q = ikine(r0H,T0H,q0H,M)  
 degree = Q/DRG  
 '-----------example 2-------------------------'   
 '---------------------------------------------'   
 clear  
 l1 = 4;  
 l2 = 3;  
 l3 = 2;  
 DRG=pi/180;  
 t1 = 10*DRG;  
 t2 = 20*DRG;  
 t3 = 30*DRG;  
 LK1 = LINK([ 0 0 t1  0 0]);  
 LK2 = LINK([ 0 l1 t2 0 0]);  
 LK3 = LINK([ 0 l2 t3 0 0]);  
 LKH = LINK([ 0 l3 0  0 0]);  
 r0H = ROBOT({LK1,LK2,LK3,LKH})  
 r0H.name = 'gripe'  
 q0H=[0 t1 t2 t3]  
 T0H=fkine(r0H,q0H)  
 M = [1,1,1,1,0,0]  
 Q = ikine(r0H,T0H,q0H,M)  
 degree = Q/DRG  
 '-----------example 3-------------------------'   
 '---------------------------------------------'   
 clear  
 l1 = 4;  
 l2 = 3;  
 l3 = 2;  
 DRG=pi/180;  
 t1 = 90*DRG;  
 t2 = 90*DRG;  
 t3 = 90*DRG;  
 LK1 = LINK([ 0 0 t1  0 0]);  
 LK2 = LINK([ 0 l1 t2 0 0]);  
 LK3 = LINK([ 0 l2 t3 0 0]);  
 LKH = LINK([ 0 l3 0  0 0]);  
 r0H = ROBOT({LK1,LK2,LK3,LKH})  
 r0H.name = 'gripe'  
 q0H=[0 t1 t2 t3]  
 T0H=fkine(r0H,q0H)  
 M = [1,1,1,1,0,0]  
 Q = ikine(r0H,T0H,q0H,M)  
 degree = Q/DRG  
