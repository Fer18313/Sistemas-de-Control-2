% Julio Lopez
% fer
% laboratorio 4
%% ---------- parte 2 -----------------------
m1 = 320;
m2 = 2500;
k = 500000; 


ks = 80000;
fv = 15020;
fs = 350;
%________________numerador___________________
a1= ks/(m1*m2);
a0 = fs/(m1*m2);
%________________denominador_________________
b4 = ks*k/(m1*m2);
b3 = (fs*k+ks*fv)/(m1*m2);
b2 = (m2*(k+ks)+fs*fv+m1*ks)/(m1*m2);
b1 = (m2*(fv+fs)+m1*fs)/(m1*m2);
b0 = 1;
%-----------funcion de tras------------------
G = tf([a0 a1], [b0 b1 b2 b3 b4]);
[A1,B1,C1,D1]=tf2ss(G.Numerator{1},G.Denominator{1});


A = [0,        0,            1,         0;...
     0,        0,            0,         1;...
-(k+ks)/m1,   ks/m1,     -(fs+fv)/m1,  fs/m1;...
   ks/m2,    -ks/m2         fs/m2      -fs/m2];


B = [0;...
     0;...
     1/m1;...
     0];
 
C = [1 0 0 0;...
     0 1 0 0;...
     0 0 1 0;...
     0 0 0 1];
 
D = [0;0;0;0];