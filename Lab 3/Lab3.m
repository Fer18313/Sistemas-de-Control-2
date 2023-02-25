% lab 3
%r1=1000;
%r2=10000;
%r3=10000;
%c1=0.000001;
%c2=0.0000001;
%c3=0.00001;
syms r1 r2 r3 c1 c2 c3
%valores de matices
a0=1/(r1*r2*r3*c1*c2*c3);
a1=(1/(r1*r3*c1*c3))+(1/(r1*r2*c1*c3))+(1/(r2*r3*c1*c3))+(1/(r2*r3*c2*c3));
a2=(1/(r1*c1))+(1/(r2*c1))+(1/(r3*c3))+(1/(r2*c3));
b0=1/(r1*r2*r3*c1*c2*c3);
%Realacion canonica de control 
A1=[0,1,0; 0,0,1; -a0,-a1,-a2];
B1=[0; 0; 1];
C1=[b0,0,0];
%Realacion canonica de obsevable 
A2=[ -a2,1,0;...
    -a1,0,1;... 
    -a0,0,0];
B2=[0; 0; b0];
C2=[1,0,0];
%ecuacion
A3=[(-1*(r1+r2)/(r1*r2*c1)), (1/(r2*c1)), -1/(r2*c1); 0,0, -1/(r3*c2); -1/(r2*c3),1/(r2*c3), -1*(r2+r3)/(r2*r3*c3)];
B3=[1/(r1*c1);0;0];
C3=[0,1,0];
% funcion de trasferencia 
G0 = tf([b0], [1, a2, a1, a0]);
[A4,B4,C4,D4]=tf2ss(G0.Numerator{1},G0.Denominator{1});

%linealizar 
load('model_linearizacion.mat');
A5=linsys1.A;
B5=linsys1.B;  
C5=linsys1.C; 

%5 𝐆(𝑠) = C(𝑠I − A)−1B + D 
s=tf('s');
G1=C1*inv(s*eye(3)-A1)*B1;
G2=C2*inv(s*eye(3)-A2)*B2;
G3=C3*inv(s*eye(3)-A3)*B3;
G4=C4*inv(s*eye(3)-A4)*B4;
G5=C5*inv(s*eye(3)-A5)*B5;




