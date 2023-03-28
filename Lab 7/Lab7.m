%% Laboratorio 7
% PARAMETROS
R1=1000;
R3=10000;
R2=R3;
C1=1e-6;
C2=0.1e-6;
C3=10e-6;
r1=1000;
r3=10000;
r2=r3;
c1=1e-6;
c2=0.1e-6;
c3=10e-6;
%datos de entrada
T=1/0.667;
TS=0.0001;
F=2*T;
[u, t] = gensig("square", T, F, TS);
u_1_2 = u+1;
A3=[(-1*(r1+r2)/(r1*r2*c1)), (1/(r2*c1)), -1/(r2*c1); 0,0, -1/(r3*c2); -1/(r2*c3),1/(r2*c3), -1*(r2+r3)/(r2*r3*c3)];
B3=[1/(r1*c1);0;0];
C3=[0,1,0];
%% PRIMERA PARTE
%% INCISO 1
% sistema de estados, original
sys=ss(A3,B3,C3,0);
Q = eye(3);
R = eye(1);
K = lqr(sys,Q,R);
% RE-ESCALAR LA ENTRADA para mostrar el step correcto
Nbar = rscale(sys,K);
Acl = A3-B3*K; % MATRIZ DE LAZO CERRADO
sysAcl = ss(Acl,B3,C3,0);
%% INCISO 2
Q = 0.1*eye(3);
R = 1*eye(1);
K = lqr(sys,Q,R);
Acl = A3-B3*K;
sysAcl = ss(Acl,B3,C3,0);
%linearSystemAnalyzer(sysAcl);

%% SEGUNDA PARTE
