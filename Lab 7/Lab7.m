%% Laboratorio 7
% Alumno: Fernando Sandoval
% Carne: 18313
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
sys=ss(A3,B3,C3,0);

%% PRIMERA PARTE
%% INCISO 1
% sistema de estados, original
Q_prim = eye(3);
R_prim = eye(1);
K_prim = lqr(sys,Q_prim,R_prim);
Nbar_prim = rscale(sys,K_prim);
Acl_prim = A3-B3*K_prim; % MATRIZ DE LAZO CERRADO
sysAcl = ss(Acl_prim,B3,C3,0);
%linearSystemAnalyzer(sysAcl)

%% INCISO 2
Q = [0.01 0 0;
    0 1 0;
    0 0 1];
R = 0.1;
K_secun = lqr(sys,Q,R);
Nbar_secun = rscale(sys,K_secun);
Acl_secun = A3-B3*K_secun;
sysAcl = ss(Acl_secun,B3,C3,0);
%linearSystemAnalyzer(sysAcl);

%% SEGUNDA PARTE
Omega = obsv(A3,C3);
rank(Omega); %  es observable
%p = [-1100, -100-150i, -100+150i]; % SECCION 1.2
p = [-1100,-132-200.43i,-132+200.43i];
%p = [-200,-350,-1100]; % SECCION 1.12
%p_L = 3*p
p_L = p-1000; % polos mas agresivos

Kpp = place(A3,B3,p);
Nbar_tercero = rscale(sys,Kpp);
Acl_tercero = A3-B3*Kpp;

Lpp = place(A3',C3',p_L)';

C_L = eye(3);
D_L = zeros(3,2);
B_L = [B3 Lpp];
Acl_L = A3-Lpp*C3;
sysAcl = ss(Acl_L,B_L,C_L,D_L);

%% Tercera parte
% inciso a)
K = lqr(A3, B3, Q_prim,R_prim);
L = lqr(A3', C3', Q_prim,R_prim)';
Acl_L_P = A3-L*C3;
Nbar_K_L = rscale(sys,K);
% inciso b) 
K_new = lqr(A3,B3,Q,R);
L_new = lqr(A3',C3',Q,R)';
Nbar_new = rscale(sys,K_new);
Acl_new = A3-L_new*C3;

