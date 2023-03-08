% Julio Lopez
% fer
% laboratorio 5
%% ---------- parte 1 -----------------------
%index 1

%% parametros 
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
u2_5 = -5*u+2.5;
u5 = -5*u+5;
%plot(t,u5);
%% index 2
load('LinearAnalysisTool.mat')
[y, tout, x] = lsim(LinearAnalysisToolProject.LocalVariables.Value, u, t);

%Gráfica X1
plot(tout, x(1:end,1))
title("Espacio de estados")
xlabel('t (s)')
ylabel('Amplitud')
hold on
%Gráfica X2
plot(tout, x(1:end,2))
hold on
%Gráfica X3
plot(tout, x(1:end,3))
legend('x1','x2','x3')
hold off
%% Inciso 3

A3=[(-1*(r1+r2)/(r1*r2*c1)), (1/(r2*c1)), -1/(r2*c1); 0,0, -1/(r3*c2); -1/(r2*c3),1/(r2*c3), -1*(r2+r3)/(r2*r3*c3)];
B3=[1/(r1*c1);0;0];
C3=[0,1,0];
sys=ss(A3,B3,C3,0);
[y_3, tout_1_3, x_3] = lsim(sys, u, t);

figure(2)
plot(tout_1_3, x_3(1:end,1))
title("Espacio de estados usando A3,B3,C3")
xlabel('t (s)')
ylabel('Amplitud')
hold on
plot(tout_1_3, x_3(1:end,2))
hold on
plot(tout, x_3(1:end,3))
legend('x1','x2','x3')
hold off
%% ---------------------------parte 2----------------------------------- 
%% inciso 2
load('diodo_c1_50.mat')
[y1, tout_1, x1] = lsim(diodo_c1_50.LocalVariables.Value, u5, t);
figure
plot(tout_1, x1(1:end,1))
title("Espacio de estados")
xlabel('t (s)')
ylabel('Amplitud')
hold on
plot(tout_1, x1(1:end,2))
hold on
plot(tout_1, x1(1:end,3))
legend('x1','x2','x3')
hold off

%% inciso 5
[y2, tout_2, x2] = lsim(diodo_c1_50.LocalVariables.Value , u2_5, t);
figure
plot(tout_2, x2(1:end,1))
title("Espacio de estados")
xlabel('t (s)')
ylabel('Amplitud')
hold on
plot(tout_2, x2(1:end,2))
hold on
plot(tout_2, x2(1:end,3))
legend('x1','x2','x3')
hold off

%% inciso 8
load('diodo_c2_50.mat')
[y3, tout_3, x3] = lsim(diodo_c2_50.LocalVariables.Value, u5, t);
figure
plot(tout_3, x3(1:end,1))
title("Espacio de estados")
xlabel('t (s)')
ylabel('Amplitud')
hold on
plot(tout_3, x3(1:end,2))
hold on
plot(tout_3, x3(1:end,3))
legend('x1','x2','x3')
hold off


%% inciso 11
load('diodo_c2_25.mat')
[y4, tout_4, x4] = lsim(diodo_c2_25.LocalVariables.Value, u2_5, t);
figure
plot(tout_4, x4(1:end,1))
title("Espacio de estados")
xlabel('t (s)')
ylabel('Amplitud')
hold on
plot(tout_4, x4(1:end,2))
hold on
plot(tout_4, x4(1:end,3))
legend('x1','x2','x3')
hold off