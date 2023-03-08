% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
% PRE LABORATORIO
% SISTEMAS DE CONTROL 2 - LABORATORIO 1


R1=1000;
R3=10000;
R2=R3;
C1=1e-6;
C2=0.1e-6;
C3=10e-6;

s_hand = tf([C3*R3,1],[C1*R1*C2*R3*C3*R2,(C1*R1*C2*R3 + C1*R1*R3*C3 + C2*R3*C3*R2),(C1*R1 + C2*R3 + R3*C3),1])
% son parecidas

s = tf(linsys1)
% son parecidas
pole(s)
pzmap(s);
%es un sistema estable

linearSystemAnalyzer(s)



%% hola 


G0 = tf(b_0, dem);

%Trayectorias de las variables de estado
tau = 2; %período de la señal
Ts = 0.0001; %tiempo de muestreo t = 0:Ts:Tf
Tf = 2*tau; %Tiempo final

[u, t] = gensig("square", tau, Tf, 0.0001);
u = -5*u+5;
%Verificación de la señal generada
%plot(t,u)

%Inciso 2
[y, tout, x] = lsim(linsys1, u, t);

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

%Inciso 3
[y_3, tout_3, x_3] = lsim(linsys3, u, t);

figure()
plot(tout, x_3(1:end,1))
title("Espacio de estados usando A3,B3,C3")
xlabel('t (s)')
ylabel('Amplitud')
hold on
%Gráfica X2
plot(tout, x_3(1:end,2))
hold on
%Gráfica X3
plot(tout, x_3(1:end,3))
legend('x1','x2','x3')
hold off

