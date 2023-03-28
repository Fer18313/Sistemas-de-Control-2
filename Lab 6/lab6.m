% =========================================================================
% IE3041 - LABORATORIO 6: Retroalimentación de Estados
% Sistemas de Control 2
% -------------------------------------------------------------------------


C3 = [0, 1, 0,0;...
      0, 0, 1,0;...
      0, 0, 0,1;...
      -16, -20, -3,-1];




%% primera parte 
R1 = 1e3;
R2 = 10e3;
R3 = 10e3;
C1 = 1e-6;
C2 = 0.1e-6;
C3 = 10e-6;
%Armamos el modelo de función de transferencia
b_0 = 1/(R1*R2*R3*C1*C2*C3); %a0
a_0 = b_0;
a_2 = 1/(C1*R1) + 1/(C1*R2) + 1/(C3*R3) + 1/(C3*R2); %b0
a_1 = 1/(C2*C3*R2*R3) + 1/(C1*C3*R2*R3) + 1/(C1*C3*R1*R3) + 1/(C1*C3*R1*R2); %c0
dem = [1, a_2, a_1, b_0];
G0 = tf(a_0, dem);
A3 = [-(R1+R2)/(R1*R2*C1), 1/(R2*C1),-1/(R2*C1);0,0,-1/(R3*C2);-1/(R2*C3),1/(R2*C3),-(R2+R3)/(R2*R3*C3)];
B3 = [1/(R1*C1);0;0];
C3 = [0,1,0];
sistema_original = ss(A3, B3, C3, 0);
%Verifiquen la controlabilidad del sistema 
Control_matrix = ctrb(A3, B3);
Controlable = rank(Control_matrix);


%% *************************************************************************
%Pole placement
%p = [-1100, -340-200i, -340+200i]; %valor óptimo
p = [-1100, -100-150i, -100+150i]; %valor inicial

K = place(A3, B3, p);
Acl = A3 - B3*K;
sistema_mejorado = ss(Acl,B3,C3,0);

%% ************************************************************************
% Señal de prueba
%Trayectorias de las variables de estado
tau = 2; %período de la señal
Ts = 0.0001; %tiempo de muestreo t = 0:Ts:Tf
Tf = 2*tau; %Tiempo final

[u, t] = gensig("square", tau, Tf, 0.0001);
u = -1*u+2;
%Verificación de la señal generada
%plot(t,u)
Nbar = rscale(sistema_original,K);
%%Inciso 7
[ysim1, tsim1, xsim1] = lsim(sistema_mejorado, Nbar*u, t);
[ysim2, tsim2, xsim2] = lsim(sistema_original, u, t);

%Graficas de resultados
figure(1);
plot(tsim1, ysim1, 'b');
title('Grafica del sistema controlado');
xlabel('t(s)');
ylabel('Amplitud');
hold on
plot(t, u, 'r');
hold off

figure(2);
plot(tsim2, ysim2, 'r');
title('Grafica del sistema original');
xlabel('t(s)');
ylabel('Amplitud');
hold on
plot(t, u, 'b');
hold off

linearSystemAnalyzer(sistema_mejorado,'r.');




