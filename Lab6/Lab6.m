%% Laboratorio 6 
%% PARAMETROS 
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

%% PRIMERA PARTE
A3=[(-1*(r1+r2)/(r1*r2*c1)), (1/(r2*c1)), -1/(r2*c1); 0,0, -1/(r3*c2); -1/(r2*c3),1/(r2*c3), -1*(r2+r3)/(r2*r3*c3)];
B3=[1/(r1*c1);0;0];
C3=[0,1,0];
% sistema de estados, original
sys=ss(A3,B3,C3,0);


% tenemos n = 3 variables de estado/dim de matriz.
est_ver = eig(A3); % el sistema es estable
gamma = ctrb(sys); % hasta n-1, la matriz de contrabilidad 
R_gamma = rank(gamma); % el sistema es completamente controlable [C.C.]

% matriz de polos de prueba
p = [-200, -350, -1100];
%p = [-1100, -100-150i, -100+150i];

% Utilizando el control u = Kx, POLE PLACEMENT
Kpp = place(A3,B3,p);

% Creando nueva matriz Acl y su variable SS.
Acl = A3-B3*Kpp; % MATRIZ DE LAZO CERRADO
sysAcl = ss(A3-B3*Kpp,B3,C3,0);
linearSystemAnalyzer(sysAcl);

% RE-ESCALAR LA ENTRADA para mostrar el step correcto
Nbar = rscale(sys,Kpp);

% SALIDA LAZO ABIERTO utilizando LSIM
[y_2,tOut_1,x_2] =lsim(sys,u_1_2,t);
figure (1)
plot(t,y_2);
hold on
plot(t,u_1_2);
hold off
title('Salida Sistema Abierto, Variable ss');
xlabel('t (s)')
ylabel('Amplitud')

% SALIDA LAZO CERRADO utilizando LSIM
[y_1,tOut_1,x_1] =lsim(sysAcl,Nbar*u_1_2,t);
figure (2)
plot(t,y_1);
hold on
plot(t,u_1_2);
hold off
title('Salida Sistema Cerrado, Variable ss');
xlabel('t (s)');
ylabel('Amplitud');

% %% Simulación
% dt = 0.01; % período de muestreo (step size)
% t0 = 0;
% tf = 10;
% K = (tf-t0) / dt;
% % Inicialización y condiciones iniciales
% x0 = 0.1 .* ones(3,1); % condiciones iniciales 
% u0 = zeros(3,1);
% x = x0; % vector de estado
% u = u0; % entrada
% % Array para almacenar las trayectorias de las variables de estado
% X = zeros(numel(x0), K+1); X(:,1) = x;
% % Array para almacenar la evolución de las entradas 
% U = zeros(numel(u0), K+1); U(:,1) = u; 
% 
% %% Discretización del sistema continuo
% % Discretización por ZOH
% sysd = c2d(sys, dt, 'zoh');
% A3d = sysd.A;
% B3d = sysd.B;
% C3d = sysd.C;
% 
% %% Solución recursiva del sistema dinámico 
% for k = 1:K
%     % Implementación del controlador
%     u = -Kpp*x; % retroalimentación de estado
%     
%     % Se propaga el sistema LTI discreto para aproximar la solución del
%     % sistema continuo
%     x = A3d*x + B3d*u;
%     
%     % Se guardan las trayectorias del estado, entrada y salida
%     X(:, k+1) = x;
%     U(:, k+1) = u;
% end
% 
% % Graficamos los resultados
% t = t0:dt:tf;
% figure;
% plot(t, X', 'LineWidth', 1);
% title('Trayectorias de variables de estado');
% xlabel('$t$','Interpreter','latex','FontSize', 16);
% ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
% l = legend('$x_1(t)$', '$x_2(t)$', '$x_3(t)$', '$x_4(t)$', '$x_5(t)$', ...
%     '$x_6(t)$', '$x_7(t)$', '$x_8(t)$', 'Location', 'southeast');
% set(l, 'Interpreter', 'latex', 'FontSize', 12);
% 
% figure;
% plot(t, U', 'LineWidth', 1);
% title('Entradas al sistema con respecto al tiempo');
% xlabel('$t$','Interpreter','latex','FontSize', 16);
% ylabel('$\mathbf{u}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
% l = legend('$u_1(t)$', '$u_2(t)$', '$u_3(t)$', '$u_4(t)$', ...
%     'Location', 'southeast');
% set(l, 'Interpreter', 'latex', 'FontSize', 12);
% 
