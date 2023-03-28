% =========================================================================
% IE3041 - EJEMPLO CICLO LÍMITE
% -------------------------------------------------------------------------
% En este ejemplo se muestra el comportamiento de un ciclo límite para el
% caso del oscilador de Van der Pol.
% =========================================================================
%% Parámetros y dinámica del sistema
m = 2; % kg
ell = 1; % m
g = 9.81; % m/s^2

% Campo vectorial del sistema
f = @(x,u) [ x(2); 
             1*(1-x(1)^2)*x(2) - x(1) ];
         
%% Parámetros de la simulación
t0 = 0;
tf = 20; % tiempo de simulación
dt = 0.01; % período de muestreo
K = (tf - t0) / dt; % número de iteraciones

%% Inicialización y condiciones iniciales
% Sistema no lineal
x0 = [1; 0]; % condición inicial
x = x0; % vector de estado
% Array para almacenar las trayectorias de las variables de estado
X = zeros(numel(x0), K+1); X(:,1) = x; 

%% Solución recursiva del sistema dinámico
for k = 1:K
    % Método RK4 para la aproximación numérica de la solución del sistema
    % no lineal
    k1 = f(x, 0);
    k2 = f(x+(dt/2)*k1, 0);
    k3 = f(x+(dt/2)*k2, 0);
    k4 = f(x+dt*k3, 0);
    x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    
    % Se guardan la trayectoria del vector de estado
    X(:, k+1) = x;
end


%% Animación y generación de figuras (NO modificar)    
% Animación de las trayectorias del sistema en el espacio de estados
figure;
meshres = 0.1;
lims = 3;
xlim([-lims, lims]);
ylim([-lims, lims]);
xlabel('$x_1$', 'Interpreter', 'latex', 'FontSize', 18);
ylabel('$x_2$', 'Interpreter', 'latex', 'FontSize', 18);

% Diagrama de fase del sistema no lineal
[mX1, mX2] = meshgrid(-lims:meshres:lims, -lims:meshres:lims);
[mDX1, mDX2] = phaseportrait2d(f, mX1, mX2, 0);
phasep_nonlin = streamslice(mX1, mX2, mDX1, mDX2);
set(phasep_nonlin, 'Color', [0.5, 0.5, 0.5]);
hold on;

% Trayectoria del sistema no lineal
flowp_nonlin = plot(X(1,1), X(2,1), 'k', 'LineWidth', 1);
    
% Se actualizan las gráficas y se pausa el programa para dar la 
% ilusión de animación
for k = 2:K+1
    flowp_nonlin.XData = [flowp_nonlin.XData, X(1,k)];
    flowp_nonlin.YData = [flowp_nonlin.YData, X(2,k)];
    pause(dt);
end


