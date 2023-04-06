clear all;
% =========================================================================
% IE3041 - Visualización de estabilización en 2D
% -------------------------------------------------------------------------
% Se considera el ejemplo de un sistema no lineal controlado en 2D para el
% cual se establece un punto de operación/equilibrio, se linealiza, evalúa
% su estabilidad, controlabilidad y luego se utiliza un controlador lineal
% para estabilizar el punto establecido.
% =========================================================================
%% Opciones de cálculos y visualización
% -------------------------------------------------------------------------
% Activa o desactiva el controlador
enable_control = true;
% El control afecta sólo a la segunda ecuación o a ambas
full_control = true;
% Activa o desactiva la visualización del diagrama de fase de la dinámica
% sin control
plot_pp_dynamics = true;
% Activa o desactiva la visualización del diagrama de fase del control
plot_pp_control = true;
% Activa o desactiva la visualización del diagrama de fase del sistema en
% lazo cerrado (luego del state feedback lineal)
plot_pp_closedloop = false;

% Punto de operación/equilibrio deseado
xss = [-2; 2];
% Condición inicial
x0 = [0; 0]; 
% Ganancia (adicional) para el controlador
ctrlgain = 1;


%% Cálculos, simulación y visualización
% -------------------------------------------------------------------------
% Definición de los campos vectoriales
if(full_control)
    dim_ctrl = 2;
    f = @(x, u) hom_dynamics(x) + u;
else
    dim_ctrl = 1;
    f = @(x, u) hom_dynamics(x) + [0; u];
end

h = @(x, u) outputs(x, u); % dummy para poder emplear linealización numérica

% Se encuentra el resto del punto de operación
fu = @(u) f(xss, u);
uss = fsolve(fu, randn(dim_ctrl, 1));

% Se obtiene la linealización, la estabilidad y la controlabilidad del
% punto de operación
[A, B, ~, ~] = loclin_best(f, h, xss, uss);
disp('Los eigenvalores de la linealización son: ');
disp(eig(A));
if(rank(ctrb(A,B)) == 2)
    disp('La linealización es completamente controlable.');
else
    disp('La linealización no es controlable, especifique otro punto de operación.');
end

% Se estabiliza la linealización mediante LQR
Q = eye(numel(xss));
R = eye(numel(uss));
Klqr = ctrlgain * lqr(A, B, Q, R);

% Campo vectorial del sistema no lineal controlado (para visualización del
% diagrama de fase)
fctrl = @(x, u) f(x, -Klqr*(x - xss) + uss);

if(full_control)
    uctrl = @(x, u) -Klqr * (x - xss) + uss;
else
    uctrl = @(x, u) [0; -Klqr * (x - xss) + uss];
end

% Simulación
t_0 = 0;
t_f = 20;
dt = 0.01;
K = (t_f-t_0) / dt;

x = x0;
X = zeros(numel(x), K+1);
X(:, 1) = x;

U = [];

for k = 1:K
    if(enable_control)
        u = -Klqr * (x - xss) + uss;
        U = [U, u];
    else
        u = zeros(dim_ctrl, 1);
    end

    x = x + f(x, u)*dt;
    if((abs(x(1)) > 5.5) || (abs(x(2)) > 5.5))
        break;
    end

    if(norm(x - xss) < 0.01)
        break;
    end
    X(:, k+1) = x;
end

% Generación de figuras
fig = figure;
fig.WindowState = 'maximized';

if(enable_control)
    subplot(2,1,1);
end
hold on;

[mX1, mX2] = meshgrid(-6:0.01:6, -6:0.01:6);

if(plot_pp_dynamics)
    [mDX1, mDX2] = phaseportrait2d(f, mX1, mX2, 2);
    phasep_dynamics = streamslice(mX1, mX2, mDX1, mDX2, 4);
    set(phasep_dynamics, 'Color', [0.5, 0.5, 0.5]);
end

if(plot_pp_control)
    [mUX1, mUX2] = phaseportrait2d(uctrl, mX1, mX2, 2);
    phasep_control = streamslice(mX1, mX2, mUX1, mUX2, 4);
    set(phasep_control, 'Color', [0.3010, 0.7450, 0.9330]);
end

if(plot_pp_closedloop)
    [mFCX1, mFCX2] = phaseportrait2d(fctrl, mX1, mX2, 2);
    phasep_closedloop = streamslice(mX1, mX2, mFCX1, mFCX2, 4);
    set(phasep_closedloop, 'Color', [0.9290, 0.6940, 0.1250]);
end

xlabel('$x_1$', 'FontSize', 18, 'Interpreter','latex');
ylabel('$x_2$', 'FontSize', 18, 'Interpreter','latex');
xlim([-6, 6]);
ylim([-6, 6]);

scatter(xss(1), xss(2), 200, 'red', 'x', 'LineWidth', 2);
scatter(X(1,1), X(2,1), 25, 'k', 'filled');

trajplot = plot(X(1,1:2), X(2,1:2), 'k', 'LineWidth', 2);
vec = X(:, 1) - X(:, 2);
vec = vec / norm(vec);
Rr = [cos(pi/6), -sin(pi/6); sin(pi/6), cos(pi/6)];
Rl = [cos(-pi/6), -sin(-pi/6); sin(-pi/6), cos(-pi/6)];
rvec = X(:, 2) - 0.1*Rr*vec;
lvec = X(:, 2) - 0.1*Rl*vec;
arrowplot = plot([rvec(1), X(1, 2), lvec(1)], [rvec(2), X(2, 2), lvec(2)], 'k', 'LineWidth', 2);
hold off;

if(enable_control)
    subplot(2,1,2);
    time = 0:dt:(k-1)*dt;
    ctrlplot = plot(time(1:2), U(:, 1:2)', 'LineWidth', 1);
    xlabel('$t$', 'FontSize', 18, 'Interpreter','latex');
    ylabel('$\mathbf{u}(t)$', 'FontSize', 18, 'Interpreter','latex');
    grid minor;
end

for j = 3:k
    trajplot.XData = [trajplot.XData, X(1, j)];
    trajplot.YData = [trajplot.YData, X(2, j)];
    vec = X(:, j) - X(:, j-1);
    vec = vec / norm(vec);    
    rvec = X(:, j) - 0.1*Rr*vec;
    lvec = X(:, j) - 0.1*Rl*vec;
    arrowplot.XData = [rvec(1), X(1, j), lvec(1)]; 
    arrowplot.YData = [rvec(2), X(2, j), lvec(2)];

    if(enable_control)
        if(full_control)
            ctrlplot(1).XData = [ctrlplot(1).XData, time(j)];
            ctrlplot(1).YData = [ctrlplot(1).YData, U(1, j)];
            ctrlplot(2).XData = [ctrlplot(2).XData, time(j)];
            ctrlplot(2).YData = [ctrlplot(2).YData, U(2, j)];
        else
            ctrlplot.XData = [ctrlplot.XData, time(j)];
            ctrlplot.YData = [ctrlplot.YData, U(1, j)];
        end
    end

    pause(dt);
end