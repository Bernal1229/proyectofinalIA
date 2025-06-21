data = readmatrix('dataset.csv');

theta = data(:, 1);   % Ángulo (rad)
omega = data(:, 2);   % Velocidad angular (rad/s)
tau   = data(:, 3);   % Torque (N·m)

dt = 0.01;
T = dt * (length(theta)-1);
t = 0:dt:T;

figure;

subplot(3,1,1);
plot(t, theta, 'LineWidth', 1.5);
ylabel('\theta (rad)');
title('Respuesta del sistema controlado');

subplot(3,1,2);
plot(t, omega, 'LineWidth', 1.5);
ylabel('\omega (rad/s)');

subplot(3,1,3);
plot(t, tau, 'LineWidth', 1.5);
ylabel('\tau (N·m)');
xlabel('Tiempo (s)');



% % =========================
% % Animación del péndulo
% % =========================
% 
% l = 0.2;  % Longitud del péndulo (en metros)
% 
% figure;
% for i = 1:10:length(theta)
%     clf;
% 
%     % Coordenadas del extremo del péndulo
%     x = l * sin(theta(i));
%     y = l * cos(theta(i));
% 
%     % Dibujo del péndulo
%     plot([0, x], [0, y], 'k-', 'LineWidth', 2);
%     hold on;
%     plot(x, y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% 
%     % Formato del gráfico
%     axis equal;
%     axis([-l l -l l]*1.2);
%     grid on;
%     title(sprintf('Pendulum Animation - t = %.2f s', t(i)));
%     xlabel('x (m)');
%     ylabel('y (m)');
% 
%     pause(0.01);
% end