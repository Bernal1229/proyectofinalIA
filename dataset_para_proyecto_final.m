% Parámetros
m = 0.25;
l = 0.2;
g = 9.81;

Kp = 3;
Ki = 0.3;
Kd = 0.4;

% Tiempo
dt = 0.01;
T = 600;
t = 0:dt:T;

% Estado inicial
theta = 3.14;
omega = 0.0;

% PID interno
int_error = 0;
prev_error = 0;

% Dataset
data = [];
tiempo_inicio = 69;
tiempo_fin = tiempo_inicio + 0.1;
pi = 3.14

for i = 1:length(t)
    % Error
    % error = 0 - theta;  % setpoint = 0
    error = atan2(sin(0 - theta), cos(0 - theta));
    int_error = int_error + error * dt;
    der_error = (error - prev_error) / dt;
    
    % PID
    tau = Kp*error + Ki*int_error + Kd*der_error;
    

    % PERTURBACIÓN EXTERNA (elige una)
    tau_disturb = 0;
    if t(i) > 10 && t(i) < 10.1
        tau_disturb = -0.5;
    end
    if t(i) > 11 && t(i) < 11.1
        tau_disturb = -0.55;
    end
    if t(i) > 13 && t(i) < 13.1
        tau_disturb = -0.58;
    end
    if t(i) > 15 && t(i) < 15.1
        tau_disturb = 0.5;
    end
    if t(i) > 17 && t(i) < 17.1
        tau_disturb = 0.55;
    end
    if t(i) > 19 && t(i) < 19.1
        tau_disturb = 0.58;
    end
    if t(i) > 21 && t(i) < 21.1
        tau_disturb = 0.7;
    end
    if t(i) > 23 && t(i) < 23.1
        tau_disturb = -0.6;
    end
    if t(i) > 26 && t(i) < 26.1
        theta = 3.14;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;


    end
    if t(i) > 35 && t(i) < 35.1
        theta = 3.14/4;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;


    end
    if t(i) > 39 && t(i) < 39.1
        theta = -3.14/4;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;

    end
    if t(i) > 45 && t(i) < 45.1
        theta = 2;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;

    end

    if t(i) > 60 && t(i) < 60.1
        theta = 3.14;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;
    end
    
    if t(i) > tiempo_inicio && t(i) < tiempo_fin
        theta = pi;
        omega = 0;
        error = atan2(sin(0 - theta), cos(0 - theta));
        prev_error = 0;
        int_error = 0;
        int_error = int_error + error * dt;
        der_error = (error - prev_error) / dt;
        tau = Kp*error + Ki*int_error + Kd*der_error;
        tiempo_inicio = tiempo_inicio + 9;
        tiempo_fin = tiempo_inicio + 0.1;
        pi = -pi;
    end
    tau = max(min(tau, 0.3), -0.3);  % limitar a ±5 N·m

    % Torque total
    tau_total = tau + tau_disturb;

    % Dinámica del sistema
    dtheta = omega;
    domega = (g/l)*sin(theta) + (1/(m*(l^2)))*tau_total;
    
    % Integrar (Euler)
    theta = theta + dtheta * dt;
    theta = atan2(sin(theta), cos(theta));
    omega = omega + domega * dt;
    
    %Utilizar el camino corto en caso de pasar el angulo 2pi
    % theta = atan2(sin(theta), cos(theta));


    % Guardar datos
    data(end+1, :) = [theta, omega, tau];
    
    % Actualizar error
    prev_error = error;
end

% Umbrales de estabilidad (ajustables)
epsilon_theta = 0.2;   % radianes
epsilon_omega = 0.2;   % rad/s

% Eliminar filas donde el sistema ya está muy cerca del equilibrio
indices_validos = abs(data(:,1)) > epsilon_theta | abs(data(:,2)) > epsilon_omega;
data_filtrado = data(indices_validos, :);

header = {'angulo', 'velocidad', 'torque'};
% Guardar dataset
writecell(header, 'dataset.csv');
writematrix(data_filtrado, 'dataset.csv','WriteMode', 'append')
